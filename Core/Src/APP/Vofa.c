/**
  ******************************************************************************
  * @file    Vofa.c
  * @brief   VOFA+ FireWater 遥测输出 + 上位机命令行解析
  *
  * 【这一版为什么长这样 —— 三个"踩过才知道"的决定】
  *
  * ① 为什么自己写 ftoa，不用 snprintf("%.3f")
  *    CubeMX 新建工程默认链 newlib-nano，它的 printf 不带浮点支持，
  *    写 %.3f 得不到数字。你可以加 -u _printf_float（flash 约 +6KB），
  *    但这里选了"自己写"：不依赖链接选项、代码小、换工程也带着走，
  *    而且 FireWater 本来就是文本协议，转字符串这一步省不掉。
  *
  * ② 为什么自己解析命令，不用 sscanf("%f")
  *    newlib-nano 的 scanf 系列默认同样不带浮点支持，会静默失败
  *    （返回值是 0，你会以为"对端没发命令"）。自己解析只认数字和小数点，
  *    逻辑比 sscanf 简单得多，也不怕前导空白、多一个逗号之类的差异。
  *
  * ③ 为什么行缓冲要排队（不是只有一个）
  *    中断在收，主循环在取。主循环一轮要刷 OLED（I2C 400kHz 传 1024 字节
  *    约 25ms），这 25ms 里对端可能已经发来两三行了。只留一个缓冲的话，
  *    后来的行会把还没被取走的那行覆盖掉 —— 表现为"偶尔丢一条命令"，
  *    而且不报错。所以用一个 4 行的小队列，满了才丢并计数。
  *
  * 【中断安全说明（重要）】
  *    s_head 只在 Vofa_FeedByte() 里改（中断上下文）
  *    s_tail 只在 Vofa_GetLine() 里改（主循环上下文）
  *    提交一行时，先把数据 memcpy 进 s_q[s_head]、写好长度，
  *    【最后才推进 s_head】—— 主循环看到 s_head 前进就认为那行已经完整。
  *    两边读写的都是 8 位变量，在 Cortex-M3 上是单指令原子访问，不需要临界区。
  *
  * 【怎么用 —— 两个方向的完整例子】
  *
  *   --- 发送遥测（放进主循环，用 HAL_GetTick 节流到 100Hz）------------
  *
  *       char  line[64];
  *       float tele[6];
  *
  *       if (HAL_GetTick() - t_last >= 10) {          // 100 Hz
  *           t_last = HAL_GetTick();
  *           tele[0] = g_target;      // 期望值
  *           tele[1] = g_actual;      // 实际值
  *           tele[2] = g_output;      // PID 输出
  *           tele[3] = g_err;         // 误差
  *           tele[4] = speedL;        // 左轮速度
  *           tele[5] = speedR;        // 右轮速度
  *           Vofa_Send(NULL, tele, 6, 3);              // 发 "1.234,5.678,...\n"
  *       }
  *
  *   --- 接收命令 -------------------------------------------------------
  *
  *       // 1) 这一行加在 ESP01S.c 的接收中断里（取出 DMA 数据之后）：
  *       //        Vofa_FeedByte(byte);
  *       //    它和原来的 SLIP 解码器互不干扰：
  *       //    SLIP 解码器"帧外一律跳过"，而 ASCII 行里没有 0xC0；
  *       //    反过来行解析器只认 '\n'，SLIP 二进制帧里不会出现 '\n'。
  *
  *       // 2) 主循环里取行、解析：
  *       char   cmd[64];
  *       float  f;
  *       while (Vofa_GetLine(cmd, sizeof(cmd)) > 0) {
  *           if      (Vofa_GetFloat(cmd, "kp", &f)) g_kp = f;
  *           else if (Vofa_GetFloat(cmd, "ki", &f)) g_ki = f;
  *           else if (Vofa_GetFloat(cmd, "kd", &f)) g_kd = f;
  *           else if (Vofa_GetFloat(cmd, "target", &f)) g_target = f;
  *           // 不认识的命令直接忽略 —— 别报错，上位机偶尔会发别的东西
  *       }
  *
  *   --- 体检表（放在 OLED 上）-----------------------------------------
  *
  *       snprintf(line, sizeof(line), "RX=%lu", (unsigned long)Vofa_StatLines());
  *       OLED_PrintASCIIString(0, 10, line, &afont8x6, OLED_COLOR_NORMAL);
  *       snprintf(line, sizeof(line), "QD=%lu", (unsigned long)Vofa_StatQueueDrop());
  *       ...
  *      ★ RX 不涨 = 串口那头没东西进来（01S/C3 侧的问题，不是 STM32 的问题）
  *      ★ QD 在涨 = 主循环取行太慢（OLED 刷屏占了太多时间）
  *      ★ 超长在涨 = 对端没按行发（0xC0 那套数据混进来了？）
  ******************************************************************************
  */

#include "Vofa.h"
#include "usart.h"
#include "dma.h"
#include <string.h>

/* ============================================================ 内部状态 */

/* 正在攒的这一行（中断上下文写） */
static char     s_cur[VOFA_LINE_MAX];
static uint16_t s_curLen = 0;
static uint8_t  s_drop   = 0;      /* 1 = 本行已超长，丢到 '\n' 为止 */

/* 已提交、等待主循环取走的行队列 */
static char             s_q[VOFA_QUEUE_LINES][VOFA_LINE_MAX];
static volatile uint8_t s_qLen[VOFA_QUEUE_LINES];

static volatile uint8_t s_head = 0;    /* 下一个可提交的槽（只在中断里改） */
static volatile uint8_t s_tail = 0;    /* 下一个可读的槽  （只在主循环里改） */

/* 体检计数 */
static volatile uint32_t s_lines   = 0;    /* 成功收到的完整行数 */
static volatile uint32_t s_qDrop   = 0;    /* 队列满丢掉的行数 */
static volatile uint32_t s_tooLong = 0;    /* 超长丢掉的行数 */

/* ============================================================ 初始化 */

void Vofa_Init(void)
{
    s_curLen = 0;
    s_drop   = 0;
    s_head   = 0;
    s_tail   = 0;
    s_lines   = 0;
    s_qDrop   = 0;
    s_tooLong = 0;

    memset(s_cur, 0, sizeof(s_cur));
    memset(s_q,   0, sizeof(s_q));
}

/* ==================================================== 接收：分行 */

/*
 * 把 s_cur 提交到队列尾部。
 * ★ 顺序很关键：先拷数据、写长度，【最后】才推进 s_head。
 *   主循环看到 s_head 变了才会去读这一槽，所以它读到的必然是完整的一行。
 */
static void vofa_pushLine(void)
{
    const uint8_t next = (uint8_t)((s_head + 1u) % VOFA_QUEUE_LINES);

    if (next == s_tail) {
        s_qDrop++;                 /* 队列满：丢这一行并留痕，绝不覆盖未读的行 */
        return;
    }

    memcpy(s_q[s_head], s_cur, s_curLen);
    s_q[s_head][s_curLen] = '\0';
    s_qLen[s_head] = (uint8_t)s_curLen;

    s_head = next;                 /* ★ 提交点 */
    s_lines++;
}

void Vofa_FeedByte(uint8_t b)
{
    if (b == '\n') {
        /* 空行（连着两个 '\n'）不提交 —— 那只是线路空闲，不是一条命令 */
        if (!s_drop && s_curLen > 0u) {
            vofa_pushLine();
        }
        s_drop   = 0;
        s_curLen = 0;
        return;
    }

    /*
     * 顺手吃掉 '\r'，让 "\r\n" 结尾也能正常工作。
     * 有些串口工具 / 上位机会自动补 CRLF，不去掉的话
     * 字段末尾会多一个不可见字符，解析时看着像"数值解析失败"。
     * FireWater 是文本协议，'\r' 不会是有效数据，吃掉没有副作用。
     */
    if (b == '\r') return;

    if (s_drop) return;            /* 本行已经超长，剩下的字节全丢 */

    if (s_curLen >= (VOFA_LINE_MAX - 1u)) {
        s_drop = 1;
        s_tooLong++;
        return;
    }

    s_cur[s_curLen++] = (char)b;
}

uint16_t Vofa_GetLine(char *out, uint16_t max)
{
    uint16_t n;

    if (out == NULL || max == 0u) return 0;

    if (s_tail == s_head) {        /* 队列空 */
        out[0] = '\0';
        return 0;
    }

    n = (uint16_t)s_qLen[s_tail];
    if (n >= max) n = (uint16_t)(max - 1u);   /* 调用者的缓冲更小就截断 */

    memcpy(out, s_q[s_tail], n);
    out[n] = '\0';

    s_tail = (uint8_t)((s_tail + 1u) % VOFA_QUEUE_LINES);

    return n;
}

/* ==================================================== 发送：float → 文本 */

/*
 * 把一个 float 写成最多 6 位小数。
 * ★ 末尾【不加】'\0'，返回写了多少字节 —— 调用者自己接下一个字段。
 *   这样多个字段可以直接首尾相接，不用一次次 strlen。
 */
uint16_t Vofa_Ftoa(char *dst, float v, uint8_t dec)
{
    uint16_t n = 0;
    uint32_t mul = 1u;
    uint32_t u;
    uint32_t ip, fp;
    uint8_t  i;
    char     tmp[12];
    uint8_t  t = 0;

    if (dst == NULL) return 0;
    if (dec > 6u) dec = 6u;

    if (v < 0.0f) {
        dst[n++] = '-';
        v = -v;
    }

    for (i = 0; i < dec; i++) mul *= 10u;

    /* 钳位：防止 v 特别大时 v*mul 溢出 uint32_t（正常 PID 数据远到不了） */
    if (v > 99999999.0f) v = 99999999.0f;

    u = (uint32_t)(v * (float)mul + 0.5f);      /* 四舍五入到第 dec 位 */

    ip = u / mul;                                /* 整数部分 */
    fp = u % mul;                                /* 小数部分（整数形式） */

    /* 整数部分倒着填，再倒回来 */
    do {
        tmp[t++] = (char)('0' + (ip % 10u));
        ip /= 10u;
    } while (ip != 0u);

    while (t > 0u) dst[n++] = tmp[--t];

    if (dec > 0u) {
        dst[n++] = '.';
        /*
         * 小数部分要【补前导零】：0.05 的 fp 是 5，必须写成 "05"。
         * 从右往左填，所以不会漏零。
         */
        for (i = dec; i > 0u; i--) {
            dst[n + i - 1u] = (char)('0' + (fp % 10u));
            fp /= 10u;
        }
        n = (uint16_t)(n + dec);
    }

    return n;
}

uint16_t Vofa_Send(const char *name, const float *data, uint8_t n, uint8_t dec)
{
    char     buf[VOFA_LINE_MAX];
    uint16_t len = 0;
    uint8_t  i;

    if (data == NULL || n == 0u) return 0;

    /* 可选的名字前缀，FireWater 格式是  名称:值0,值1,... */
    if (name != NULL) {
        while (*name != '\0' && len < 16u) buf[len++] = *name++;
        if (len < (VOFA_LINE_MAX - 1u)) buf[len++] = ':';
    }

    for (i = 0; i < n; i++) {
        /*
         * 预算检查：一个 float 最坏写成 "-99999999.999999" 是 15 字节，
         * 再加一个逗号=16，再加 '\n' 和保险位=18。留够就写，留不够就停。
         * ★ 宁可少发几个通道，也不要写越界 —— 这是 C 语言，越界不会报错。
         */
        if ((uint16_t)(len + 18u) >= VOFA_LINE_MAX) break;

        len = (uint16_t)(len + Vofa_Ftoa(&buf[len], data[i], dec));

        if ((uint8_t)(i + 1u) < n) buf[len++] = ',';
    }

    buf[len++] = '\n';

    /*
     * ★ 阻塞发送。100Hz / 6 通道 / 3 位小数约 56 字节，@115200 约 4.9ms。
     *   所以 PID 必须跑在固定周期的定时器中断里，不能放主循环 ——
     *   否则这 4.9ms 会把 PID 的 dt 拖得忽长忽短，积分和微分全错。
     */
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 20u);

    return len;
}

uint16_t Vofa_SendAsync(const char *name, const float *data, uint8_t n, uint8_t dec)
{
    static uint8_t tx_buf[VOFA_LINE_MAX];
    char value_buf[20];
    uint8_t irq_state;
    uint16_t len = 0;
    uint16_t value_len;
    uint8_t i;

    if (data == NULL || n == 0u || huart1.gState != HAL_UART_STATE_READY) return 0u;
    irq_state = __get_PRIMASK();

    if (name != NULL) {
        while (*name != '\0' && len < 16u) tx_buf[len++] = (uint8_t)*name++;
        if (len < (VOFA_LINE_MAX - 1u)) tx_buf[len++] = ':';
    }

    for (i = 0u; i < n; i++) {
        value_len = Vofa_Ftoa(value_buf, data[i], dec);
        if ((uint16_t)(len + value_len + (i > 0u ? 1u : 0u) + 1u) > VOFA_LINE_MAX) break;
        if (i > 0u) tx_buf[len++] = ',';
        memcpy(&tx_buf[len], value_buf, value_len);
        len = (uint16_t)(len + value_len);
    }
    tx_buf[len++] = '\n';

    __disable_irq();
    if (huart1.gState != HAL_UART_STATE_READY) {
        if (irq_state == 0u) __enable_irq();
        return 0u;
    }
    if (HAL_UART_Transmit_DMA(&huart1, tx_buf, len) != HAL_OK) {
        if (irq_state == 0u) __enable_irq();
        return 0u;
    }
    if (irq_state == 0u) __enable_irq();
    return len;
}

/* ==================================================== 解析 "key=数值" */

/*
 * 极简 atof：只认 [+-]?digits[.digits]
 * 返回 1 = 至少读到了一个数字。
 */
static uint8_t vofa_atof(const char *s, float *out)
{
    int      sign = 1;
    float    v    = 0.0f;
    uint8_t  any  = 0;

    if (s == NULL || out == NULL) return 0;

    if (*s == '-')      { sign = -1; s++; }
    else if (*s == '+') { s++; }

    while (*s >= '0' && *s <= '9') {
        v = v * 10.0f + (float)(*s - '0');
        s++;
        any = 1;
    }

    if (*s == '.') {
        uint32_t d = 0u, m = 1u;
        s++;
        while (*s >= '0' && *s <= '9') {
            d = d * 10u + (uint32_t)(*s - '0');
            m *= 10u;
            s++;
            any = 1;
        }
        /* 用整数除法而不是逐位乘 0.1f —— 后者会累积浮点误差 */
        v += (float)d / (float)m;
    }

    if (!any) return 0;

    *out = (sign < 0) ? -v : v;
    return 1;
}

uint8_t Vofa_GetFloat(const char *line, const char *key, float *out)
{
    const char *p;
    size_t      klen;

    if (line == NULL || key == NULL || out == NULL) return 0;

    klen = strlen(key);
    if (klen == 0u) return 0;

    /*
     * 找 key。要求它前面是行首或分隔符 ——
     * 否则查 "kp" 会在 "kkp=1" 这种串里误命中。
     */
    p = line;
    for (;;) {
        p = strstr(p, key);
        if (p == NULL) return 0;
        if (p == line || p[-1] == ' ' || p[-1] == '\t' ||
            p[-1] == ',' || p[-1] == ';') {
            break;
        }
        p += klen;
    }

    p += klen;

    /* 跳过 '=' ':' 和空白，剩下的交给 atof */
    while (*p == ' ' || *p == '\t' || *p == '=' || *p == ':') p++;

    return vofa_atof(p, out);
}

/* ==================================================== 体检计数 */

uint32_t Vofa_StatLines(void)      { return s_lines;   }
uint32_t Vofa_StatQueueDrop(void)  { return s_qDrop;   }
uint32_t Vofa_StatTooLong(void)    { return s_tooLong; }
