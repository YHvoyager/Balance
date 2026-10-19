#include "main.h"
#include "usart.h"
#include "ESP01S.h"
#include "Vofa.h"           /* ★ 新增：接收到的字节要喂给 Vofa 的分行器 */
#include <string.h>

/* 发送缓冲区大小 */
#define ESP01S_TX_BUFFER_SIZE   128

/* SLIP 的四个特殊字节（RFC 1055） */
#define SLIP_END        0xC0U   /* 帧界 */
#define SLIP_ESC        0xDBU   /* 转义符 */
#define SLIP_ESC_END    0xDCU   /* 代表原字节 0xC0 */
#define SLIP_ESC_ESC    0xDDU   /* 代表原字节 0xDB */

/* ESP01S 挂在哪条串口上，只改这一行就能换 */
#define ESP01S_UART_HANDLE      huart1

/* ============================ 发送 ============================ */

/* ★ 必须放全局：HAL_UART_Transmit_DMA 是异步的，它只把地址交给 DMA 就返回，
   真正的移位输出在后面几百微秒才做。放栈上会被下一轮循环改掉。 */
static uint8_t ESP01S_TxBuffer[ESP01S_TX_BUFFER_SIZE];

/* ============================ 接收 ============================ */

/*
 * ★★ 接收方案（2026-09-29 换版：改成 HAL_UARTEx_ReceiveToIdle_DMA）
 *
 *   旧版：HAL_UART_Receive_DMA() 收 128 字节
 *         + 手工 __HAL_UART_ENABLE_IT(..., UART_IT_IDLE)
 *         + 自己在 stm32f1xx_it.c 的 USART1_IRQHandler() 里补一句 ESP01S_IRQHandler()
 *         + 中断里还要先 HAL_UART_AbortReceive() 把收端停干净（让 RxState 回到 READY）
 *           才能重新武装，否则 HAL_UART_Receive_DMA() 直接 return HAL_BUSY（静默空操作）
 *         + 长度要自己用 __HAL_DMA_GET_COUNTER() 算
 *
 *   新版：HAL_UARTEx_ReceiveToIdle_DMA() —— IDLE 是 HAL 内建的。
 *         空闲时它【自动停 DMA + 把 RxState 置回 READY + 回调】，
 *         并且把"这一批收了多少字节"直接当回调参数给你。
 *         ⇒ 不用手工开 IDLEIE、不用自己写 IRQHandler、
 *           stm32f1xx_it.c 里那行补丁可以删掉、也不用再自己算 CNDTR。
 *
 *   ★ 用法要点（两条，都是必踩的坑）
 *
 *   ① 给它【大缓冲】，不要给 1 字节。
 *      参考代码 FarUart 里写的是 ReceiveToIdle_DMA(&huart1, &rx_byte, 1) ——
 *      缓冲只有 1 字节 ⇒ 每收 1 个字节就回调一次 + 重装一次。
 *      @115200 字节间隔只有 87µs，而重装 DMA（stop/start）要几十µs
 *      ⇒ CPU 被中断淹没，稍微慢一点就丢字节（ORE）。
 *      给 128 字节缓冲后，回调频率降到"每帧一次"（因为 IDLE 只在总线空闲时才来）。
 *
 *   ② ★ 启动后必须关掉 DMA 的【半传输中断】(HT)。
 *      ReceiveToIdle_DMA 默认会开 HT：128 字节的缓冲 ⇒ 收满 64 字节先回调一次
 *      RxEventCallback(Size=64)，收满 128 再回调一次 (Size=128)。
 *      如果回调里不做区分，前 64 字节就会被【处理两遍】——
 *      现象是"命令被执行两次"，而且不报错，极难查。
 *      一行 __HAL_DMA_DISABLE_IT() 就能根治（见 esp_start_rx 里）。
 *      （源码依据见 HAL_UARTEx_RxEventCallback() 里的注释；
 *        那里还有一行兜底判断，双保险。）
 */

/* DMA 落点缓冲。每次重新武装，DMA 都从 [0] 开始写。 */
static uint8_t ESP01S_RxBuffer[ESP01S_RX_BUFFER_SIZE];

/*
 * 帧搬运暂存区。
 * ★ 仍然必要：HAL_UARTEx 的回调一返回，我们就要重新武装 DMA，
 *   而重装之后 DMA 立刻会从 RxBuffer[0] 开始写 —— 会把刚收到的数据盖掉。
 *   所以回调里必须先把这一段 memcpy 走。
 *   （新链路只用 ESP01S_RawFrame 里这份副本喂 Vofa；
 *     留着它也让老的 ESP01S_GetFrame() 保持可用，不影响。）
 */
static uint8_t           ESP01S_RawFrame[ESP01S_RX_BUFFER_SIZE];  // 原始数据，未解码
static volatile uint16_t ESP01S_RawLen     = 0;                   // 原始数据长度
static volatile uint8_t  ESP01S_FrameReady = 0;                   // 是否有新数据，0=无，1=有

/* 诊断计数：出问题时先看这两个。都为 0 才算链路健康。 */
volatile uint32_t ESP01S_OreCount  = 0;   /* 串口溢出(ORE)：读得太慢，丢字节了 */
volatile uint32_t ESP01S_OverCount = 0;   /* 解出来的帧超过 FRAME_MAX，已被丢弃 */

/*
 * 重新武装接收。
 * ★ IDLE / 缓冲写满 / 出错之后都必须调它 —— 不调就再也收不到数据了，
 *   而且不会有任何报错，表现为"串口突然哑了"。
 */
static HAL_StatusTypeDef esp_start_rx(void)
{
    HAL_StatusTypeDef st;

    st = HAL_UARTEx_ReceiveToIdle_DMA(&ESP01S_UART_HANDLE,
                                      ESP01S_RxBuffer,
                                      ESP01S_RX_BUFFER_SIZE);

    /* ★★ 关掉半传输中断 —— 不关的话前一半字节会被处理两遍（见文件头 ②） */
    __HAL_DMA_DISABLE_IT(ESP01S_UART_HANDLE.hdmarx, DMA_IT_HT);

    return st;
}

/*
 * 收到一批字节。★ 在中断上下文（HAL 的回调）里运行。
 *   ① 先把这一批 memcpy 走 —— 重装 DMA 会立刻覆盖 RxBuffer
 *   ② 原样喂给 Vofa 的分行器
 *
 * ★ 为什么不需要任何解码：
 *   新链路里 01S 是【透明桥】，它把从 C3 空中收到的字节原封不动吐到这条串口上。
 *   所以这里拿到的就是 PC 发来的一行行文本（"kp=1.2\n"），不是 SLIP 帧。
 *   原来的 SLIP 解码器在这条链路上永远解不出东西（它在找 0xC0，而文本里没有）。
 *
 * ★ Vofa_FeedByte 是中断安全的：它只改 8 位变量，Cortex-M3 上是单指令原子访问。
 */
static void esp_on_rx_batch(uint16_t n)
{
    if ((n == 0U) || (n > ESP01S_RX_BUFFER_SIZE)) return;

    memcpy(ESP01S_RawFrame, ESP01S_RxBuffer, n);
    ESP01S_RawLen     = n;
    ESP01S_FrameReady = 1;

    /* ★ 新链路的接收路径：一个字节不落地喂进去，按 '\n' 分行 */
    for (uint16_t i = 0U; i < n; i++)
    {
        Vofa_FeedByte(ESP01S_RawFrame[i]);
    }
}

void ESP01S_Init(void)
{
    /* 1. 停掉可能在跑的 DMA，清旧错误 */
    HAL_UART_DMAStop(&ESP01S_UART_HANDLE);
    __HAL_UART_CLEAR_OREFLAG(&ESP01S_UART_HANDLE);

    ESP01S_FrameReady = 0;
    ESP01S_RawLen     = 0;
    ESP01S_OreCount   = 0;
    ESP01S_OverCount  = 0;

    /* 2. 武装接收。
       ★ 这里不再需要 __HAL_UART_ENABLE_IT(..., UART_IT_IDLE) ——
         HAL_UARTEx_ReceiveToIdle_DMA() 内部会自己打开 IDLE 中断。 */
    esp_start_rx();
}

/*
 * ★ 新版接收回调。
 *   触发条件：① 总线空闲（IDLE）  ② 缓冲被写满（TC）  ③ ★半传输(HT) —— 下面挡掉
 *   Size = 这一批收了多少字节。
 *
 *   ★★ 必须在返回前重新武装，否则只能收到第一批数据 —— 这是
 *      "换成 ReceiveToIdle 之后串口突然只收一次" 的唯一原因。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART1) return;

    /*
     * ★ 兜底：把"半传输(HT)"事件挡掉。这是 ReceiveToIdle 最隐蔽的一个坑。
     *
     *   源码依据（F1 HAL v1.8.7）：
     *     · HAL_DMA_Start_IT() 里 —— `if (hdma->XferHalfCpltCallback != NULL)`
     *       就"连 HT 一起开"（DMA_IT_TC | DMA_IT_HT | DMA_IT_TE）；UART 收端必然
     *       设了 XferHalfCpltCallback = UART_DMARxHalfCplt，所以 HT 是开着的。
     *     · UART_DMARxHalfCplt() 里 —— 当 ReceptionType == TOIDLE 时，它调用的
     *       就是【这个回调】，且传进来的 Size 恒等于 RxXferSize / 2。
     *   ⇒ 128 字节缓冲：收满 64 字节先回调一次(Size=64)，IDLE 到齐再回调一次
     *     (Size=总数) ⇒ 前 64 字节被喂两遍。
     *     现象是"命令被执行两次"、"某几行文本重复出现"，而且【不报错】，极难查。
     *
     *   esp_start_rx() 里已经 __HAL_DMA_DISABLE_IT(..., DMA_IT_HT) 把它关了，
     *   但那是"武装之后"才关的，理论上有极小窗口（恰好正在爆发式收字节时）
     *   会被 HT 抢进来一次。这一行是兜底：一次比较，换"彻底不可能重复"。
     *
     *   ★ HT 事件【不需要】重新武装 —— 它既不停 DMA 也不改 RxState，
     *     后面的 IDLE / 缓冲写满(TC) 会正常再来。
     *     （真在这里调 esp_start_rx() 也只会拿到 HAL_BUSY，是空操作。）
     */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) return;

    esp_on_rx_batch(Size);

    esp_start_rx();                 /* ★ 关键：不重装就只收一次 */
}

/* 溢出 / 帧错误回调。不清标志 + 不重新武装 = 串口永久死掉。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    ESP01S_OreCount++;
    __HAL_UART_CLEAR_OREFLAG(huart);

    /* ★ 出错后 RxState 可能停在 BUSY_RX，直接重装会是空操作 ——
       先 AbortReceive 让它回到 READY，再武装。 */
    HAL_UART_AbortReceive(huart);
    esp_start_rx();
}

/* 把 ESP01S_RawFrame 里的【第一帧】解出来写进 buffer。
   *length 返回 payload 字节数；没解出完整帧则置 0。

   ★ 注意：这是给【旧链路（01S 做 SLIP 网关）】用的。
     在新链路（01S 是透明桥）下，串口上跑的是裸文本、没有 0xC0，
     这个解码器会全程走 "帧外一律跳过"，每次都返回 0 —— 这不是坏了，是用不上了。
     新链路请改用 Vofa_GetLine() / Vofa_GetFloat()。 */
void ESP01S_GetFrame(uint8_t *buffer, uint16_t *length)   /* SLIP 解码 */
{
    if (!ESP01S_FrameReady)
    {
        *length = 0;
        return;
    }

    uint16_t n = ESP01S_RawLen;
    ESP01S_FrameReady = 0;          /* ★ 先取走数据再清标志：期间来新帧也不会丢 */

    uint16_t outCount    = 0;
    uint8_t  insideFrame = 0;       /* 0 = 还在找帧头 */
    uint8_t  gotEsc      = 0;       /* 1 = 上一个字节是 0xDB，本字节要还原 */

    for (uint16_t i = 0; i < n; i++)
    {
        uint8_t byte = ESP01S_RawFrame[i];

        if (byte == SLIP_END)
        {
            if (!insideFrame) { insideFrame = 1; outCount = 0; }
            else              { *length = outCount; return; }   /* 第二个 C0 = 帧尾 */
        }
        else if (!insideFrame)
        {
            continue;   /* ★ 帧外脏数据一律跳过 —— 开机乱码就是靠这一跳被吃掉的 */
        }
        else if (gotEsc)
        {
            if      (byte == SLIP_ESC_END) { buffer[outCount++] = SLIP_END; }
            else if (byte == SLIP_ESC_ESC) { buffer[outCount++] = SLIP_ESC; }
            /* 其它组合按 RFC 1055 属于非法转义，丢掉不写 */
            gotEsc = 0;
        }
        else if (byte == SLIP_ESC)
        {
            gotEsc = 1;
        }
        else
        {
            buffer[outCount++] = byte;
        }

        if (outCount >= ESP01S_FRAME_MAX)   /* ★ 越界保护：宁可丢帧，不许踩内存 */
        {
            ESP01S_OverCount++;
            *length = 0;
            return;
        }
    }

    *length = 0;   /* 走到这 = 这段话里没等到帧尾 */
}

/* ============================ 发送 ============================ */

/*
 * ★★ 新链路下【不要再调用这个函数】。
 *
 *   它做 SLIP 编码（首尾各加 0xC0、特殊字节转义），而 01S 现在是透明桥，
 *   会把这一串字节原封不动送到 VOFA+ —— VOFA+ 收到的是
 *   "C0 48 65 6C 6C 6F ... C0" 这种带 0xC0 的垃圾，波形区全乱。
 *
 *   往 PC 发数据请用 Vofa_Send()（裸文本 + '\n'）。
 *
 *   ★ 还有一个理由：这个函数走 DMA 发送，Vofa_Send() 走阻塞发送，
 *     两者都用 huart1 的 TX —— 同时开会在同一条串口上打架。
 */
void ESP01S_Send(const uint8_t *data, uint16_t len)
{
    /* ★ 上界修正：最坏情况每个字节都转义成 2 字节，再加首尾 2 个 C0，
       所以上限是 (128-2)/2 = 63。
       原来是 len > 128-4（=124），写 124 字节会输出 250 字节 → 溢出 TxBuffer。 */
    if ((len == 0U) || (len > (ESP01S_TX_BUFFER_SIZE - 2U) / 2U)) return;

    uint16_t pos = 0;  // 发送指针

    ESP01S_TxBuffer[pos++] = SLIP_END;    /* 起始 END */

    for (uint16_t i = 0; i < len; i++)
    {
        if      (data[i] == SLIP_END) { ESP01S_TxBuffer[pos++] = SLIP_ESC; ESP01S_TxBuffer[pos++] = SLIP_ESC_END; }
        else if (data[i] == SLIP_ESC) { ESP01S_TxBuffer[pos++] = SLIP_ESC; ESP01S_TxBuffer[pos++] = SLIP_ESC_ESC; }
        else                          { ESP01S_TxBuffer[pos++] = data[i]; }
    }

    ESP01S_TxBuffer[pos++] = SLIP_END;    /* 结束 END */

    /* 等上一次 DMA 发送完成 */
    while (huart1.gState != HAL_UART_STATE_READY) { }

    HAL_UART_Transmit_DMA(&ESP01S_UART_HANDLE, ESP01S_TxBuffer, pos);
}