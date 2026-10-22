/**
  ******************************************************************************
  * @file    Vofa.h
  * @brief   VOFA+ FireWater 遥测输出 + 上位机命令行解析
  *
  * 这套接口解决三件事：
  *   ① 把若干 float 拼成一行 FireWater 文本发出去（遥测）
  *   ② 把串口收到的字节按 '\n' 切成行（命令）
  *   ③ 从一行里取出 "key=数值" 里的数值（不用 sscanf）
  *
  * 配套固件：esp-now-demo/espnow-bridge（ESP-01S）+ c3-link（C3 mini）
  * 上位机：VOFA+，连接方式选串口（C3 的 COM 口），控件协议选 FireWater
  *
  * 接进 Balance 工程只要三步（都是纯新增，不动任何现有文件）：
  *   1. 把 Vofa.c 放到 Core/Src/，把 Vofa.h 放到 Core/Inc/
  *   2. 在 CMakeLists.txt 的 target_sources 里加一行
  *        ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/Vofa.c
  *   3. 在 ESP01S.c 的接收中断里加一行 Vofa_FeedByte(byte);
  ******************************************************************************
  */

#ifndef __VOFA_H__
#define __VOFA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * 容量配置
 *   VOFA_LINE_MAX   : 一行最多多少字节（不含结尾补的 '\0'）
 *                     FireWater 6 通道、3 位小数约 56 字节，64 够用
 *   VOFA_QUEUE_LINES: 行队列深度（主循环来不及取时最多攒几行）
 *                     4 行 x 64 字节 = 256 字节 RAM
 * ------------------------------------------------------------------------ */
#define VOFA_LINE_MAX      64u
#define VOFA_QUEUE_LINES   4u

/* ---------------------------------------------------------------------------
 * 状态计数器（都是 32 位、只增不减，用来在 OLED 上做体检表）
 * ------------------------------------------------------------------------ */

/** 清空所有内部状态和计数。上电时调一次即可。 */
void Vofa_Init(void);

/* ---------------------------------------------------------------------------
 * 接收侧：串口字节 → 完整的一行
 * ------------------------------------------------------------------------ */

/**
  * @brief  喂一个字节进分行器。★ 在串口接收中断里调用。
  * @param  b 收到的字节
  * @note   遇到 '\n' 提交这一行；遇到 '\r' 直接吃掉（兼容 "\r\n" 结尾）。
  *         超长（>= VOFA_LINE_MAX-1）的行整行丢弃并计数，不会撑爆缓冲。
  * @note   本函数很快（普通字节路径只有一次存储），可以放在中断里。
  */
void Vofa_FeedByte(uint8_t b);

/**
  * @brief  取出一行。主循环里调。
  * @param  out 输出缓冲（会被补 '\0'）
  * @param  max 缓冲大小（含 '\0' 的位置）
  * @retval 这一行的长度（不含 '\0'）；0 = 还没有完整的行
  * @note   返回的内容【不含】行尾的 '\n' 和 '\r'。
  */
uint16_t Vofa_GetLine(char *out, uint16_t max);

/* ---------------------------------------------------------------------------
 * 发送侧：若干 float → 一行 FireWater
 * ------------------------------------------------------------------------ */

/**
  * @brief  拼一行并【阻塞】发出去。
  * @param  name 可选的流名前缀。给 NULL 就发 "1.0,2.0\n"；
  *              给 "d" 就发 "d:1.0,2.0\n"（VOFA+ 里更好认）
  * @param  data float 数组
  * @param  n    通道数（建议 4~6 个）
  * @param  dec  小数位数（建议 2~3；位数越多越费带宽）
  * @retval 实际发出的字节数
  * @note   ★ 用的是 HAL_UART_Transmit（阻塞）。100 Hz、6 通道 3 位小数
  *         约 56 字节，@115200 阻塞约 4.9 ms。所以【PID 不要放在主循环里跑】，
  *         要放在固定周期定时器中断里 —— 否则这段阻塞会拖乱 PID 的 dt。
  *         想彻底去掉阻塞：先 HAL_UART_Transmit_DMA() 然后判断
  *         huart1.gState == HAL_UART_STATE_READY 再发下一帧。
  */
uint16_t Vofa_Send(const char *name, const float *data, uint8_t n, uint8_t dec);

/**
  * @brief  把一个 float 写成十进制字符串。
  * @param  dst 输出缓冲（★ 末尾【不会】补 '\0'）
  * @param  v   数值
  * @param  dec 小数位数
  * @retval 写入的字节数
  * @note   ★ 为什么不用 snprintf("%.3f")：
  *         CubeMX 新建工程默认链的是 newlib-nano，它的 printf 【不带浮点支持】，
  *         写 %.3f 会打出空字符串或者只留一个 "f"。
  *         两个修法：加链接选项 -u _printf_float（flash 约 +6KB），
  *         或者用这个自己写的版本（推荐，不依赖链接选项，换工程也带着走）。
  */
uint16_t Vofa_Ftoa(char *dst, float v, uint8_t dec);

/* ---------------------------------------------------------------------------
 * 命令解析
 * ------------------------------------------------------------------------ */

/**
  * @brief  在一行里找 "key=数值" 并解析出浮点数。
  * @param  line 一行文本（Vofa_GetLine 取出来的）
  * @param  key  键名，例如 "kp"
  * @param  out  解析结果
  * @retval 1 = 找到了并解析成功；0 = 没找到 / 格式不对
  * @note   ★ 为什么不用 sscanf(line, "kp=%f", &f)：
  *         newlib-nano 的 scanf 系列默认【同样不带浮点支持】，
  *         加了 -u _printf_float 才会连带生效。为了不依赖链接选项，
  *         这里自己解析（只认可选符号、数字、一个小数点）。
  * @note   支持的写法（空白随意）：
  *             kp=1.2      kp 1.2      kp:1.2      kp=-0.35
  */
uint8_t Vofa_GetFloat(const char *line, const char *key, float *out);

/* ---------------------------------------------------------------------------
 * 体检计数器（放在 OLED 上，出问题时一眼看出断在哪一层）
 * ------------------------------------------------------------------------ */

/** 成功收到的完整行数（上行通不通） */
uint32_t Vofa_StatLines(void);

/** 队列满而丢掉的行数（主循环取行太慢） */
uint32_t Vofa_StatQueueDrop(void);

/** 超长被丢弃的行数（正常应恒为 0；在涨说明对端没按行发） */
uint32_t Vofa_StatTooLong(void);

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
