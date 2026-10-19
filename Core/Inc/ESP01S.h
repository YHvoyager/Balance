#ifndef ESP01S_H
#define ESP01S_H

#include <stdint.h>

/*
 * 接收缓冲大小（★ 2026-09-29 换版后它的角色变了）
 *
 *   现在它是 HAL_UARTEx_ReceiveToIdle_DMA() 的【DMA 落点缓冲】：
 *   串口收到多少字节就 DMA 多少个，总线一空闲（IDLE）HAL 就回调，
 *   并且把"这一批收了多少字节"当参数交出来。
 *
 *   ★ 故意给大一些（128）：缓冲越大，一次能收的一整段越长、回调次数越少。
 *     别学参考例程那样给 1 字节 —— 那等于每收一个字节就中断一次 + 重装一次 DMA，
 *     @115200 下字节间隔只有 87µs，而重装要几十µs，很快就把 CPU 吃满还会丢字节。
 */
#define ESP01S_RX_BUFFER_SIZE  128

/* 一帧 SLIP 解开后最多允许多少字节 payload。
   ★ 只在【旧链路】（01S 做 SLIP 网关）里用得到；
     新链路里 01S 是透明桥，串口上跑的是裸文本，用 Vofa_GetLine() 取行。 */
#define ESP01S_FRAME_MAX       64

void ESP01S_Init(void);

/* ★ 已删除 ESP01S_IRQHandler()
 *
 *   改用 HAL_UARTEx_ReceiveToIdle_DMA() 之后，IDLE 中断由 HAL 自己处理
 *   （HAL_UART_IRQHandler() 内部会调 HAL_UARTEx_RxEventCallback），
 *   不需要在 stm32f1xx_it.c 的 USART1_IRQHandler() 里再补一刀。
 *   ⇒ 记得把 stm32f1xx_it.c 里那句 ESP01S_IRQHandler(); 也删掉。
 */

/* 取一帧（SLIP 解码）。
   *length 会被置成 payload 字节数；暂时没有新帧时置 0。
   ★ buffer 至少要 ESP01S_FRAME_MAX 字节；函数内部有上界检查，不会越界写。

   ★ 这是【旧链路】的接口。新链路（01S 透明桥）下串口上是裸文本、没有 0xC0，
     这个解码器每次都会返回 0 —— 不是坏了，是用不上了。
     新链路请改用 Vofa_GetLine() / Vofa_GetFloat()。 */
void ESP01S_GetFrame(uint8_t *buffer, uint16_t *length);

/* ★ 新链路下【不要】调用它。
   它做 SLIP 编码（首尾加 0xC0），而 01S 是透明桥、会原样转发到 VOFA+，
   于是波形区里全是带 0xC0 的垃圾。
   往 PC 发数据请用 Vofa_Send()（裸文本 + '\n'）。详见 ESP01S.c 里的说明。 */
void ESP01S_Send(const uint8_t *data, uint16_t len);

/* ---------------------------------------------------------------------------
 * 诊断计数（定义在 ESP01S.c，这里是 extern 声明）
 *
 *   ★ 光在 .c 里定义成全局变量，别的 .c 是【看不见】的 —— 必须在头文件里
 *     extern 出来才能引用。少了这一行 main.c 会直接报
 *     "'ESP01S_OreCount' undeclared"。
 * ------------------------------------------------------------------------ */

/** 串口溢出(ORE)次数：CPU 读得太慢丢了字节。
    ★ 正常必须【恒为 0】；在涨说明接收侧跟不上（坏在链路上，不是协议）。 */
extern volatile uint32_t ESP01S_OreCount;

/** 旧链路里 SLIP 解出的帧超过 ESP01S_FRAME_MAX 而被丢掉的次数。
    新链路用 Vofa 分行，这个计数不会动。 */
extern volatile uint32_t ESP01S_OverCount;

#endif /* ESP01S_H */