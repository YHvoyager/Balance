// printf.c
#include "printf.h"
#include "usart.h"
#include <string.h>

static uint8_t tx_buf[128];
static volatile uint8_t tx_busy = 0;

int _write(int file, char *ptr, int len)
{
    (void)file;

    if (tx_busy) {
        return len; // 忙时丢弃
    }

    if (len > (int)sizeof(tx_buf)) {
        len = sizeof(tx_buf);
    }

    memcpy(tx_buf, ptr, (size_t)len);
    tx_busy = 1;

    if (HAL_UART_Transmit_DMA(&huart1, tx_buf, (uint16_t)len) != HAL_OK) {
        tx_busy = 0;
        return -1;
    }

    return len;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        tx_busy = 0;
    }
}