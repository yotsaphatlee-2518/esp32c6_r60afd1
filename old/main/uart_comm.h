#ifndef UART_COMM_H
#define UART_COMM_H

#include "driver/uart.h"

#define UART_PORT_NUM      UART_NUM_1
#define BUF_SIZE           1024

#define FRAME_HEADER0      0x53
#define FRAME_HEADER1      0x59
#define FRAME_TAIL0        0x54
#define FRAME_TAIL1        0x43

void init_uart(void);
void send_query(uint8_t control, uint8_t command, uint8_t data_payload);

#endif // UART_COMM_H 