#include "uart_comm.h"

void init_uart(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void send_query(uint8_t control, uint8_t command, uint8_t data_payload) {
    uint8_t frame[10];
    frame[0] = FRAME_HEADER0;
    frame[1] = FRAME_HEADER1;
    frame[2] = control;
    frame[3] = command;
    frame[4] = 0x00;
    frame[5] = 0x01; // 1 byte payload
    frame[6] = data_payload;
    uint32_t sum = 0;
    for (int j = 0; j < 7; j++) {
        sum += frame[j];
    }
    frame[7] = sum & 0xFF;
    frame[8] = FRAME_TAIL0;
    frame[9] = FRAME_TAIL1;
    uart_write_bytes(UART_PORT_NUM, (const char*)frame, sizeof(frame));
} 