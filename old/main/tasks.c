#include "tasks.h"
#include "uart_comm.h"
#include "json_payloads.h"

void uart_read_task(void *arg) {
    uint8_t data[BUF_SIZE];
    while(1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if(len > 0) {
            int i = 0;
            while(i < len) {
                if(data[i] == FRAME_HEADER0 && (i + 1) < len && data[i+1] == FRAME_HEADER1) {
                    if(i + 6 > len) {
                        break;
                    }
                    uint8_t control = data[i+2];
                    uint8_t command = data[i+3];
                    uint16_t payload_len = ((uint16_t)data[i+4] << 8) | data[i+5];
                    int frame_total_length = payload_len + 9;
                    if(i + frame_total_length > len) {
                        break;
                    }
                    if(data[i + frame_total_length - 2] != FRAME_TAIL0 ||
                       data[i + frame_total_length - 1] != FRAME_TAIL1) {
                        i++;
                        continue;
                    }
                    uint32_t sum = 0;
                    for (int j = 0; j < 6 + payload_len; j++) {
                        sum += data[i + j];
                    }
                    uint8_t calc_check = sum & 0xFF;
                    uint8_t recv_check = data[i + 6 + payload_len];
                    if(calc_check != recv_check) {
                        printf("Checksum mismatch: calc 0x%02X, recv 0x%02X\n", calc_check, recv_check);
                        i += frame_total_length;
                        continue;
                    }
                    
                    // Process the frame based on control and command values
                    // (The frame processing code remains unchanged)
                    
                    i += frame_total_length;
                } else {
                    i++;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void live_json_print_task(void *arg) {
    while(1) {
        print_live_json_payload();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void settings_json_print_task(void *arg) {
    while(1) {
        print_settings_json_payload();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void product_info_json_print_task(void *arg) {
    while(1) {
        print_product_info_payload();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void usage_json_print_task(void *arg) {
    while(1) {
        print_usage_json_payload();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void key_read_task(void *arg) {
    int ch;
    while(1) {
        ch = getchar();
        if(ch == 'r' || ch == 'R'){
            printf("Working Status Report (on demand): 0x%02X\n", g_working_status);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void settings_read_task(void *arg) {
    while(1) {
        send_query(0x05, 0x81, 0x0F);
        printf("Sent Working Status Query\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void product_info_query_task(void *arg) {
    while(1) {
        send_query(0x02, 0xA1, 0x0F);
        printf("Sent Product Model Query\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
        send_query(0x02, 0xA2, 0x0F);
        printf("Sent Product ID Query\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
        send_query(0x02, 0xA3, 0x0F);
        printf("Sent Hardware Model Query\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
        send_query(0x02, 0xA4, 0x0F);
        printf("Sent Firmware Version Query\n");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
} 