#ifndef TASKS_H
#define TASKS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void uart_read_task(void *arg);
void live_json_print_task(void *arg);
void settings_json_print_task(void *arg);
void product_info_json_print_task(void *arg);
void usage_json_print_task(void *arg);
void key_read_task(void *arg);
void settings_read_task(void *arg);
void product_info_query_task(void *arg);

#endif // TASKS_H 