#ifndef JSON_PAYLOADS_H
#define JSON_PAYLOADS_H

#include "cJSON.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

extern volatile bool    g_presence;
extern volatile bool    g_fall_alarm;
extern volatile bool    g_stay_still_alarm;
extern volatile uint8_t g_movement_state;
extern volatile uint8_t g_body_movement_param;
extern volatile uint8_t g_heartbeat;
extern volatile int16_t g_traj_x;
extern volatile int16_t g_traj_y;
extern volatile uint16_t g_total_height_count;
extern volatile uint8_t  g_height_prop_0_0_5;
extern volatile uint8_t  g_height_prop_0_5_1;
extern volatile uint8_t  g_height_prop_1_1_5;
extern volatile uint8_t  g_height_prop_1_5_2;
extern volatile uint32_t g_non_presence_time;
extern volatile uint8_t g_working_status;
extern volatile uint8_t g_scenario;
extern volatile int16_t g_installation_angle_x;
extern volatile int16_t g_installation_angle_y;
extern volatile int16_t g_installation_angle_z;
extern volatile uint16_t g_installation_height;
extern volatile uint8_t g_fall_detection_sensitivity;
extern volatile uint32_t g_fall_duration;
extern volatile uint16_t g_fall_breaking_height;
extern volatile uint16_t g_sitting_still_distance;
extern volatile uint16_t g_moving_distance;
extern volatile uint32_t g_operating_time;
extern char g_product_model[];
extern char g_product_id[];
extern char g_hardware_model[];
extern char g_firmware_version[];

const char* movement_state_str(uint8_t state);
void print_live_json_payload(void);
void print_settings_json_payload(void);
void print_product_info_payload(void);
void print_usage_json_payload(void);

#endif // JSON_PAYLOADS_H 