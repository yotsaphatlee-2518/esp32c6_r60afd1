#include "json_payloads.h"

const char* movement_state_str(uint8_t state) {
    switch(state) {
        case 0: return "No movement";
        case 1: return "Static";
        case 2: return "Active";
        default: return "Unknown";
    }
}

void print_live_json_payload(void) {
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        printf("Error creating live JSON object\n");
        return;
    }

    cJSON_AddBoolToObject(json, "presence", g_presence);
    cJSON_AddBoolToObject(json, "fall_alarm", g_fall_alarm);
    cJSON_AddBoolToObject(json, "stay_still_alarm", g_stay_still_alarm);
    cJSON_AddNumberToObject(json, "movement_state", g_movement_state);
    cJSON_AddStringToObject(json, "movement_state_readable", movement_state_str(g_movement_state));
    cJSON_AddNumberToObject(json, "body_movement_param", g_body_movement_param);
    cJSON_AddNumberToObject(json, "heartbeat", g_heartbeat);
    cJSON_AddNumberToObject(json, "trajectory_x", g_traj_x);
    cJSON_AddNumberToObject(json, "trajectory_y", g_traj_y);
    cJSON_AddNumberToObject(json, "total_height_count", g_total_height_count);
    cJSON_AddNumberToObject(json, "height_prop_0_0_5", g_height_prop_0_0_5);
    cJSON_AddNumberToObject(json, "height_prop_0_5_1", g_height_prop_0_5_1);
    cJSON_AddNumberToObject(json, "height_prop_1_1_5", g_height_prop_1_1_5);
    cJSON_AddNumberToObject(json, "height_prop_1_5_2", g_height_prop_1_5_2);
    cJSON_AddNumberToObject(json, "non_presence_time", g_non_presence_time);

    char *json_str = cJSON_Print(json);
    if(json_str) {
        printf("Live JSON Payload: %s\n", json_str);
        free(json_str);
    }
    cJSON_Delete(json);
}

void print_settings_json_payload(void) {
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        printf("Error creating settings JSON object\n");
        return;
    }
    
    cJSON_AddNumberToObject(json, "working_status", g_working_status);
    cJSON_AddNumberToObject(json, "scenario", g_scenario);
    cJSON_AddNumberToObject(json, "installation_angle_x", g_installation_angle_x);
    cJSON_AddNumberToObject(json, "installation_angle_y", g_installation_angle_y);
    cJSON_AddNumberToObject(json, "installation_angle_z", g_installation_angle_z);
    cJSON_AddNumberToObject(json, "installation_height", g_installation_height);
    
    cJSON_AddNumberToObject(json, "fall_detection_sensitivity", g_fall_detection_sensitivity);
    cJSON_AddNumberToObject(json, "fall_duration", g_fall_duration);
    cJSON_AddNumberToObject(json, "fall_breaking_height", g_fall_breaking_height);
    
    cJSON_AddNumberToObject(json, "sitting_still_distance", g_sitting_still_distance);
    cJSON_AddNumberToObject(json, "moving_distance", g_moving_distance);
    
    char *json_str = cJSON_Print(json);
    if(json_str) {
        printf("******************************************************** \n");
        printf("Settings JSON Payload: %s\n", json_str);
        free(json_str);
        printf("********************************************************\n");
    }
    cJSON_Delete(json);
}

void print_product_info_payload(void) {
    cJSON *json = cJSON_CreateObject();
    if(json == NULL) {
        printf("Error creating product info JSON object\n");
        return;
    }
    cJSON_AddStringToObject(json, "product_model", g_product_model);
    cJSON_AddStringToObject(json, "product_id", g_product_id);
    cJSON_AddStringToObject(json, "hardware_model", g_hardware_model);
    cJSON_AddStringToObject(json, "firmware_version", g_firmware_version);

    char *json_str = cJSON_Print(json);
    if(json_str) {
        printf("========================================================\n");
        printf("Product Information JSON Payload: %s\n", json_str);
        printf("========================================================\n");
        free(json_str);
    }
    cJSON_Delete(json);
}

void print_usage_json_payload(void) {
    cJSON *json = cJSON_CreateObject();
    if(json == NULL) {
        printf("Error creating usage JSON object\n");
        return;
    }
    cJSON_AddNumberToObject(json, "operating_time", g_operating_time);
    char *json_str = cJSON_Print(json);
    if(json_str) {
        printf("Usage/Operating Time JSON Payload: %s\n", json_str);
        free(json_str);
    }
    cJSON_Delete(json);
} 