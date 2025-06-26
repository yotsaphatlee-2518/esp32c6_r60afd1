set(DEVICE_ID "falldetector" CACHE STRING "Device ID for this build")
set(CERT_FILE "crt/server1.crt" CACHE STRING "Path to the certificate file")

idf_component_register(
    SRCS "main.c" "ota_update.c"         # เพิ่ม ota_update.c
    INCLUDE_DIRS "."      # Include directories
    REQUIRES wifiManager   # เพิ่ม dependencies ของ wifiManager
    PRIV_REQUIRES json mqtt esp_https_ota     # เพิ่ม esp_https_ota เพื่อให้ include esp_https_ota.h ได้
    # Add any other parameters as needed
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID=\"${DEVICE_ID}\")
target_add_binary_data(${COMPONENT_TARGET} "${CERT_FILE}" TEXT) 