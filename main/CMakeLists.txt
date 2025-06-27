set(DEVICE_ID "falldetector" CACHE STRING "Device ID for this build")
set(CERT_FILE "dev.crt" CACHE STRING "Path to the certificate file")

idf_component_register(
    SRCS "main.c" "ota_update.c"         # เพิ่ม ota_update.c
    INCLUDE_DIRS "."      # Include directories
    REQUIRES wifiManager   # เพิ่ม dependencies ของ wifiManager
    PRIV_REQUIRES json mqtt esp_https_ota     # เพิ่ม esp_https_ota เพื่อให้ include esp_https_ota.h ได้
    # Add any other parameters as needed
)

target_compile_definitions(${COMPONENT_LIB} PRIVATE DEVICE_ID=\"${DEVICE_ID}\")

if(NOT DEFINED CERT_FILE)
    set(CERT_FILE "dev.crt")
endif()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${CERT_FILE}")
    message(FATAL_ERROR "Certificate file not found: ${CMAKE_CURRENT_SOURCE_DIR}/${CERT_FILE}")
endif()

target_add_binary_data(${COMPONENT_TARGET} "${CERT_FILE}" TEXT) 