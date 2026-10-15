idf_component_register(SRCS "CameraService.cpp" "robot_camera.cpp"
    INCLUDE_DIRS "." "inc" REQUIRES espressif__esp32-camera esp_http_server driver esp_timer)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
target_compile_definitions(${COMPONENT_LIB} PRIVATE RF_HAS_ESP32_CAMERA=1)
