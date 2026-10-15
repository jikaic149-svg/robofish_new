idf_component_register(SRCS "robot_link.c" INCLUDE_DIRS "inc"
    REQUIRES BSP driver esp_timer freertos)
target_include_directories(${COMPONENT_LIB} PUBLIC "${PROJECT_DIR}/../../../common")
