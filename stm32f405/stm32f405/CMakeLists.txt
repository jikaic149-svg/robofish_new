set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Immutable upstream revisions; downloads stay inside the ignored build tree.
set(FETCHCONTENT_BASE_DIR "${CMAKE_BINARY_DIR}/../deps"
    CACHE PATH "Shared vendor source cache")
include(FetchContent)
FetchContent_Declare(stm32_hal
    URL https://codeload.github.com/STMicroelectronics/stm32f4xx-hal-driver/zip/b6f0ed3829f3829eb358a2e7417d80bba1a42db7
    URL_HASH SHA256=db48e624fd80cc87d1ac4c2797c3e8dbba87710297c39e99e9b6db6005d4d181
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE SOURCE_SUBDIR sources_only)
FetchContent_Declare(stm32_device
    URL https://codeload.github.com/STMicroelectronics/cmsis-device-f4/zip/0fa0e489e053fa1ca7790bb40b4d76458f64c55d
    URL_HASH SHA256=c52b7e008d10c28d3a5edb664f88b466b07a6ac8a7a0b741504933a7a0bfb571
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE SOURCE_SUBDIR sources_only)
FetchContent_Declare(cmsis
    URL https://codeload.github.com/ARM-software/CMSIS_5/zip/2b7495b8535bdcb306dac29b9ded4cfb679d7e5c
    URL_HASH SHA256=9ed0b6ebe8be4470bc5b20190df4a8ea863cdb9651b44689947d1c7783fdf8e4
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE SOURCE_SUBDIR sources_only)
FetchContent_Declare(freertos
    URL https://codeload.github.com/FreeRTOS/FreeRTOS-Kernel/zip/0adc196d4bd52a2d91102b525b0aafc1e14a2386
    URL_HASH SHA256=3150cc63b717c8a4c3abb004cb4c8d21ed2562d475fa49d4da392e3d591de36d
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE SOURCE_SUBDIR sources_only)
FetchContent_MakeAvailable(stm32_hal stm32_device cmsis freertos)

# Fixed layout constants are consumed from the same header as Boot/host tools.
set(boot_common "${PROJECT_SOURCE_DIR}/../common/stm32_boot.h")
file(STRINGS "${boot_common}" layout_lines REGEX "^#define RF_(APP_A|APP_B|APP_MAX_BYTES|BOOT_BASE|BOOT_END) ")
foreach(line IN LISTS layout_lines)
  string(REGEX MATCH "#define (RF_[A-Z_]+) UINT32_C\\((0x[0-9A-Fa-f]+)\\)" parsed "${line}")
  if(NOT parsed)
    message(FATAL_ERROR "Cannot parse fixed boot layout: ${line}")
  endif()
  set(${CMAKE_MATCH_1} "${CMAKE_MATCH_2}")
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${boot_common}")
set(generated "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${generated}")
file(READ "${stm32_device_SOURCE_DIR}/Source/Templates/gcc/startup_stm32f405xx.s" startup)
string(REPLACE "  bl  SystemInit" "  bl  robofish_early_watchdog_start\n  bl  SystemInit" application_startup "${startup}")
if(application_startup STREQUAL startup)
  message(FATAL_ERROR "Early watchdog reset hook did not match vendor startup")
endif()
file(WRITE "${generated}/startup_application.s" "${application_startup}")
set(RF_LINK_BASE "${RF_APP_A}")
set(RF_LINK_LENGTH "${RF_APP_MAX_BYTES}")
configure_file(STM32F405RGTx_FLASH.ld "${generated}/app_a.ld" @ONLY)
set(RF_LINK_BASE "${RF_APP_B}")
configure_file(STM32F405RGTx_FLASH.ld "${generated}/app_b.ld" @ONLY)
set(RF_LINK_BASE "${RF_BOOT_BASE}")
math(EXPR RF_LINK_LENGTH "${RF_BOOT_END} - ${RF_BOOT_BASE}" OUTPUT_FORMAT HEXADECIMAL)
configure_file(STM32F405RGTx_FLASH.ld "${generated}/boot.ld" @ONLY)
set(shared "${CMAKE_CURRENT_SOURCE_DIR}/../src")
set(hal "${stm32_hal_SOURCE_DIR}/Src")
add_executable(RoboFishF405
    Core/Src/main.cpp bsp/BoardInit.cpp bsp/RuntimeSafety.cpp boot/BootStore.c
    bsp/BatteryAdc.cpp bsp/SpiLink.cpp bsp/ServoPwm.cpp bsp/LoRaUart.cpp bsp/RgbStatusStrip.cpp
    ${shared}/algorithm/PidController.cpp
    ${shared}/control/l0/IntentAdapter.cpp ${shared}/control/l0/VisionMotionPolicy.cpp
    ${shared}/control/l2/Allocator.cpp ${shared}/control/l2/tail/Tail.cpp
    ${shared}/control/l2/propulsion/Propulsion.cpp ${shared}/control/l2/pectoral/Pectoral.cpp
    ${shared}/device/motion/Propulsion.cpp
    ${shared}/device/motion/RoboFishDriver.cpp ${shared}/device/motion/PectoralFins.cpp
    ${shared}/device/motion/TailServos.cpp
    ${shared}/task/RobotTasks.cpp ${shared}/task/MotionController.cpp ${shared}/task/MotionVision.cpp ${shared}/task/MotionFeedback.cpp ${shared}/task/MotionCommandInput.cpp
    ${shared}/service/lora/LoraProtocol.cpp ${shared}/service/lora/LoRaService.cpp
    ${hal}/stm32f4xx_hal.c ${hal}/stm32f4xx_hal_cortex.c
    ${hal}/stm32f4xx_hal_rcc.c
    ${hal}/stm32f4xx_hal_gpio.c ${hal}/stm32f4xx_hal_dma.c
    ${hal}/stm32f4xx_hal_pwr.c
    ${hal}/stm32f4xx_hal_tim.c
    ${hal}/stm32f4xx_hal_uart.c ${hal}/stm32f4xx_hal_spi.c
    ${hal}/stm32f4xx_hal_adc.c ${hal}/stm32f4xx_hal_adc_ex.c
    ${stm32_device_SOURCE_DIR}/Source/Templates/system_stm32f4xx.c
    ${generated}/startup_application.s
    ${freertos_SOURCE_DIR}/tasks.c ${freertos_SOURCE_DIR}/queue.c
    ${freertos_SOURCE_DIR}/list.c
    ${freertos_SOURCE_DIR}/portable/GCC/ARM_CM4F/port.c)
set_target_properties(RoboFishF405 PROPERTIES
    SUFFIX .elf RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
target_include_directories(RoboFishF405 PRIVATE
    Core/Inc . ${shared} "${PROJECT_SOURCE_DIR}/../common"
    ${stm32_hal_SOURCE_DIR}/Inc ${stm32_device_SOURCE_DIR}/Include
    ${cmsis_SOURCE_DIR}/CMSIS/Core/Include
    ${freertos_SOURCE_DIR}/include ${freertos_SOURCE_DIR}/portable/GCC/ARM_CM4F)
target_compile_definitions(RoboFishF405 PRIVATE
    STM32F405xx USE_HAL_DRIVER HSE_VALUE=8000000U USER_VECT_TAB_ADDRESS VECT_TAB_OFFSET=0x10000U
    RF_HAS_NETWORK_LINK=0)
set(cpu_flags -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_options(RoboFishF405 PRIVATE ${cpu_flags}
    -ffunction-sections -fdata-sections -Wall -Wextra
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics;-fno-use-cxa-atexit>)
set(linker_script "${generated}/app_a.ld")
target_link_options(RoboFishF405 PRIVATE ${cpu_flags}
    --specs=nano.specs --specs=nosys.specs "-T${linker_script}"
    -Wl,--gc-sections -Wl,--print-memory-usage
    "-Wl,-Map=${CMAKE_BINARY_DIR}/RoboFishF405.map")
set_property(TARGET RoboFishF405 APPEND PROPERTY LINK_DEPENDS "${linker_script}")
target_link_libraries(RoboFishF405 PRIVATE m)
add_custom_command(TARGET RoboFishF405 POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O binary $<TARGET_FILE:RoboFishF405>
            ${CMAKE_BINARY_DIR}/RoboFishF405.bin
    COMMAND "${CMAKE_OBJCOPY}" -O ihex $<TARGET_FILE:RoboFishF405>
            ${CMAKE_BINARY_DIR}/RoboFishF405.hex
    COMMAND "${CMAKE_SIZE}" $<TARGET_FILE:RoboFishF405>
    BYPRODUCTS "${CMAKE_BINARY_DIR}/RoboFishF405.bin" "${CMAKE_BINARY_DIR}/RoboFishF405.hex"
    VERBATIM)

# Same application, separately linked for the two fixed slots.
get_target_property(application_sources RoboFishF405 SOURCES)
add_executable(RoboFishF405_B ${application_sources})
set_target_properties(RoboFishF405_B PROPERTIES SUFFIX .elf RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
foreach(property INCLUDE_DIRECTORIES COMPILE_OPTIONS)
  get_target_property(value RoboFishF405 ${property})
  set_property(TARGET RoboFishF405_B PROPERTY ${property} "${value}")
endforeach()
get_target_property(definitions RoboFishF405 COMPILE_DEFINITIONS)
list(FILTER definitions EXCLUDE REGEX "^VECT_TAB_OFFSET=")
target_compile_definitions(RoboFishF405_B PRIVATE ${definitions} VECT_TAB_OFFSET=0x80000U)
target_link_options(RoboFishF405_B PRIVATE ${cpu_flags} --specs=nano.specs --specs=nosys.specs
  "-T${generated}/app_b.ld" -Wl,--gc-sections -Wl,--print-memory-usage
  "-Wl,-Map=${CMAKE_BINARY_DIR}/RoboFishF405_B.map")
target_link_libraries(RoboFishF405_B PRIVATE m)
set_property(TARGET RoboFishF405_B APPEND PROPERTY LINK_DEPENDS "${generated}/app_b.ld")

add_executable(RoboFishBoot boot/main.c boot/BootStore.c
  ${stm32_device_SOURCE_DIR}/Source/Templates/gcc/startup_stm32f405xx.s)
set_target_properties(RoboFishBoot PROPERTIES SUFFIX .elf RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
target_include_directories(RoboFishBoot PRIVATE . "${PROJECT_SOURCE_DIR}/../common"
  ${stm32_device_SOURCE_DIR}/Include ${cmsis_SOURCE_DIR}/CMSIS/Core/Include)
target_compile_definitions(RoboFishBoot PRIVATE STM32F405xx)
target_compile_options(RoboFishBoot PRIVATE ${cpu_flags} -Os -ffunction-sections -fdata-sections -Wall -Wextra)
target_link_options(RoboFishBoot PRIVATE ${cpu_flags} --specs=nano.specs --specs=nosys.specs
  "-T${generated}/boot.ld" -Wl,--gc-sections -Wl,--print-memory-usage
  "-Wl,-Map=${CMAKE_BINARY_DIR}/RoboFishBoot.map")
set_property(TARGET RoboFishBoot APPEND PROPERTY LINK_DEPENDS "${generated}/boot.ld")
foreach(image RoboFishF405_B RoboFishBoot)
  add_custom_command(TARGET ${image} POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O binary $<TARGET_FILE:${image}> "${CMAKE_BINARY_DIR}/${image}.bin"
    COMMAND "${CMAKE_OBJCOPY}" -O ihex $<TARGET_FILE:${image}> "${CMAKE_BINARY_DIR}/${image}.hex"
    BYPRODUCTS "${CMAKE_BINARY_DIR}/${image}.bin" "${CMAKE_BINARY_DIR}/${image}.hex" VERBATIM)
endforeach()

find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/RoboFishFactory.bin"
  "${CMAKE_BINARY_DIR}/RoboFishFactory.elf" "${CMAKE_BINARY_DIR}/stm32_ab_manifest.json"
  COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/../tools/package_stm32_ab.py"
          --build-dir "${CMAKE_BINARY_DIR}"
  COMMAND "${CMAKE_OBJCOPY}" -I binary -O elf32-littlearm -B arm
          --rename-section .data=.factory,alloc,load,readonly,code
          --change-addresses=${RF_BOOT_BASE}
          "${CMAKE_BINARY_DIR}/RoboFishFactory.bin" "${CMAKE_BINARY_DIR}/RoboFishFactory.elf"
  DEPENDS RoboFishBoot RoboFishF405 RoboFishF405_B
          "${PROJECT_SOURCE_DIR}/../tools/package_stm32_ab.py" "${boot_common}" VERBATIM)
add_custom_target(factory-image ALL DEPENDS "${CMAKE_BINARY_DIR}/RoboFishFactory.bin"
  "${CMAKE_BINARY_DIR}/RoboFishFactory.elf" "${CMAKE_BINARY_DIR}/stm32_ab_manifest.json")

find_program(OPENOCD_EXECUTABLE openocd HINTS "$ENV{OPENOCD_BIN}")
if(OPENOCD_EXECUTABLE)
    add_custom_target(flash
        COMMAND "${OPENOCD_EXECUTABLE}" -f "${CMAKE_CURRENT_SOURCE_DIR}/openocd.cfg"
                -c "program {${CMAKE_BINARY_DIR}/RoboFishFactory.bin} ${RF_BOOT_BASE} verify reset exit"
        DEPENDS factory-image USES_TERMINAL VERBATIM)
    add_custom_target(debug-server
        COMMAND "${OPENOCD_EXECUTABLE}" -f "${CMAKE_CURRENT_SOURCE_DIR}/openocd.cfg"
        DEPENDS RoboFishF405 USES_TERMINAL VERBATIM)
else()
    message(STATUS "OpenOCD not found: set OPENOCD_BIN or OPENOCD_EXECUTABLE to enable flash/debug-server")
endif()
if(ARM_GDB_EXECUTABLE)
    add_custom_target(debug
        COMMAND "${ARM_GDB_EXECUTABLE}" -x "${CMAKE_CURRENT_SOURCE_DIR}/debug.gdb"
                $<TARGET_FILE:RoboFishF405>
        DEPENDS RoboFishF405 USES_TERMINAL VERBATIM)
endif()
if(OPENOCD_EXECUTABLE AND ARM_GDB_EXECUTABLE)
    # Machine-specific executable paths belong in the ignored generated file.
    get_filename_component(arm_toolchain_dir "${CMAKE_C_COMPILER}" DIRECTORY)
    set(F405_WORKSPACE_PREFIX "")
    set(F405_BUILD_TASK "Build Debug")
    configure_file("${PROJECT_SOURCE_DIR}/.vscode/launch.json.in"
                   "${PROJECT_SOURCE_DIR}/.vscode/launch.json" @ONLY)
    set(F405_WORKSPACE_PREFIX "stm32f405/")
    set(F405_BUILD_TASK "Build STM32 Debug")
    configure_file("${PROJECT_SOURCE_DIR}/.vscode/launch.json.in"
                   "${PROJECT_SOURCE_DIR}/../.vscode/launch.json" @ONLY)
endif()
if(BUILD_TESTING)
    find_package(Python3 COMPONENTS Interpreter)
    if(Python3_Interpreter_FOUND)
        add_test(NAME firmware-validation
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/verify_stm32_target.py"
                    --build-dir "${CMAKE_BINARY_DIR}" --nm "${CMAKE_NM}")
        add_test(NAME firmware-validation-b
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/verify_stm32_target.py"
                    --build-dir "${CMAKE_BINARY_DIR}" --nm "${CMAKE_NM}"
                    --image RoboFishF405_B --base 0x08080000)
        add_test(NAME bootloader-validation
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/verify_stm32_target.py"
                    --build-dir "${CMAKE_BINARY_DIR}" --nm "${CMAKE_NM}"
                    --image RoboFishBoot --base 0x08000000 --boot)
    endif()
endif()
