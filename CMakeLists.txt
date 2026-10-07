cmake_minimum_required(VERSION 3.21)
project(xingxuan_uav LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(cmake/sources.cmake)
add_executable(xingxuan_uav ${FIRMWARE_SOURCES})
set_target_properties(xingxuan_uav PROPERTIES SUFFIX ".elf")
set_property(TARGET xingxuan_uav APPEND PROPERTY LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/firmware/boards/stm32/firmware.ld)
target_include_directories(xingxuan_uav PRIVATE ${FIRMWARE_INCLUDES})
target_compile_definitions(xingxuan_uav PRIVATE USE_HAL_DRIVER STM32F407xx)
target_compile_options(xingxuan_uav PRIVATE -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  -ffunction-sections -fdata-sections -fstack-usage -Wall -Wextra -Werror=implicit-function-declaration -Werror=uninitialized -Werror=maybe-uninitialized -Werror=return-type
  $<$<CONFIG:Debug>:-Og> $<$<CONFIG:Debug>:-g3> $<$<CONFIG:Release>:-Os>)
target_link_options(xingxuan_uav PRIVATE -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  -T${CMAKE_CURRENT_SOURCE_DIR}/firmware/boards/stm32/firmware.ld
  --specs=nano.specs --specs=nosys.specs -Wl,-z,noexecstack -Wl,--gc-sections -Wl,--print-memory-usage
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/xingxuan_uav.map -u _printf_float)
target_link_libraries(xingxuan_uav PRIVATE m)
add_custom_command(TARGET xingxuan_uav POST_BUILD
 COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:xingxuan_uav> ${CMAKE_CURRENT_BINARY_DIR}/xingxuan_uav.hex
 COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:xingxuan_uav> ${CMAKE_CURRENT_BINARY_DIR}/xingxuan_uav.bin
 COMMAND ${CMAKE_SIZE} $<TARGET_FILE:xingxuan_uav>)
find_program(OPENOCD openocd)
if(OPENOCD)
 add_custom_target(flash COMMAND ${OPENOCD} -f ${CMAKE_CURRENT_SOURCE_DIR}/openocd/board.cfg
  -c "program $<TARGET_FILE:xingxuan_uav> verify reset exit" DEPENDS xingxuan_uav USES_TERMINAL)
endif()
