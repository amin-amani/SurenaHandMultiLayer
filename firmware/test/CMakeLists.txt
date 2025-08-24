cmake_minimum_required(VERSION 3.5)

project(RunAllTests LANGUAGES C)


file(GLOB SRC_COMMON ${CMAKE_SOURCE_DIR}/Core/Src/*.c )
 list(REMOVE_ITEM SRC_COMMON
     ${CMAKE_SOURCE_DIR}/Core/Src/main.c
     ${CMAKE_SOURCE_DIR}/Core/Src/uart.c
     ${CMAKE_SOURCE_DIR}/Core/Src/stm32f1xx_hal_msp.c
     ${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
     ${CMAKE_SOURCE_DIR}/Core/Src/eeprom.c
     ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f1xx.c
     ${CMAKE_SOURCE_DIR}/Core/Src/switch.c
     ${CMAKE_SOURCE_DIR}/Core/Src/adc.c
     ${CMAKE_SOURCE_DIR}/Core/Src/dac.c
     ${CMAKE_SOURCE_DIR}/Core/Src/stm32f1xx_it.c
     ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
     ${CMAKE_SOURCE_DIR}/Core/Src/lib_crc.c
     ${CMAKE_SOURCE_DIR}/Core/Src/memory.c
     )

add_compile_definitions(UNIT_TESTING)
# list(APPEND SRC_COMMON
#     ${CMAKE_SOURCE_DIR}/Core/Inc/bsp.h

#     )

file(GLOB SRC_TEST ${CMAKE_SOURCE_DIR}/test/*.c )
include_directories(
${CMAKE_SOURCE_DIR}/lib/Unity/src/
${CMAKE_SOURCE_DIR}/lib/Unity/extras/fixture/src
${CMAKE_SOURCE_DIR}/lib/Unity/extras/memory/src
${CMAKE_SOURCE_DIR}/lib/Unity/src/
${CMAKE_BINARY_DIR}/includes
)


add_executable(${PROJECT_NAME} ${SRC_TEST}  ${SRC_COMMON} )

target_link_libraries(${PROJECT_NAME} -L${CMAKE_BINARY_DIR}/lib/Unity -lunity gcov)
# Include the Unity header files

# Enable testing
enable_testing()

# Enable code coverage flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0")

# Add coverage flags specifically for the test executable
target_compile_options(${PROJECT_NAME} PRIVATE --coverage)
message("======================!!!!========>>>>>>>>${CMAKE_SOURCE_DIR}")
add_custom_target(coverage_report
    COMMAND lcov --ignore-errors mismatch --capture --directory ${CMAKE_BINARY_DIR} --output-file coverage.info
    COMMAND lcov --ignore-errors mismatch --ignore-errors unused --remove coverage.info '${CMAKE_SOURCE_DIR}/test/*' '${CMAKE_SOURCE_DIR}/lib/Unity/*' --output-file coverage_filtered.info
    COMMAND genhtml coverage_filtered.info --output-directory coverage_report
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
add_dependencies(coverage_report ${PROJECT_NAME})
