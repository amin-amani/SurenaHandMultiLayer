cmake_minimum_required(VERSION 3.5)

project(pid_test LANGUAGES C)

include_directories(${CMAKE_SOURCE_DIR}/src)
file(GLOB SRC_COMMON  ${CMAKE_SOURCE_DIR}/Core/Src/*.c )
list(REMOVE_ITEM SRC_COMMON
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f4xx.c
    ${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32f1xx_it.c
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f4xx.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32f1xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/Core/Inc/stm32f4xx_hal_conf.h
    ${CMAKE_SOURCE_DIR}/Core/Inc/stm32f1xx_it.h
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32f1xx.c
    ${CMAKE_SOURCE_DIR}/Core/Inc/main.h

    )
add_compile_definitions(UNIT_TESTING)
# list(APPEND SRC_COMMON
#     ${CMAKE_SOURCE_DIR}/Core/Inc/bsp.h

#     )
message("==================${CMAKE_SOURCE_DIR}/Core/Inc")
file(GLOB SRC_TEST ${CMAKE_SOURCE_DIR}/test/pid/*.c )
include_directories(
${CMAKE_SOURCE_DIR}/lib/Unity/src/
${CMAKE_SOURCE_DIR}/Core/Inc
)

add_executable(${PROJECT_NAME}  ${SRC_TEST}  ${SRC_COMMON})
target_link_libraries(pid_test -L${CMAKE_BINARY_DIR}/lib/Unity -lunity gcov)
target_link_libraries(${PROJECT_NAME} -L${CMAKE_BINARY_DIR}/lib/Unity -lunity gcov)
# Include the Unity header files

# Enable testing
enable_testing()

# Enable code coverage flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0")

# Add coverage flags specifically for the test executable
target_compile_options(${PROJECT_NAME} PRIVATE --coverage)
message("==============================>>>>>>>>${CMAKE_BINARY_DIR}")
add_custom_target(coverage_report_${PROJECT_NAME} 
    COMMAND lcov --capture --directory ${CMAKE_BINARY_DIR} --output-file coverage.info
    COMMAND lcov --remove coverage.info '${CMAKE_SOURCE_DIR}/test/pid/*' '${CMAKE_SOURCE_DIR}/lib/Unity/*' --output-file coverage_filtered.info
    COMMAND genhtml coverage_filtered.info --output-directory coverage_report
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating code coverage report..."
    COMMAND DISPLAY=:99 cutycapt --url=file://${CMAKE_BINARY_DIR}/coverage_report/Src/index.html --out=${CMAKE_BINARY_DIR}/coverage-screenshot.png
)
add_dependencies(coverage_report ${PROJECT_NAME})
