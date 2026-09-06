# Host unit tests for the OpenDiag firmware.
#
# Standalone project: it does not use ESP-IDF and is not reachable from the
# top level CMakeLists.txt. From the repository root:
#
#   cmake -S test/host -B test/host/build
#   cmake --build test/host/build
#   ctest --test-dir test/host/build --output-on-failure
#
# The firmware sources are compiled from main/ directly. stubs/ shadows the
# ESP-IDF headers on the include path, which is the only seam: nothing in
# main/ is conditionally compiled for the host.

cmake_minimum_required(VERSION 3.16)
project(opendiag_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

set(FW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../main)

option(COVERAGE "Instrument the firmware sources for coverage reporting" OFF)

add_compile_options(
    -g
    -O0
    -Wall
    -Wextra
    -Wno-unused-parameter
    -fno-omit-frame-pointer
)

# The stub directory has to come first so that "freertos/FreeRTOS.h" and
# friends resolve here rather than in a real IDF checkout.
include_directories(BEFORE
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/framework
    ${CMAKE_CURRENT_SOURCE_DIR}/fakes
    ${FW_DIR}
)

# ------------------------------------------------------------------ #
# Support libraries
# ------------------------------------------------------------------ #

add_library(td_framework STATIC
    framework/td_test.c
    stubs/idf_stub.c
)

add_library(td_fakes STATIC
    fakes/fake_ble_uart.c
    fakes/fake_board.c
    fakes/fake_bus.c
    fakes/fake_can_bus.c
    fakes/fake_gpio.c
    fakes/fake_led.c
    fakes/fake_port.c
)
target_link_libraries(td_fakes PUBLIC td_framework)

# ------------------------------------------------------------------ #
# Suites
#
# Each suite names the firmware sources it links. A suite that pulls a
# firmware .c into its own translation unit to reach that file's statics must
# NOT list it here as well, or the symbols land twice:
#
#   the ELM327 suites include elm327_at.c   -> they link comm_iface.c and vif.c
#   test_comm_iface includes comm_iface.c   -> it links neither
# ------------------------------------------------------------------ #

set(ELM327_FW
    ${FW_DIR}/comm_iface.c
    ${FW_DIR}/kline_codec.c
    ${FW_DIR}/bus.c
    ${FW_DIR}/timer.c
    ${FW_DIR}/utility.c
    ${FW_DIR}/vif.c
)

set(SUITES
    test_elm327_at
    test_elm327_can
    test_elm327_bytebus
    test_elm327_search
    test_comm_iface
    test_vif
    test_vif_ctrl
    test_slcan
    test_button
    test_can_driver
    test_kline_driver
    test_j1850_lifecycle
    test_j1850_pwm_codec
    test_j1850_vpw_codec
    test_kline_codec
)

set(test_elm327_at_FW      ${ELM327_FW})
set(test_elm327_can_FW     ${ELM327_FW})
set(test_elm327_bytebus_FW ${ELM327_FW})
set(test_elm327_search_FW  ${ELM327_FW})
set(test_comm_iface_FW     "")
set(test_button_FW         "")
set(test_ble_pair_policy_FW "")
set(test_vif_FW
    ${FW_DIR}/comm_iface.c
    ${FW_DIR}/kline_codec.c
    ${FW_DIR}/bus.c
    ${FW_DIR}/vif.c
)
set(test_vif_ctrl_FW
    ${FW_DIR}/comm_iface.c
    ${FW_DIR}/kline_codec.c
    ${FW_DIR}/bus.c
    ${FW_DIR}/vif.c
    ${FW_DIR}/vif_ctrl.c
)
set(test_can_driver_FW ${FW_DIR}/can_bus.c driver_stubs/driver_rtos.c)
set(test_kline_driver_FW ${FW_DIR}/kline_codec.c ${FW_DIR}/bus.c
    ${FW_DIR}/utility.c driver_stubs/driver_rtos.c)
set(test_j1850_lifecycle_FW ${FW_DIR}/j1850_lifecycle.c)
set(test_j1850_pwm_codec_FW ${FW_DIR}/j1850_pwm_codec.c
    ${FW_DIR}/j1850_common.c)
set(test_j1850_vpw_codec_FW ${FW_DIR}/j1850_vpw_codec.c
    ${FW_DIR}/j1850_common.c)
set(test_kline_codec_FW     ${FW_DIR}/kline_codec.c
    ${FW_DIR}/bus.c)
set(test_slcan_FW
    ${FW_DIR}/comm_iface.c
    ${FW_DIR}/kline_codec.c
    ${FW_DIR}/bus.c
    ${FW_DIR}/slcan.c
    ${FW_DIR}/utility.c
    ${FW_DIR}/vif.c
)

if(COVERAGE)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang")
        set(COV_FLAGS -fprofile-instr-generate -fcoverage-mapping)
    else()
        set(COV_FLAGS --coverage)
    endif()
endif()

foreach(suite ${SUITES})
    add_executable(${suite} suites/${suite}.c ${${suite}_FW})
    target_link_libraries(${suite} PRIVATE td_fakes td_framework)
    if(suite STREQUAL "test_can_driver" OR suite STREQUAL "test_kline_driver")
        target_include_directories(${suite} BEFORE PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/driver_stubs)
    endif()

    if(COVERAGE)
        # Only the suite objects are instrumented, and those are the ones
        # holding the firmware sources. Measuring the fakes and the framework
        # would just dilute the number.
        target_compile_options(${suite} PRIVATE ${COV_FLAGS})
        target_link_options(${suite} PRIVATE ${COV_FLAGS})
    endif()

    add_test(NAME ${suite} COMMAND ${suite})

    # Each binary writes its own profile, merged by coverage.sh.
    set_tests_properties(${suite} PROPERTIES
        ENVIRONMENT "LLVM_PROFILE_FILE=${CMAKE_BINARY_DIR}/${suite}.profraw")
endforeach()

# ------------------------------------------------------------------ #
# make coverage
# ------------------------------------------------------------------ #

add_custom_target(coverage
    COMMAND ${CMAKE_COMMAND} -E env
            COV_COMPILER=${CMAKE_C_COMPILER_ID}
            BUILD_DIR=${CMAKE_BINARY_DIR}
            SRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            FW_DIR=${FW_DIR}
            SUITE_LIST="${SUITES}"
            ${CMAKE_CURRENT_SOURCE_DIR}/coverage.sh
    DEPENDS ${SUITES}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running the suites and reporting firmware coverage"
    USES_TERMINAL
)
