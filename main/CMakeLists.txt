idf_component_register(SRCS main.c
                            utility.c
                            elm327_at.c
                            timer.c
                            ws2812_led.c
                            shell.c
                            board.c
                            button.c
                            slcan.c
                            j2534.c
                            j2534_scheduler.c
                            j2534_wire.c
                            j2534.pb.c
                            usb_cdc.c
                            ble_uart.c
                            can_bus.c
                            isotp.c
                            kline.c
                            kline_codec.c
                            bus_tx.c
                            bus_tx_worker.c
                            bus.c
                            j1850_pwm.c
                            j1850_pwm_codec.c
                            j1850_vpw.c
                            j1850_vpw_codec.c
                            j1850_common.c
                            j1850_lifecycle.c
                            comm_iface.c
                            vif.c
                            vif_ctrl.c
                            power.c
                            power_policy.c
                            wifi_link.c
                       PRIV_REQUIRES
                            nanopb
                            esp_hal_usb
                            spi_flash
                            bt
                            nvs_flash
                            esp_adc
                            esp_ringbuf
                            esp_app_format
                            esp_driver_ledc
                            esp_driver_gpio
                            esp_driver_mcpwm
                            esp_driver_uart
                            esp_driver_rmt
                            esp_driver_gptimer
                            esp_driver_twai
                            esp_driver_pcnt
                            esp_wifi
                            esp_netif
                            esp_event
                            lwip
                       REQUIRES driver freertos
                       INCLUDE_DIRS "")

# Board pin map. Each board has a directory under boards/ holding its own
# pinout.h. Select one with idf.py -DOPENDIAG_BOARD=<name>. The default is the
# OpenDIAG hardware v1 board, so an unchanged build is the same image.
set(OPENDIAG_BOARD "opendiag" CACHE STRING "Board pin map to build for")
set(OPENDIAG_BOARD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../boards/${OPENDIAG_BOARD}")
if(NOT EXISTS "${OPENDIAG_BOARD_DIR}/pinout.h")
    message(FATAL_ERROR "Unknown board '${OPENDIAG_BOARD}': no boards/${OPENDIAG_BOARD}/pinout.h")
endif()
target_include_directories(${COMPONENT_LIB} PRIVATE "${OPENDIAG_BOARD_DIR}")
