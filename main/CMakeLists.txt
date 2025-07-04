idf_component_register(SRCS main.c
                            utility.c
                            elm327_at.c
                            timer.c
                            ws2812_led.c
                            shell.c
                            board.c
                            button.c
                            slcan.c
                            usb_cdc.c
                            ble_uart.c
                            can_bus.c
                            kline.c
                            j1850_pwm.c
                            j1850_vpw.c
                            comm_iface.c
                            vif.c
                            vif_ctrl.c
                       PRIV_REQUIRES spi_flash bt nvs_flash esp_adc esp_ringbuf
                       REQUIRES driver
                       INCLUDE_DIRS "")
