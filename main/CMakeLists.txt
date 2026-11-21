idf_component_register(SRCS "main.c" INCLUDE_DIRS "."
                       REQUIRES vl53l1x driver esp_driver_gpio esp_driver_ledc esp_timer esp_driver_uart esp_driver_usb_serial_jtag esp_hw_support)
if(NOT DEFINED LIDAR_INT_GPIO)
    set(LIDAR_INT_GPIO -1)
endif()
if(NOT DEFINED LIDAR_USB_FRAMES)
    set(LIDAR_USB_FRAMES 0)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE LIDAR_INT_GPIO=${LIDAR_INT_GPIO} LIDAR_USB_FRAMES=${LIDAR_USB_FRAMES})
