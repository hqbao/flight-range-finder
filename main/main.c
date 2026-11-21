#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l1_api.h"
#include "vl53l1_api_core.h"

#define PIN_SDA 5
#define PIN_SCL 6
#define PIN_FRAME_TX 43
#define PIN_FRAME_RX 44
#define FRAME_UART UART_NUM_0
#define FRAME_BAUD 115200
#define I2C_ADDR_7BIT 0x29
#define I2C_ADDR_8BIT 0x52
#define I2C_SPEED_HZ 400000
#define TIMING_BUDGET_US 20000
#define INTER_MEASUREMENT_MS 25
#define STATUS_ERROR 0xFF
#define CMD_LASER_RANGE 0x17
#define PAYLOAD_LEN 4
#define FRAME_LEN (6 + PAYLOAD_LEN + 2)
#define PIN_LED 21
#define LED_BITS 13
#define LED_MAX ((1u << LED_BITS) - 1)
#define LED_MIN 80u
#define LED_FULL_MM 4000u
#define LED_FLASH_US 10000
#define LED_ERR_HALF_US 500000
#define READY_TIMEOUT_US 100000
#define RESTART_AFTER_US 500000

static VL53L1_Dev_t dev;
static TaskHandle_t lidar_task;
static esp_timer_handle_t led_timer;

#if LIDAR_INT_GPIO >= 0
static void IRAM_ATTR on_ready(void *arg)
{
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(lidar_task, &woken);
    portYIELD_FROM_ISR(woken);
}
#endif

static void build_frame(uint8_t *f, uint8_t seq, uint8_t status, uint16_t range)
{
    f[0] = 'd';
    f[1] = 'b';
    f[2] = CMD_LASER_RANGE;
    f[3] = 0x00;
    f[4] = PAYLOAD_LEN;
    f[5] = 0x00;
    f[6] = seq;
    f[7] = status;
    f[8] = (uint8_t)range;
    f[9] = (uint8_t)(range >> 8);
    uint16_t sum = 0;
    for (int i = 2; i < 10; i++) {
        sum = (uint16_t)(sum + f[i]);
    }
    f[10] = (uint8_t)sum;
    f[11] = (uint8_t)(sum >> 8);
}

static void emit(const uint8_t *f)
{
    uart_write_bytes(FRAME_UART, f, FRAME_LEN);
#if LIDAR_USB_FRAMES
    usb_serial_jtag_write_bytes(f, FRAME_LEN, 0);
#endif
}

static void led_set(uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void led_off(void *arg)
{
    led_set(0);
}

static void show(uint8_t status, uint16_t range)
{
    static int64_t last_toggle;
    static bool err_on;
    esp_timer_stop(led_timer);
    if (status == STATUS_ERROR) {
        int64_t now = esp_timer_get_time();
        if (now - last_toggle >= LED_ERR_HALF_US) {
            last_toggle = now;
            err_on = !err_on;
            led_set(err_on ? LED_MAX : 0);
        }
        return;
    }
    uint32_t r = range < LED_FULL_MM ? range : LED_FULL_MM;
    led_set(LED_MIN + (uint32_t)((uint64_t)(LED_MAX - LED_MIN) * r * r / (LED_FULL_MM * LED_FULL_MM)));
    esp_timer_start_once(led_timer, LED_FLASH_US);
}

static VL53L1_Error sensor_start(void)
{
    VL53L1_UserRoi_t roi = {.TopLeftX = 0, .TopLeftY = 15, .BotRightX = 15, .BotRightY = 0};
    VL53L1_Error e = VL53L1_WaitDeviceBooted(&dev);
    if (e == VL53L1_ERROR_NONE) e = VL53L1_DataInit(&dev);
    if (e == VL53L1_ERROR_NONE) e = VL53L1_StaticInit(&dev);
    if (e == VL53L1_ERROR_NONE) e = VL53L1_SetDistanceMode(&dev, VL53L1_DISTANCEMODE_LONG);
    if (e == VL53L1_ERROR_NONE) e = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, TIMING_BUDGET_US);
    if (e == VL53L1_ERROR_NONE) e = VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev, INTER_MEASUREMENT_MS);
    if (e == VL53L1_ERROR_NONE) e = VL53L1_SetUserROI(&dev, &roi);
    if (e == VL53L1_ERROR_NONE) e = VL53L1_StartMeasurement(&dev);
    return e;
}

static void lidar_loop(void *arg)
{
    uint8_t seq = 0, frame[FRAME_LEN];
    int64_t last_frame = esp_timer_get_time(), last_good = last_frame;
    for (;;) {
#if LIDAR_INT_GPIO >= 0
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
#else
        vTaskDelay(1);
#endif
        uint8_t ready = 0;
        VL53L1_Error e = VL53L1_GetMeasurementDataReady(&dev, &ready);
        if (e == VL53L1_ERROR_NONE && !ready && esp_timer_get_time() - last_frame < READY_TIMEOUT_US) {
            continue;
        }
        uint8_t status = STATUS_ERROR;
        uint16_t range = 0;
        if (e == VL53L1_ERROR_NONE && ready) {
            VL53L1_RangingMeasurementData_t m;
            if (VL53L1_GetRangingMeasurementData(&dev, &m) == VL53L1_ERROR_NONE) {
                status = m.RangeStatus;
                range = (uint16_t)m.RangeMilliMeter;
            }
        }
        build_frame(frame, seq++, status, range);
        emit(frame);
        show(status, range);
        last_frame = esp_timer_get_time();
        if (status != STATUS_ERROR) {
            last_good = last_frame;
            VL53L1_ClearInterruptAndStartMeasurement(&dev);
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(INTER_MEASUREMENT_MS));
        if (esp_timer_get_time() - last_good >= RESTART_AFTER_US) {
            last_good = esp_timer_get_time();
            sensor_start();
        } else {
            VL53L1_ClearInterruptAndStartMeasurement(&dev);
        }
    }
}

void app_main(void)
{
    ledc_timer_config_t lt = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LED_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&lt));
    ledc_channel_config_t lc = {
        .gpio_num = PIN_LED,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = LED_MAX,
        .flags.output_invert = 1,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&lc));
    esp_timer_create_args_t ta = {.callback = led_off, .name = "led_off"};
    ESP_ERROR_CHECK(esp_timer_create(&ta, &led_timer));
    uart_config_t uc = {
        .baud_rate = FRAME_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(FRAME_UART, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(FRAME_UART, &uc));
    ESP_ERROR_CHECK(uart_set_pin(FRAME_UART, PIN_FRAME_TX, PIN_FRAME_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
#if LIDAR_USB_FRAMES
    usb_serial_jtag_driver_config_t uj = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&uj));
#endif

    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t bc = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &bus));
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR_7BIT,
        .scl_speed_hz = I2C_SPEED_HZ,
    };
    i2c_master_dev_handle_t dh;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dc, &dh));
    dev.I2cHandle = dh;
    dev.I2cDevAddr = I2C_ADDR_8BIT;

    VL53L1_Error started = sensor_start();

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    printf("xiao_lidar mac=%02x:%02x:%02x:%02x:%02x:%02x mode=LONG budget_us=%d imp_ms=%d roi=16x16 baud=%d int_gpio=%d usb_frames=%d sensor_start=%d\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], TIMING_BUDGET_US, INTER_MEASUREMENT_MS,
           FRAME_BAUD, LIDAR_INT_GPIO, LIDAR_USB_FRAMES, started);

    xTaskCreatePinnedToCore(lidar_loop, "lidar", 4096, NULL, configMAX_PRIORITIES - 2, &lidar_task, 1);

#if LIDAR_INT_GPIO >= 0
    VL53L1_DeviceInterruptPolarity pol = VL53L1_DEVICEINTERRUPTPOLARITY_ACTIVE_LOW;
    VL53L1_get_interrupt_polarity(&dev, &pol);
    gpio_config_t gc = {
        .pin_bit_mask = 1ULL << LIDAR_INT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .intr_type = pol == VL53L1_DEVICEINTERRUPTPOLARITY_ACTIVE_HIGH ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&gc));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(LIDAR_INT_GPIO, on_ready, NULL));
#endif
}
