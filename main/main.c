#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "dac8871_idf6.h"
#include "dacx0004_idf6.h"

static const char *TAG = "shadowgraph";

// ---------------------------------------------------------------------------
// Galvo SPI bus  (SPI2 / HSPI)
// ---------------------------------------------------------------------------
#define GALVO_SPI_HOST   SPI2_HOST
#define GALVO_PIN_MOSI   13
#define GALVO_PIN_CLK    14
#define GALVO_CLK_HZ     40000000

// Galvo X  (DAC8871, U$2)
#define GX_PIN_CS        21
#define GX_PIN_LDAC      19
#define GX_PIN_RST       22

// Galvo Y  (DAC8871, U$1)
#define GY_PIN_CS        16
#define GY_PIN_LDAC      15
#define GY_PIN_RST       17

// ---------------------------------------------------------------------------
// Laser SPI bus  (SPI3 / VSPI)
// ---------------------------------------------------------------------------
#define LASER_SPI_HOST   SPI3_HOST
#define LASER_PIN_MOSI   23
#define LASER_PIN_CLK    18
#define LASER_CLK_HZ     40000000

// Laser color  (DAC80004, U$3)
#define LASER_PIN_SYNC   26
#define LASER_PIN_LDAC   27
#define LASER_PIN_CLR    25

#define LASER_CH_RED     DACX0004_ADD_A
#define LASER_CH_GREEN   DACX0004_ADD_B
#define LASER_CH_BLUE    DACX0004_ADD_C

// ---------------------------------------------------------------------------
// Device handles
// ---------------------------------------------------------------------------
static dac8871_dev_t  galvo_x = {0};
static dac8871_dev_t  galvo_y = {0};
static dacx0004_dev_t laser   = {0};

static dac8871_idf6_arg_t galvo_x_arg = {
    .host       = GALVO_SPI_HOST,
    .clk_freq   = GALVO_CLK_HZ,
    .spi_q_size = 1,
    .cs_pin     = GX_PIN_CS,
    .ldac_pin   = GX_PIN_LDAC,
    .rst_pin    = GX_PIN_RST,
};

static dac8871_idf6_arg_t galvo_y_arg = {
    .host       = GALVO_SPI_HOST,
    .clk_freq   = GALVO_CLK_HZ,
    .spi_q_size = 1,
    .cs_pin     = GY_PIN_CS,
    .ldac_pin   = GY_PIN_LDAC,
    .rst_pin    = GY_PIN_RST,
};

static dacx0004_idf6_arg_t laser_arg = {
    .host       = LASER_SPI_HOST,
    .clk_freq   = LASER_CLK_HZ,
    .spi_q_size = 1,
    .sync_pin   = LASER_PIN_SYNC,
    .ldac_pin   = LASER_PIN_LDAC,
    .clr_pin    = LASER_PIN_CLR,
    .miso_pin   = -1,
};

void app_main(void)
{
    // -- Galvo SPI bus (shared by galvo X and Y) ----------------------------
    spi_bus_config_t galvo_bus = {
        .mosi_io_num     = GALVO_PIN_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = GALVO_PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(GALVO_SPI_HOST, &galvo_bus, SPI_DMA_CH_AUTO));

    // -- Laser SPI bus -------------------------------------------------------
    spi_bus_config_t laser_bus = {
        .mosi_io_num     = LASER_PIN_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = LASER_PIN_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 0,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LASER_SPI_HOST, &laser_bus, SPI_DMA_CH_AUTO));

    // -- DAC devices ---------------------------------------------------------
    ESP_ERROR_CHECK(dac8871_init_dev(&galvo_x, &dac8871_if_idf6, &galvo_x_arg) == DAC8871_STAT_OK ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(dac8871_init_dev(&galvo_y, &dac8871_if_idf6, &galvo_y_arg) == DAC8871_STAT_OK ? ESP_OK : ESP_FAIL);
    // Force both devices onto the shared SPI bus before any sweep so neither
    // CS floats during the other device's first transaction (lazy-init race).
    dac8871_set_code(&galvo_x, 0x8000);
    dac8871_set_code(&galvo_y, 0x8000);
    ESP_ERROR_CHECK(dacx0004_init_dev(&laser, DAC80004, &dacx0004_if_idf6, &laser_arg) == DACX0004_STAT_OK ? ESP_OK : ESP_FAIL);

    ESP_LOGI(TAG, "galvo X, galvo Y, and laser DAC initialised");

    ESP_ERROR_CHECK(dacx0004_write_update_channel(&laser, LASER_CH_RED,   0x0000) == DACX0004_STAT_OK ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(dacx0004_write_update_channel(&laser, LASER_CH_GREEN, 0x0000) == DACX0004_STAT_OK ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(dacx0004_write_update_channel(&laser, LASER_CH_BLUE,  0x0000) == DACX0004_STAT_OK ? ESP_OK : ESP_FAIL);
    ESP_LOGI(TAG, "laser red ramping");

    // Ramp red 0 -> 25% -> 0, 50 steps each way at 10 ms/step = 1 second per cycle
    // 10 ms = 1 tick at CONFIG_FREERTOS_HZ=100; smaller delays truncate to 0
    #define LASER_MAX  0xFFFF
    #define RAMP_STEPS 50
    #define STEP_MS    10

    while (1) {

    
    #define TEST_RED_RAMP 0
    #define TEST_GREEN_RAMP 0
    #define TEST_BLUE_RAMP 0
    #define TEST_GALVO_SWEEP 1

    #if TEST_RED_RAMP
        for (int i = 0; i <= RAMP_STEPS; i++) {
            uint16_t val = (uint16_t)((uint32_t)LASER_MAX * i / RAMP_STEPS);
            dacx0004_write_update_channel(&laser, LASER_CH_BLUE, val);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
        for (int i = RAMP_STEPS; i >= 0; i--) {
            uint16_t val = (uint16_t)((uint32_t)LASER_MAX * i / RAMP_STEPS);
            dacx0004_write_update_channel(&laser, LASER_CH_BLUE, val);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
    #endif

    #if TEST_GREEN_RAMP
        for (int i = 0; i <= RAMP_STEPS; i++) {
            uint16_t val = (uint16_t)((uint32_t)LASER_MAX * i / RAMP_STEPS);
            dacx0004_write_update_channel(&laser, LASER_CH_GREEN, val);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
        for (int i = RAMP_STEPS; i >= 0; i--) {
            uint16_t val = (uint16_t)((uint32_t)LASER_MAX * i / RAMP_STEPS);
            dacx0004_write_update_channel(&laser, LASER_CH_GREEN, val);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
    #endif

    #if TEST_BLUE_RAMP
        for (int i = 0; i <= RAMP_STEPS; i++) {
            uint16_t val = (uint16_t)((uint32_t)LASER_MAX * i / RAMP_STEPS);
            dacx0004_write_update_channel(&laser, LASER_CH_BLUE, val);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
        for (int i = RAMP_STEPS; i >= 0; i--) {
            uint16_t val = (uint16_t)((uint32_t)LASER_MAX * i / RAMP_STEPS);
            dacx0004_write_update_channel(&laser, LASER_CH_BLUE, val);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
    #endif

    #if TEST_GALVO_SWEEP
        for (int i = 0; i <= RAMP_STEPS; i++) {
            uint16_t val = (uint16_t)((uint32_t)0xFFFF * i / RAMP_STEPS);

            for (int repeat = 0; repeat < 5; repeat++) {
                dac8871_set_code(&galvo_x, val);
                dac8871_set_code(&galvo_y, val);
                dac8871_latch(&galvo_x);
                dac8871_latch(&galvo_y);
                vTaskDelay(pdMS_TO_TICKS(STEP_MS));
            }
        }
        for (int i = RAMP_STEPS; i >= 0; i--) {
            uint16_t val = (uint16_t)((uint32_t)0xFFFF * i / RAMP_STEPS);
            dac8871_set_code(&galvo_x, val);
            dac8871_set_code(&galvo_y, val);
            dac8871_latch(&galvo_x);
            dac8871_latch(&galvo_y);
            vTaskDelay(pdMS_TO_TICKS(STEP_MS));
        }
    #endif

    }
}
