/**
 ******************************************************************************
 * @file        : main.cpp
 * @brief       : rc522 example
 * @author      : Jacques Supcik <jacques@supcik.net>
 * @date        : 13 October 2024
 ******************************************************************************
 * @copyright   : Copyright (c) 2024 Jacques Supcik
 * @attention   : SPDX-License-Identifier: MIT
 ******************************************************************************
 * @details
 *
 ******************************************************************************
 */

#include <inttypes.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "rc522.hpp"

const char* kTag = "RC522 (example)";

static const int kMISOPin = 6;
static const int kMOSIPin = 7;
static const int kSCKPin = 15;

static const int kSsPin = 4;
static const int kResetPin = 5;

extern "C" {
void app_main(void);
}

void app_main(void) {
    ESP_LOGI(kTag, "Starting rc522 test");
    RC522::InitSpiBus(SPI2_HOST, kMISOPin, kMOSIPin, kSCKPin);
    RC522 rc522(SPI2_HOST, kSsPin, kResetPin, -1);
    rc522.LogVersion();

    while (true) {
        rc522.AntennaOn();
        if (rc522.IsCardPresent()) {
            uint32_t uid;
            esp_err_t err = rc522.ReadCardUID(&uid);
            if (err == ESP_OK) {
                rc522.PiccReqA();
                ESP_LOGI(kTag, "Card UID: 0x%08" PRIX32, uid);
            } else {
                ESP_LOGE(kTag, "Failed to read card UID: %d", err);
            }
        } else {
            ESP_LOGI(kTag, "No card present");
        }
        rc522.AntennaOff();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
