/**
 ******************************************************************************
 * @file        : rc522.hpp
 * @brief       : RC522 RFID controller
 * @author      : Jacques Supcik <jacques@supcik.net>
 * @date        : 8 August 2024
 ******************************************************************************
 * @copyright   : Copyright (c) 2024 Jacques Supcik
 * @attention   : SPDX-License-Identifier: MIT
 ******************************************************************************
 * @details     : RC522 RFID controller
 ******************************************************************************
 */

#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

class RC522 {
   public:
    enum Gain {
        gain180dB = 0x00,
        gain230dB = 0x01,
        gain18dB = 0x02,
        gain23dB = 0x03,
        gain33dB = 0x04,
        gain38dB = 0x05,
        gain43dB = 0x06,
        gain48dB = 0x07,
    };

    enum Reason {
        kOk = 0,
        kCollision,
        kBufferOverflow,
        kInternalError,
        kCrcTimeout,
        kComIrqTimeout,
        kComTimeout,
    };

    static esp_err_t InitSpiBus(spi_host_device_t host_device,
                                int miso_pin,
                                int mosi_pin,
                                int sck_pin,
                                spi_device_handle_t* spi_handle);

    RC522(int ss_pin, int reset_pin, int irq_pin, spi_device_handle_t spi_handle);
    virtual ~RC522() = default;

    void HardReset();
    void SoftReset();

    void AntennaOn();
    void AntennaOff();

    uint8_t GetAntennaGain();
    esp_err_t SetAntennaGain(uint8_t gain);

    void LogVersion();

    esp_err_t PiccReqA();
    esp_err_t PiccWupA();
    esp_err_t PiccHaltA();

    esp_err_t PiccSelect(uint32_t* uid);
    esp_err_t ReadCardUID(uint32_t* uid);

    bool IsCardPresent();

    Reason reason_;

   private:
    int ss_pin_;
    int reset_pin_;
    int irq_pin_;
    spi_device_handle_t spi_handle_;

    void Enable();
    void Disable();

    void WriteReg(uint8_t reg, uint8_t value);
    void WriteRegs(uint8_t reg, uint8_t* values, size_t len);
    uint8_t ReadReg(uint8_t reg);
    void ReadRegs(uint8_t reg, uint8_t* values, size_t len);

    void ClearRegBits(uint8_t reg, uint8_t bits, bool always = false);
    void SetRegBits(uint8_t reg, uint8_t bits, bool always = false);

    esp_err_t CalculateCRC(uint8_t* data, size_t len, uint16_t* result);

    esp_err_t Communicate(uint8_t cmd,
                          uint8_t wait_irq,
                          uint8_t* tx_data,
                          size_t tx_len,
                          uint8_t* rx_data = nullptr,
                          size_t* rx_len = nullptr,
                          uint8_t* valid_bits = nullptr,
                          uint8_t rx_align = 0,
                          bool check_crc = false);

    esp_err_t TransceiveData(uint8_t* tx_data,
                             size_t tx_len,
                             uint8_t* rx_data = nullptr,
                             size_t* rx_len = nullptr,
                             uint8_t* valid_bits = nullptr,
                             uint8_t rx_align = 0,
                             bool check_crc = false);
    esp_err_t AnticollisionLoop(uint8_t cascade_level, uint32_t* uid);
    esp_err_t PiccSendShortFrame(uint8_t command);
};
