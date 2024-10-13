/**
 ******************************************************************************
 * @file        : rc522.cpp
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

#include "rc522.hpp"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* kTag = "RC522";

static int kCrcTimeOutMs = 100;
static int kComTimeOutMs = 100;
static int kMaxAnticollisionIter = 100;

static constexpr uint8_t kCommandReg = 0x01;
static constexpr uint8_t kComlEnReg = 0x02;
static constexpr uint8_t kDivlEnReg = 0x03;
static constexpr uint8_t kComIrqReg = 0x04;
static constexpr uint8_t kDivIrqReg = 0x05;
static constexpr uint8_t kErrorReg = 0x06;
static constexpr uint8_t kStatus1Reg = 0x07;
static constexpr uint8_t kStatus2Reg = 0x08;
static constexpr uint8_t kFifoDataReg = 0x09;
static constexpr uint8_t kFifoLevelReg = 0x0A;
static constexpr uint8_t kWaterLevelReg = 0x0B;
static constexpr uint8_t kControlReg = 0x0C;
static constexpr uint8_t kBitFramingReg = 0x0D;
static constexpr uint8_t kCollReg = 0x0E;
static constexpr uint8_t kModeReg = 0x11;
static constexpr uint8_t kTxModeReg = 0x12;
static constexpr uint8_t kRxModeReg = 0x13;
static constexpr uint8_t kTxControlReg = 0x14;
static constexpr uint8_t kTxAskReg = 0x15;
static constexpr uint8_t kTxSelReg = 0x16;
static constexpr uint8_t kRxSelReg = 0x17;
static constexpr uint8_t kRxThresholdReg = 0x18;
static constexpr uint8_t kDemodReg = 0x19;
static constexpr uint8_t kMfTxReg = 0x1C;
static constexpr uint8_t kMfRxReg = 0x1D;
static constexpr uint8_t kSerialSpeedReg = 0x1F;
static constexpr uint8_t kCrcResultReg = 0x21;
static constexpr uint8_t kCrcResultRegH = 0x21;
static constexpr uint8_t kCrcResultRegL = 0x22;
static constexpr uint8_t kModWidthReg = 0x24;
static constexpr uint8_t kRfCfgReg = 0x26;
static constexpr uint8_t kGsNReg = 0x27;
static constexpr uint8_t kCwGsPReg = 0x28;
static constexpr uint8_t kModGsPReg = 0x29;
static constexpr uint8_t kTModeReg = 0x2A;
static constexpr uint8_t kTPrescalerReg = 0x2B;
static constexpr uint8_t kTReloadRegH = 0x2C;
static constexpr uint8_t kTReloadRegL = 0x2D;
static constexpr uint8_t kTCounterValReg = 0x2E;
static constexpr uint8_t kTestSel1Reg = 0x31;
static constexpr uint8_t kTestSel2Reg = 0x32;
static constexpr uint8_t kTestPinEnReg = 0x33;
static constexpr uint8_t kTestPinValueReg = 0x34;
static constexpr uint8_t kTestBusReg = 0x35;
static constexpr uint8_t kAutoTestReg = 0x36;
static constexpr uint8_t kVersionReg = 0x37;
static constexpr uint8_t kAnalogTestReg = 0x38;
static constexpr uint8_t kTestDac1Reg = 0x39;
static constexpr uint8_t kTestDac2Reg = 0x3A;
static constexpr uint8_t kTestAdcReg = 0x3B;

static constexpr uint8_t kIdleCmd = 0x00;
static constexpr uint8_t kMemCmd = 0x01;
static constexpr uint8_t kGenerateRandomIdCmd = 0x02;
static constexpr uint8_t kCalcCrcCmd = 0x03;
static constexpr uint8_t kTransmitCmd = 0x04;
static constexpr uint8_t kNoCmdChangeCmd = 0x05;
static constexpr uint8_t kReceiveCmd = 0x08;
static constexpr uint8_t kTransceiveCmd = 0x0C;
static constexpr uint8_t kMfAuthentCmd = 0x0E;
static constexpr uint8_t kSoftResetCmd = 0x0F;

static constexpr uint8_t kPiccCmdReqA = 0x26;
static constexpr uint8_t kPiccCmdWupA = 0x52;
static constexpr uint8_t kPiccCmdCt = 0x88;
static constexpr uint8_t kPiccCmdSelCl1 = 0x93;
static constexpr uint8_t kPiccCmdSelCl2 = 0x95;
static constexpr uint8_t kPiccCmdSelCl3 = 0x97;
static constexpr uint8_t kPiccCmdHltA = 0x50;
static constexpr uint8_t kPiccCmdRats = 0xE0;
static constexpr uint8_t kPiccCmdMfAuthKeyA = 0x60;
static constexpr uint8_t kPiccCmdMfAuthKeyB = 0x61;
static constexpr uint8_t kPiccCmdMfRead = 0x30;
static constexpr uint8_t kPiccCmdMfWrite = 0xA0;
static constexpr uint8_t kPiccCmdMfDecrement = 0xC0;
static constexpr uint8_t kPiccCmdMfIncrement = 0xC1;
static constexpr uint8_t kPiccCmdMfRestore = 0xC2;
static constexpr uint8_t kPiccCmdMfTransfer = 0xB0;
static constexpr uint8_t kPiccCmdUlWrite = 0xA2;

esp_err_t RC522::InitSpiBus(spi_host_device_t host_device,
                            int miso_pin,
                            int mosi_pin,
                            int sck_pin,
                            spi_device_handle_t* spi_handle) {
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = miso_pin;
    buscfg.mosi_io_num = mosi_pin;
    buscfg.sclk_io_num = sck_pin;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;
    esp_err_t err = spi_bus_initialize(host_device, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to initialize SPI bus: %d", err);
        return err;
    }

    spi_device_interface_config_t dev_config = {};
    dev_config.command_bits = 0;
    dev_config.address_bits = 0;
    dev_config.dummy_bits = 0;
    dev_config.mode = 0;
    dev_config.clock_speed_hz = 1000000;  // 1MHz
    dev_config.queue_size = 2;

    err = spi_bus_add_device(host_device, &dev_config, spi_handle);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "Failed to add SPI device: %d", err);
        return err;
    }
    ESP_LOGI(kTag, "SPI bus initialized OK");
    return ESP_OK;
}

RC522::RC522(int ss_pin, int reset_pin, int irq_pin, spi_device_handle_t spi_handle)
    : ss_pin_(ss_pin), reset_pin_(reset_pin), irq_pin_(irq_pin), spi_handle_(spi_handle) {
    ESP_ERROR_CHECK(gpio_set_level(static_cast<gpio_num_t>(ss_pin_), 0));
    if (reset_pin_ >= 0) {
        ESP_ERROR_CHECK(gpio_set_level(static_cast<gpio_num_t>(reset_pin_), 1));
    }

    gpio_config_t io_conf = {};

    // Configure ss and rst pin as input
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << ss_pin_);
    if (reset_pin >= 0) {
        io_conf.pin_bit_mask |= (1ULL << reset_pin_);
    }
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    if (irq_pin_ >= 0) {
        // Configure irq pin as input
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << irq_pin_);
        ESP_ERROR_CHECK(gpio_config(&io_conf));
    }

    if (reset_pin_ >= 0) {
        HardReset();
    }
    SoftReset();

    WriteReg(kTxModeReg, 0x00);
    WriteReg(kRxModeReg, 0x00);
    WriteReg(kModWidthReg, 0x26);
    WriteReg(kTModeReg, 0x80);
    WriteReg(kTPrescalerReg, 0xA9);
    WriteReg(kTReloadRegH, 0x03);
    WriteReg(kTReloadRegL, 0xE8);
    WriteReg(kTxAskReg, 0x40);
    WriteReg(kModeReg, 0x3D);
    // AntennaOn();

    ESP_LOGI(kTag, "initialized OK");
}

void RC522::Enable() { gpio_set_level(static_cast<gpio_num_t>(ss_pin_), 0); }
void RC522::Disable() { gpio_set_level(static_cast<gpio_num_t>(ss_pin_), 1); }

// ----- Register access methods -----

void RC522::WriteReg(uint8_t reg, uint8_t value) {
    spi_transaction_t transaction = {};

    ESP_ERROR_CHECK(spi_device_acquire_bus(spi_handle_, portMAX_DELAY));

    uint8_t tx_buffer[2] = {static_cast<uint8_t>(reg << 1), value};
    transaction.length = 16;
    transaction.tx_buffer = tx_buffer;
    transaction.rx_buffer = NULL;

    Enable();
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &transaction));
    Disable();

    spi_device_release_bus(spi_handle_);
}

void RC522::WriteRegs(uint8_t reg, uint8_t* values, size_t len) {
    spi_transaction_t transaction = {};

    ESP_ERROR_CHECK(spi_device_acquire_bus(spi_handle_, portMAX_DELAY));
    uint8_t tx_buffer = reg << 1;
    transaction.length = 8;
    transaction.tx_buffer = &tx_buffer;
    transaction.rx_buffer = NULL;

    Enable();
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &transaction));
    transaction.length = 8 * len;
    transaction.tx_buffer = values;
    transaction.rx_buffer = NULL;
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &transaction));
    Disable();

    spi_device_release_bus(spi_handle_);
}

uint8_t RC522::ReadReg(uint8_t reg) {
    spi_transaction_t transaction = {};

    ESP_ERROR_CHECK(spi_device_acquire_bus(spi_handle_, portMAX_DELAY));

    uint8_t tx_buffer[2] = {static_cast<uint8_t>(0x80 | (reg << 1)), 0x00};
    uint8_t rx_buffer[2] = {0};

    transaction.length = 16;
    transaction.tx_buffer = tx_buffer;
    transaction.rx_buffer = rx_buffer;

    Enable();
    ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &transaction));
    Disable();
    spi_device_release_bus(spi_handle_);

    return rx_buffer[1];
}

void RC522::ReadRegs(uint8_t reg, uint8_t* values, size_t len) {
    spi_transaction_t transaction = {};
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi_handle_, portMAX_DELAY));

    Enable();

    for (int i = 0; i <= len; i++) {  // Note: i <= len and not i < len !
        uint8_t tx_buffer = 0x80 | (i < len ? (reg << 1) : 0x00);
        uint8_t rx_buffer = 0;
        transaction.length = 8;
        transaction.tx_buffer = &tx_buffer;
        transaction.rx_buffer = &rx_buffer;

        ESP_ERROR_CHECK(spi_device_transmit(spi_handle_, &transaction));
        if (i > 0) values[i - 1] = rx_buffer;
    }
    Disable();

    spi_device_release_bus(spi_handle_);
}

void RC522::ClearRegBits(uint8_t reg, uint8_t bits, bool always) {
    uint8_t value = ReadReg(reg);
    if (always || ((value & bits) != 0)) {
        WriteReg(reg, value & ~bits);
    }
}

void RC522::SetRegBits(uint8_t reg, uint8_t bits, bool always) {
    uint8_t value = ReadReg(reg);
    if (always || ((value & bits) != bits)) {
        WriteReg(reg, value | bits);
    }
}

// ----- Other methods -----

void RC522::HardReset() {
    if (reset_pin_ < 0) {
        ESP_LOGW(kTag, "No reset pin defined");
        return;
    }
    ESP_LOGD(kTag, "Hard reset");
    Disable();
    ESP_ERROR_CHECK(gpio_set_level(static_cast<gpio_num_t>(reset_pin_), 0));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(gpio_set_level(static_cast<gpio_num_t>(reset_pin_), 1));
    vTaskDelay(pdMS_TO_TICKS(100));
}

void RC522::SoftReset() {
    WriteReg(kCommandReg, kSoftResetCmd);

    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
        uint8_t status = ReadReg(kCommandReg);
        if ((status & 0x10) == 0) return;
    }
    ESP_LOGE(kTag, "Soft reset failed");
}

// ----- MFRC522 specific methods -----

void RC522::AntennaOn() {
    uint8_t value = ReadReg(kTxControlReg);
    if ((value & 0x03) == 0) {
        WriteReg(kTxControlReg, value | 0x03);
    }
}

void RC522::AntennaOff() {
    uint8_t value = ReadReg(kTxControlReg);
    if ((value & 0x03) != 0) {
        WriteReg(kTxControlReg, value & (~0x03));
    }
}

uint8_t RC522::GetAntennaGain() { return (ReadReg(kRfCfgReg) >> 4) & 0x07; }
esp_err_t RC522::SetAntennaGain(uint8_t gain) {
    if (gain > 7) {
        ESP_LOGE(kTag, "Invalid gain value");
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t value = ReadReg(kRfCfgReg);
    if (((value >> 4) & 0x07) != gain) {
        value = (value & ~0x70) | (gain << 4);
        WriteReg(kRfCfgReg, value);
    }
    return ESP_OK;
}

void RC522::LogVersion() {
    uint8_t version = ReadReg(kVersionReg);
    char buffer[64];
    if ((version & 0xf0) == 0x90) {
        snprintf(buffer, sizeof(buffer), "MFRC522 v%d.0 (0x%02X)", version & 0x0f, version);
    } else if (version == 0x12) {
        snprintf(buffer, sizeof(buffer), "counterfeit chip (0x12)");
    } else if (version == 0x88) {
        snprintf(buffer, sizeof(buffer), "FM17522 (0x88)");
    } else if (version == 0x89) {
        snprintf(buffer, sizeof(buffer), "FM17522E (0x89)");
    } else if (version == 0xb2) {
        snprintf(buffer, sizeof(buffer), "FM17522_1 (0xb2)");
    } else {
        snprintf(buffer, sizeof(buffer), "unknown (0x%02X)", version);
    }
    ESP_LOGI(kTag, "Chip version : %s", buffer);
}

esp_err_t RC522::CalculateCRC(uint8_t* data, size_t len, uint16_t* result) {
    WriteReg(kCommandReg, kIdleCmd);     // Stop any active command.
    WriteReg(kDivIrqReg, 0x04);          // Clear the CRCIRq interrupt request bit
    WriteReg(kFifoLevelReg, 0x80);       // Flush the FIFO buffer
    WriteRegs(kFifoDataReg, data, len);  // Write data to the FIFO
    WriteReg(kCommandReg, kCalcCrcCmd);

    int64_t deadline = esp_timer_get_time() / 1000 + kCrcTimeOutMs;

    do {
        uint8_t irq = ReadReg(kDivIrqReg);
        if (irq & 0x04) {                     // Completed
            WriteReg(kCommandReg, kIdleCmd);  // Stop any active command.
            *result = (ReadReg(kCrcResultRegH) << 8) | ReadReg(kCrcResultRegL);
            ESP_LOGD(kTag, "CRC: %04X", *result);
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    } while (esp_timer_get_time() / 1000 < deadline);

    reason_ = kCrcTimeout;
    return ESP_ERR_TIMEOUT;
}

esp_err_t RC522::Communicate(uint8_t cmd,
                             uint8_t wait_irq,
                             uint8_t* tx_data,
                             size_t tx_len,
                             uint8_t* rx_data,
                             size_t* rx_len,
                             uint8_t* tx_last_bits_ptr,
                             uint8_t rx_align,
                             bool check_crc) {
    reason_ = kOk;
    uint8_t tx_last_bits = tx_last_bits_ptr ? *tx_last_bits_ptr : 0;
    uint8_t bit_framing = ((rx_align & 0x7) << 4) | (tx_last_bits & 0x7);

    ESP_LOGD(kTag,
             "Communicate: cmd=0x%02X, wait_irq=0x%02X, tx_len=%d, rx_len=%d, bit_framing=0x%02X",
             cmd,
             wait_irq,
             tx_len,
             rx_len ? *rx_len : 0,
             bit_framing);

    WriteReg(kCommandReg, kIdleCmd);           // Stop any active command.
    WriteReg(kComIrqReg, 0x7F);                // Clear all interrupt request bits
    WriteReg(kFifoLevelReg, 0x80);             // Flush the FIFO buffer
    WriteRegs(kFifoDataReg, tx_data, tx_len);  // Write data to the FIFO
    WriteReg(kBitFramingReg, bit_framing);     // Bit adjustments
    WriteReg(kCommandReg, cmd);                // Execute the command

    if (cmd == kTransceiveCmd) {
        SetRegBits(kBitFramingReg, 0x80);  // Start sending data
    }

    int64_t deadline = esp_timer_get_time() / 1000 + kComTimeOutMs;
    bool completed = false;

    do {
        uint8_t irq = ReadReg(kComIrqReg);
        if (irq & wait_irq) {
            completed = true;
            break;
        }
        if (irq & 0x01) {
            reason_ = kComIrqTimeout;
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (esp_timer_get_time() / 1000 < deadline);

    if (!completed) {
        reason_ = kComTimeout;
        return ESP_ERR_TIMEOUT;
    }

    uint8_t error = ReadReg(kErrorReg);
    if (error & 0x13) {  // Buffer overflow, Parity error, Protocol error
        reason_ = kBufferOverflow;
        return ESP_FAIL;
    }

    uint8_t vb = 0;
    if (rx_data && rx_len) {
        uint8_t fifo_level = ReadReg(kFifoLevelReg);
        if (fifo_level > *rx_len) {
            return ESP_ERR_INVALID_ARG;
        }
        *rx_len = fifo_level;
        ReadRegs(kFifoDataReg, rx_data, *rx_len);
        vb = ReadReg(kControlReg) & 0x07;
        if (tx_last_bits_ptr) {
            *tx_last_bits_ptr = vb;
        }
    }

    if (error & 0x08) {  // Collision error
        reason_ = kCollision;
        return ESP_FAIL;
    }

    if (rx_data && rx_len && check_crc) {
        if (*rx_len < 2 || vb != 0) {
            return ESP_ERR_INVALID_ARG;
        }
        uint16_t crc;
        esp_err_t err = CalculateCRC(rx_data, *rx_len - 2, &crc);
        if (err != ESP_OK) {
            return err;
        }

        uint16_t rx_crc = (rx_data[*rx_len - 1] << 8) | rx_data[*rx_len - 2];
        if (crc != rx_crc) {
            ESP_LOGW(kTag, "CRC error: %04X != %04X", crc, rx_crc);
            return ESP_ERR_INVALID_CRC;
        }
    }
    return ESP_OK;
}

esp_err_t RC522::TransceiveData(uint8_t* tx_data,
                                size_t tx_len,
                                uint8_t* rx_data,
                                size_t* rx_len,
                                uint8_t* tx_last_bits_ptr,
                                uint8_t rx_align,
                                bool check_crc) {
    return Communicate(
        kTransceiveCmd, 0x30, tx_data, tx_len, rx_data, rx_len, tx_last_bits_ptr, check_crc);
}

esp_err_t RC522::PiccSendShortFrame(uint8_t command) {
    uint8_t tx_buffer[] = {command};
    uint8_t rx_buffer[2];
    size_t rx_size = sizeof(rx_buffer);
    uint8_t valid_bits = 7;
    return TransceiveData(tx_buffer, sizeof(tx_buffer), rx_buffer, &rx_size, &valid_bits);
}

esp_err_t RC522::PiccReqA() { return PiccSendShortFrame(kPiccCmdReqA); }

esp_err_t RC522::PiccWupA() { return PiccSendShortFrame(kPiccCmdWupA); }

esp_err_t RC522::PiccHaltA() {
    uint8_t buffer[4] = {kPiccCmdHltA, 0};
    uint16_t crc;
    esp_err_t res = CalculateCRC(buffer, 2, &crc);
    if (res != ESP_OK) {
        return res;
    }
    buffer[2] = crc & 0xFF;  // LSB
    buffer[3] = crc >> 8;    // MSB

    res = TransceiveData(buffer, sizeof(buffer), nullptr, 0);

    // Only a timeout is considered a success
    if (res == ESP_ERR_TIMEOUT) {
        return ESP_OK;
    }
    if (res == ESP_OK) {
        return ESP_FAIL;
    }
    return res;
}

esp_err_t RC522::AnticollisionLoop(uint8_t cascade_level, uint32_t* uid) {
    uint8_t uid_valid_bits = 0;
    uint8_t buffer[9] = {0};
    for (int count = 0; count < kMaxAnticollisionIter; count++) {
        uint8_t* rx_buffer = nullptr;
        size_t rx_len = 0;
        size_t tx_len = 0;

        uint8_t byte_count = (2 + (uid_valid_bits / 8)) & 0x07;
        uint8_t bit_count = uid_valid_bits & 0x07;

        buffer[0] = cascade_level;
        if (uid_valid_bits >= 32) {  // SELECT
            buffer[1] = 0x70;
            buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
            uint16_t crc;
            esp_err_t res = CalculateCRC(buffer, 7, &crc);
            if (res != ESP_OK) {
                return res;
            }
            buffer[7] = crc & 0xFF;  // LSB
            buffer[8] = crc >> 8;    // MSB
            tx_len = 9;
            rx_buffer = buffer + 6;
            rx_len = 3;
        } else {  // ANTICOLLISION
            // byte_count also contains 2 additional bytes, namely SEL and NVB
            buffer[1] = (byte_count << 4) | (bit_count);
            tx_len = byte_count + ((bit_count > 0) ? 1 : 0);
            rx_buffer = buffer + byte_count;
            rx_len = sizeof(buffer) - byte_count;
        }

        uint8_t rx_align = bit_count;
        uint8_t tx_last_bits = (8 - bit_count) & 0xf;

        esp_err_t res = TransceiveData(buffer, tx_len, rx_buffer, &rx_len, &tx_last_bits, rx_align);

        if (res == ESP_FAIL && reason_ == kCollision) {
            uint8_t coll_pos = ReadReg(kCollReg);
            ESP_LOGI(kTag, "Collision detected at position %d", coll_pos);
            if ((coll_pos & 0x20) != 0) {
                return ESP_FAIL;
            }
            coll_pos &= 0x1F;
            if (coll_pos == 0) {
                coll_pos = 32;
            }
            if (coll_pos <= uid_valid_bits) {
                reason_ = kInternalError;
                return ESP_FAIL;
            }
            uid_valid_bits = coll_pos;
            buffer[2 + (coll_pos - 1) / 8] |= (1 << ((coll_pos - 1) & 0x07));
        } else if (res != ESP_OK) {
            return res;
        } else {  // res == ESP_OK
            if (uid_valid_bits >= 32) {
                *uid = static_cast<uint32_t>(buffer[2]) << 24 |
                       static_cast<uint32_t>(buffer[3]) << 16 |
                       static_cast<uint32_t>(buffer[4]) << 8 |
                       static_cast<uint32_t>(buffer[5]) << 0;
                return ESP_OK;
            } else {
                uid_valid_bits = 32;
            }
        }
    }
    reason_ = kInternalError;
    return ESP_FAIL;
}

esp_err_t RC522::PiccSelect(uint32_t* uid) {
    ClearRegBits(kCollReg, 0x80);  //  all received bits will be cleared after a collision
    return AnticollisionLoop(kPiccCmdSelCl1, uid);
}

bool RC522::IsCardPresent() {
    WriteReg(kTxModeReg, 0x00);    // Reset Tx data rates
    WriteReg(kRxModeReg, 0x00);    // Reset Rx data rates
    WriteReg(kModWidthReg, 0x26);  // Reset modulation width
    ClearRegBits(kCollReg, 0x80);  //  all received bits will be cleared after a collision
    esp_err_t result = PiccReqA();
    return (result == ESP_OK || (result == ESP_FAIL && reason_ == kCollision));
}

esp_err_t RC522::ReadCardUID(uint32_t* uid) { return PiccSelect(uid); }