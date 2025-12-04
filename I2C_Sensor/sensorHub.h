#ifndef MAX32664_SENSOR_HUB_H
#define MAX32664_SENSOR_HUB_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "i2c.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "board.h"
#include "mxc_errors.h"
#include "gpio.h"

// Family Bytes
#define HUB_STATUS 0x00
#define SET_DEVICE_MODE 0x01
#define READ_DEVICE_MODE 0x02
#define OUTPUT_MODE 0x10
#define READ_OUTPUT_MODE 0x11
#define READ_DATA_OUTPUT 0x12
#define READ_DATA_INPUT 0x13
#define WRITE_REGISTER 0x40
#define READ_REGISTER 0x41
#define READ_ATTRIBUTES_AFE 0x42
#define DUMP_REGISTERS 0x43
#define ENABLE_SENSOR 0x44
#define READ_SENSOR_MODE 0x45
#define CHANGE_ALGORITHM_CONFIG 0x50
#define READ_ALGORITHM_CONFIG 0x51
#define ENABLE_ALGORITHM 0x52
#define BOOTLOADER_FLASH 0x80
#define BOOTLOADER_INFO 0x81
#define IDENTITY 0xFF

// Index Bytes
#define READ_NUM_SAMPLES 0x00
#define HUB_STATUS_INDEX 0x00
#define DEVICE_MODE_INDEX 0x00
#define OUTPUT_MODE_FORMAT 0x00
#define SET_FIFO_THRESHOLD 0x01
#define READ_OUTPUT_BYTES 0x01
#define AGC_INDEX_BYTE 0x00
#define HRSPO2_INDEX_BYTE 0x02
#define MAX30101_INDEX_BYTE 0x03
#define ENABLE_ACCELEROMETER 0x04

// Output Modes
#define OUTPUT_MODE_PAUSE 0x00
#define OUTPUT_MODE_SENSOR 0x01
#define OUTPUT_MODE_ALGO 0x02
#define OUTPUT_MODE_SENSOR_ALGO 0x03
#define OUTPUT_MODE_SAMPLE_SENSOR 0x05
#define OUTPUT_MODE_SAMPLE_ALGO 0x06
#define OUTPUT_MODE_SAMPLE_SENSOR_ALGO 0x07

// Command Delays, Enable, and Error Codes
#define CMD_DELAY 2
#define CMD_DELAY_ENABLE_AGC 20
#define CMD_DELAY_ENABLE_SENSOR 40
#define DISABLE 0x00
#define ENABLE 0x01
#define ENABLE_MODE1 0x01
#define ENABLE_MODE2 0x02
#define E_NO_ERROR 0
#define ERR_UNKNOWN 0xFF
#define ERR_DATA_NOT_READY 0x06

// --- Core I2C Driver Wrappers ---

uint8_t i2c_read(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t *rx_buf, uint32_t rx_len);
uint8_t i2c_write(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t *data, uint32_t len);

// --- Core Sensor Hub I/O ---

/**
 * @brief Writes a command to the Sensor Hub and reads the resulting status byte.
 * @param i2c Pointer to the I2C peripheral registers.
 * @param address The I2C slave address of the sensor hub.
 * @param cmd Pointer to the command buffer.
 * @param cmd_len Length of the command buffer.
 * @param cmd_delay Delay in milliseconds after writing the command.
 * @return The status byte returned by the sensor hub.
 */
uint8_t sensorHub_write(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t *cmd, uint8_t cmd_len, uint32_t cmd_delay);

/**
 * @brief Sends a read command to the Sensor Hub, reads the status, and extracts data.
 * @param i2c Pointer to the I2C peripheral registers.
 * @param address The I2C slave address of the sensor hub.
 * @param family The command family byte.
 * @param index The command index byte.
 * @param rxData Pointer to the buffer to receive the data bytes.
 * @param len The number of data bytes expected (excluding status byte).
 * @param cmd_delay Delay in milliseconds after writing the command.
 * @return The status byte (register 0x00) returned by the sensor hub.
 */
uint8_t sensorHub_read(mxc_i2c_regs_t *i2c, uint8_t address,uint8_t family, uint8_t index, 
                       uint8_t *rxData, size_t len, uint32_t cmd_delay);


// --- High-Level Configuration/Control Functions ---

uint8_t read_bpm_algo(mxc_i2c_regs_t *i2c, uint8_t address, uint16_t *hr, uint8_t *conf, 
                  uint16_t *spo2, uint8_t *state);

uint8_t read_sensor_hub_mode(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t *mode);

uint8_t set_output_mode(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t mode);

uint8_t set_interrupt_threshold(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t num_samples);

uint8_t enable_agc_algorithm(mxc_i2c_regs_t *i2c, uint8_t address);

uint8_t enable_max30101(mxc_i2c_regs_t *i2c, uint8_t address);

uint8_t enable_hrspo2_algorithm(mxc_i2c_regs_t *i2c, uint8_t address);

uint8_t init_max32664(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t num_samples);


#endif