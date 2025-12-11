#include "sensorHub.h"
#include <stdlib.h>
#include <stdio.h>

// Note if you want to read raw sensor data, you must write a read_bpm function
// that works for a different output mode (read_bpm_algo works for mode 0x03 with algo mode 0x01)


uint8_t read_bpm_algo(mxc_i2c_regs_t *i2c, uint8_t address, uint16_t *hr, uint8_t *conf, uint16_t *spo2, uint8_t *state) { 
    uint8_t byte;
    uint8_t status = sensorHub_read(i2c, address, HUB_STATUS, HUB_STATUS_INDEX, &byte, 1, CMD_DELAY);
    if(status != 0x00){
        return status;
    } 
    if(byte != 0x08 && byte != 0x00){
        return byte;
    } else if (byte == 0x00) {
      return ERR_DATA_NOT_READY;
    }

    status = sensorHub_read(i2c, address, READ_DATA_OUTPUT, READ_NUM_SAMPLES, &byte, 1, CMD_DELAY);
    if (status != 0x00) {
        return status;
    } 

    size_t data_len = byte * 6;
    uint8_t *data = (uint8_t*)malloc(data_len);
    if (data == NULL) {
        return -1;
    }

    status = sensorHub_read(i2c, address, READ_DATA_OUTPUT, READ_OUTPUT_BYTES, data, data_len, CMD_DELAY);
    if (status != 0x00) {
        free(data);
        return status;
    }

    *hr = data[0] << 8 | data[1];
    *conf = data[2];
    *spo2 = (data[3] << 8 | data[4]);
    *state = data[5];

    free(data);
    return 0;
}

uint8_t read_sensor_hub_mode(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t *mode) {
    uint8_t status = sensorHub_read(i2c, address, READ_DEVICE_MODE, DEVICE_MODE_INDEX, mode, 1, CMD_DELAY);
    return status;
}

uint8_t set_output_mode(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t mode) {
    uint8_t cmd[3] = {OUTPUT_MODE, OUTPUT_MODE_FORMAT, mode};
    uint8_t status = sensorHub_write(i2c, address, cmd, 3, CMD_DELAY);
    return status;
}

uint8_t set_interrupt_threshold(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t num_samples) {
    uint8_t cmd[3] = {OUTPUT_MODE, SET_FIFO_THRESHOLD, num_samples};
    uint8_t status = sensorHub_write(i2c, address, cmd, 3, CMD_DELAY);
    return status;
}

uint8_t set_agc_algorithm(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t enable) {
    uint8_t cmd[3] = {ENABLE_ALGORITHM, AGC_INDEX_BYTE, enable};
    uint8_t status = sensorHub_write(i2c, address, cmd, 3, CMD_DELAY_ENABLE_AGC);
    return status;
}

uint8_t set_max30101(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t enable) {
    uint8_t cmd[3] = {ENABLE_SENSOR, MAX30101_INDEX_BYTE, enable};
    uint8_t status = sensorHub_write(i2c, address, cmd, 3, CMD_DELAY_ENABLE_SENSOR);
    return status;
}

uint8_t set_hrspo2_algorithm(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t enable) {
    uint8_t cmd[3] = {ENABLE_ALGORITHM, HRSPO2_INDEX_BYTE, enable};
    uint8_t status = sensorHub_write(i2c, address, cmd, 3, CMD_DELAY_ENABLE_HRSPO2);
    return status;
}

uint8_t init_max32664(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t num_samples) {
    uint8_t status = set_output_mode(i2c, address, OUTPUT_MODE_ALGO);
    if (status != E_NO_ERROR) return status;

    status = set_interrupt_threshold(i2c, address, num_samples);
    if (status != E_NO_ERROR) return status;

    status = set_agc_algorithm(i2c, address, ENABLE);
    if (status != E_NO_ERROR) return status;

    status = set_max30101(i2c, address, ENABLE);
    if (status != E_NO_ERROR) return status;

    status = set_hrspo2_algorithm(i2c, address, ENABLE_MODE1);
    if (status != E_NO_ERROR) return status;

    return 0;

}

uint8_t reinit_max32664(mxc_i2c_regs_t *i2c, uint8_t address) {
    uint8_t status = set_agc_algorithm(i2c, address, ENABLE);
    if (status != E_NO_ERROR) return status;

    status = set_max30101(i2c, address, ENABLE);
    if (status != E_NO_ERROR) return status;

    status = set_hrspo2_algorithm(i2c, address, ENABLE_MODE1);
    if (status != E_NO_ERROR) return status;

    return 0;

}


uint8_t end_bpm_algo(mxc_i2c_regs_t *i2c, uint8_t address) {
    uint8_t status = set_agc_algorithm(i2c, address, DISABLE);
    if (status != E_NO_ERROR) return status;

    status = set_max30101(i2c, address, DISABLE);
    if (status != E_NO_ERROR) return status;

    status = set_hrspo2_algorithm(i2c, address, DISABLE);
    if (status != E_NO_ERROR) return status;

    return 0;
}


uint8_t i2c_write(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t *data, uint32_t len) {
    mxc_i2c_req_t req;
    req.i2c = i2c;
    req.addr = address;
    req.tx_buf = data;
    req.tx_len = len;
    req.rx_buf = NULL;
    req.rx_len = 0;
    req.restart = 0;
    req.callback = NULL;
    
    return MXC_I2C_MasterTransaction(&req);
}


uint8_t i2c_read(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t *rx_buf, uint32_t rx_len) {
    mxc_i2c_req_t req;
    req.i2c = i2c;
    req.addr = address;
    req.tx_buf = NULL;
    req.tx_len = 0;
    req.rx_buf = rx_buf;
    req.rx_len = rx_len;
    req.restart = 0;
    
    return MXC_I2C_MasterTransaction(&req);
}

uint8_t sensorHub_write(mxc_i2c_regs_t *i2c, uint8_t address, uint8_t *cmd, uint8_t cmd_len, uint32_t cmd_delay)
{
    i2c_write(i2c, address, cmd, cmd_len);

    MXC_Delay(MXC_DELAY_MSEC(cmd_delay));

    uint8_t statusByte;
    i2c_read(i2c, address, &statusByte, 1);

    MXC_Delay(MXC_DELAY_MSEC(CMD_DELAY));
    
    return statusByte;
}

uint8_t sensorHub_read(mxc_i2c_regs_t *i2c, uint8_t address,uint8_t family, uint8_t index, uint8_t *rxData, size_t len, uint32_t cmd_delay)
{
    uint8_t *buffer = (uint8_t*)malloc(len + 1);
    uint8_t cmd[2] = {family, index};

    if (buffer == NULL) {
        return 0xFD;
    }

    i2c_write(i2c, address, cmd, 2);
    MXC_Delay(MXC_DELAY_MSEC(cmd_delay));

    i2c_read(i2c, address, buffer, len+1);

    for (size_t i = 0; i < len; i++) {
        rxData[i] = buffer[i+1];
    }

    uint8_t status = buffer[0];
    free(buffer);
    
    MXC_Delay(MXC_DELAY_MSEC(10));

    return status;
} 