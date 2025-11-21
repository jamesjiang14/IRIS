#include "adxl345.h"
#include "i2c.h"
#include "mxc_device.h"
#include <stdio.h> // For NULL

// --- CONFIGURATION ---
// Maxim uses the 7-bit address (0x53). Do NOT shift it left by 1 (do not use 0xA6).
#define ADXL_ADDR_7BIT  0x53 
#define I2C_INSTANCE    MXC_I2C0  // Change to MXC_I2C0 if using those pins

// Global buffers
uint8_t data_rec[6];
uint8_t chipid = 0;

// --- WRITE FUNCTION ---
void adxl_write(uint8_t reg, uint8_t value)
{
    int error;
    uint8_t tx_data[2];
    
    tx_data[0] = reg;
    tx_data[1] = value;

    // Create the transaction request
    mxc_i2c_req_t reqMaster;
    reqMaster.i2c = I2C_INSTANCE;    // The "Handler" is the hardware instance
    reqMaster.addr = ADXL_ADDR_7BIT; // 7-bit address
    reqMaster.tx_buf = tx_data;      // Data to send
    reqMaster.tx_len = 2;            // Send Reg then Value
    reqMaster.rx_buf = NULL;         // Not reading
    reqMaster.rx_len = 0;
    
    error = MXC_I2C_MasterTransaction(&reqMaster);
    
    if (error != E_NO_ERROR) {
        // Optional: Handle error
    }
}

// --- READ FUNCTION (Replaces HAL_I2C_Mem_Read) ---
// This function writes the register address, sends a RESTART, then reads data
void adxl_read_values(uint8_t reg)
{
    int error;
    uint8_t reg_addr = reg; // The register we want to read from

    mxc_i2c_req_t reqMaster;
    reqMaster.i2c = I2C_INSTANCE;
    reqMaster.addr = ADXL_ADDR_7BIT;
    
    // In Maxim SDK, setting BOTH tx and rx creates a "Write-Restart-Read"
    reqMaster.tx_buf = &reg_addr;    // 1. Write the register address
    reqMaster.tx_len = 1;
    reqMaster.rx_buf = data_rec;     // 2. Read into the global buffer
    reqMaster.rx_len = 6;            // 3. Read 6 bytes (X, Y, Z)
    
    error = MXC_I2C_MasterTransaction(&reqMaster);
    
    if (error != E_NO_ERROR) {
        // Optional: Handle error
    }
}

// --- READ SINGLE BYTE (For Device ID) ---
void adxl_read_address(uint8_t reg)
{
    int error;
    uint8_t reg_addr = reg;

    mxc_i2c_req_t reqMaster;
    reqMaster.i2c = I2C_INSTANCE;
    reqMaster.addr = ADXL_ADDR_7BIT;
    
    reqMaster.tx_buf = &reg_addr;
    reqMaster.tx_len = 1;
    reqMaster.rx_buf = &chipid;      // Store result in chipid variable
    reqMaster.rx_len = 1;            // Read only 1 byte
    
    error = MXC_I2C_MasterTransaction(&reqMaster);
}

// --- INIT AND HELPERS (Logic remains mostly the same) ---
void adxl_init(void)
{
    adxl_read_address(0x00); // read the DEVID

    // logic change: ensure we aren't stuck in sleep before settings
    adxl_write(0x2D, 0x00);  // reset power control
    adxl_write(0x31, 0x01);  // data_format range= +- 4g
    adxl_write(0x2D, 0x08);  // measure mode
}

int16_t adxl_readx(void)
{
    int16_t x;
    adxl_read_values(0x32); // This fills data_rec[0] through data_rec[5]
    x = ((data_rec[1] << 8) | data_rec[0]);
    return x;
}

int16_t adxl_ready(void)
{
    int16_t y;
    // Note: Optimized to not re-read I2C 3 times. 
    // Calling adxl_read_values once updates X, Y, and Z buffers together.
    // If you call read_values here again, it's fine, just slightly slower.
    y = ((data_rec[3] << 8) | data_rec[2]);
    return y;
}

int16_t adxl_readz(void)
{
    int16_t z;
    z = ((data_rec[5] << 8) | data_rec[4]);
    return z;
}