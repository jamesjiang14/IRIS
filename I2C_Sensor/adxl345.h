#ifndef ADXL345_H
#define ADXL345_H

#include <stdint.h>
#include "mxc_device.h"
#include "i2c.h"


//Adxl345 Device Address 
#define adxl_address 0x53

//Set i2c Handeler here
#define ADXL_I2C_INST MXC_I2C1;



void adxl_write (uint8_t reg, uint8_t value);
void adxl_read_values (uint8_t reg);
void adxl_read_address (uint8_t reg);
void adxl_init (void);
int16_t adxl_readx(void);
int16_t adxl_ready(void);
int16_t adxl_readz(void);

#endif ADXL345_H