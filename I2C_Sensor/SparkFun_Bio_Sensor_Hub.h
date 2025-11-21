#ifndef _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H_
#define _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h> 

// Maxim SDK Includes
#include "mxc_device.h"
#include "i2c.h"
#include "gpio.h"
#include "mxc_delay.h"

// Defines
#define SFE_BIO_WRITE_FIFO_INPUT_BYTE 0x04
#define SFE_BIO_DISABLE 0x00
#define SFE_BIO_ENABLE 0x01
#define SFE_BIO_MODE_ONE 0x01
#define SFE_BIO_MODE_TWO 0x02
#define SFE_BIO_APP_MODE 0x00
#define SFE_BIO_BOOTLOADER_MODE 0x08
#define SFE_BIO_NO_WRITE 0x00
#define SFE_BIO_INCORR_PARAM 0xEE

#define SFE_BIO_CONFIGURATION_REGISTER 0x0A
#define SFE_BIO_PULSE_MASK 0xFC
#define SFE_BIO_READ_PULSE_MASK 0x03
#define SFE_BIO_SAMP_MASK 0xE3
#define SFE_BIO_READ_SAMP_MASK 0x1C
#define SFE_BIO_ADC_MASK 0x9F
#define SFE_BIO_READ_ADC_MASK 0x60

#define SFE_BIO_ENABLE_CMD_DELAY 45     
#define SFE_BIO_ALGO_CMD_DELAY_SHORT 45 
#define SFE_BIO_ALGO_CMD_DELAY_LONG 45  
#define SFE_BIO_CMD_DELAY 2             
#define SFE_BIO_MAXFAST_ARRAY_SIZE 6    
#define SFE_BIO_MAXFAST_EXTENDED_DATA 5
#define SFE_BIO_MAX30101_LED_ARRAY 12 

#define SFE_BIO_SET_FORMAT 0x00
#define SFE_BIO_READ_FORMAT 0x01         
#define SFE_BIO_WRITE_SET_THRESHOLD 0x01 
#define SFE_BIO_WRITE_EXTERNAL_TO_FIFO 0x00

#define SFE_BIO_ADDRESS 0x55

// Data Structs
typedef struct
{
    uint32_t irLed;
    uint32_t redLed;
    uint16_t heartRate;  
    uint8_t confidence;  
    uint16_t oxygen;     
    uint8_t status;      
    float rValue;        
    int8_t extStatus;    
    uint8_t reserveOne;  
    uint8_t resserveTwo; 
} bioData_t;

typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
} version_t;

typedef struct
{
    uint8_t byteWord;
    uint8_t availRegisters;
} sensorAttr_t;

// Enums
typedef enum
{
    SFE_BIO_SUCCESS = 0x00,
    SFE_BIO_ERR_UNAVAIL_CMD,
    SFE_BIO_ERR_UNAVAIL_FUNC,
    SFE_BIO_ERR_DATA_FORMAT,
    SFE_BIO_ERR_INPUT_VALUE,
    SFE_BIO_ERR_TRY_AGAIN,
    SFE_BIO_ERR_BTLDR_GENERAL = 0x80,
    SFE_BIO_ERR_BTLDR_CHECKSUM,
    SFE_BIO_ERR_BTLDR_AUTH,
    SFE_BIO_ERR_BTLDR_INVALID_APP,
    SFE_BIO_ERR_UNKNOWN = 0xFF
} sfe_bio_status_t;

// THE CONTEXT STRUCT (Replaces the Class)
typedef struct
{
    // Maxim Hardware Handles
    mxc_i2c_regs_t *_i2cPort;
    mxc_gpio_regs_t *_resetPort;
    uint32_t _resetPin;
    mxc_gpio_regs_t *_mfioPort;
    uint32_t _mfioPin;
    uint8_t _address;

    // State Variables
    uint8_t _userSelectedMode;
    uint8_t _sampleRate;
    
    // Buffers (formerly class members)
    uint8_t bpmArr[SFE_BIO_MAXFAST_ARRAY_SIZE];
    uint8_t bpmArrTwo[SFE_BIO_MAXFAST_ARRAY_SIZE + SFE_BIO_MAXFAST_EXTENDED_DATA];
    uint8_t senArr[SFE_BIO_MAX30101_LED_ARRAY];
    uint8_t bpmSenArr[SFE_BIO_MAXFAST_ARRAY_SIZE + SFE_BIO_MAX30101_LED_ARRAY];
    uint8_t bpmSenArrTwo[SFE_BIO_MAXFAST_ARRAY_SIZE + SFE_BIO_MAXFAST_EXTENDED_DATA + SFE_BIO_MAX30101_LED_ARRAY];
} sfe_bio_ctx_t;

// -------------------------------------------------------------------------
// Function Prototypes
// -------------------------------------------------------------------------

// Initialization
uint8_t SFE_Bio_Begin(sfe_bio_ctx_t *ctx, mxc_i2c_regs_t *wirePort, 
                      mxc_gpio_regs_t *resetPort, uint32_t resetPin, 
                      mxc_gpio_regs_t *mfioPort, uint32_t mfioPin);

uint8_t SFE_Bio_BeginBootloader(sfe_bio_ctx_t *ctx, mxc_i2c_regs_t *wirePort, 
                                mxc_gpio_regs_t *resetPort, uint32_t resetPin, 
                                mxc_gpio_regs_t *mfioPort, uint32_t mfioPin);

// Core Functions
uint8_t SFE_Bio_ReadSensorHubStatus(sfe_bio_ctx_t *ctx);
uint8_t SFE_Bio_SetOperatingMode(sfe_bio_ctx_t *ctx, uint8_t selection);
uint8_t SFE_Bio_ConfigBpm(sfe_bio_ctx_t *ctx, uint8_t mode);
uint8_t SFE_Bio_ConfigSensor(sfe_bio_ctx_t *ctx);
uint8_t SFE_Bio_ConfigSensorBpm(sfe_bio_ctx_t *ctx, uint8_t mode);

// Data Retrieval
bioData_t SFE_Bio_ReadBpm(sfe_bio_ctx_t *ctx);
bioData_t SFE_Bio_ReadSensor(sfe_bio_ctx_t *ctx);
bioData_t SFE_Bio_ReadSensorBpm(sfe_bio_ctx_t *ctx);

// Configuration
uint8_t SFE_Bio_SetPulseWidth(sfe_bio_ctx_t *ctx, uint16_t width);
uint16_t SFE_Bio_ReadPulseWidth(sfe_bio_ctx_t *ctx);
uint8_t SFE_Bio_SetSampleRate(sfe_bio_ctx_t *ctx, uint16_t sampRate);
uint16_t SFE_Bio_ReadSampleRate(sfe_bio_ctx_t *ctx);
uint8_t SFE_Bio_SetAdcRange(sfe_bio_ctx_t *ctx, uint16_t adcVal);
uint16_t SFE_Bio_ReadAdcRange(sfe_bio_ctx_t *ctx);

// Metadata
uint8_t SFE_Bio_GetMcuType(sfe_bio_ctx_t *ctx);
int32_t SFE_Bio_GetBootloaderInf(sfe_bio_ctx_t *ctx);
version_t SFE_Bio_ReadBootloaderVers(sfe_bio_ctx_t *ctx);
version_t SFE_Bio_ReadSensorHubVersion(sfe_bio_ctx_t *ctx);
version_t SFE_Bio_ReadAlgorithmVersion(sfe_bio_ctx_t *ctx);

// Device Control
uint8_t SFE_Bio_Max30101Control(sfe_bio_ctx_t *ctx, uint8_t senSwitch);
uint8_t SFE_Bio_ReadMAX30101State(sfe_bio_ctx_t *ctx);
uint8_t SFE_Bio_AccelControl(sfe_bio_ctx_t *ctx, uint8_t accelSwitch);
uint8_t SFE_Bio_SetOutputMode(sfe_bio_ctx_t *ctx, uint8_t outputType);
uint8_t SFE_Bio_SetFifoThreshold(sfe_bio_ctx_t *ctx, uint8_t intThresh);

// FIFO
uint8_t SFE_Bio_NumSamplesOutFifo(sfe_bio_ctx_t *ctx);
uint8_t *SFE_Bio_GetDataOutFifo(sfe_bio_ctx_t *ctx, uint8_t data[]);
uint8_t SFE_Bio_NumSamplesExternalSensor(sfe_bio_ctx_t *ctx);

// Registers
void SFE_Bio_WriteRegisterMAX30101(sfe_bio_ctx_t *ctx, uint8_t regAddr, uint8_t regVal);
void SFE_Bio_WriteRegisterAccel(sfe_bio_ctx_t *ctx, uint8_t regAddr, uint8_t regVal);
uint8_t SFE_Bio_ReadRegisterMAX30101(sfe_bio_ctx_t *ctx, uint8_t regAddr);
uint8_t SFE_Bio_ReadRegisterAccel(sfe_bio_ctx_t *ctx, uint8_t regAddr);
uint8_t SFE_Bio_DumpRegisterMAX30101(sfe_bio_ctx_t *ctx, uint8_t regArray[]);
uint8_t SFE_Bio_DumpRegisterAccelerometer(sfe_bio_ctx_t *ctx, uint8_t numReg, uint8_t regArray[]);

// Algorithm Config
uint8_t SFE_Bio_SetAlgoRange(sfe_bio_ctx_t *ctx, uint8_t perc);
uint8_t SFE_Bio_SetAlgoStepSize(sfe_bio_ctx_t *ctx, uint8_t step);
uint8_t SFE_Bio_SetAlgoSensitivity(sfe_bio_ctx_t *ctx, uint8_t sense);
uint8_t SFE_Bio_SetAlgoSamples(sfe_bio_ctx_t *ctx, uint8_t avg);
uint8_t SFE_Bio_SetMaximFastCoef(sfe_bio_ctx_t *ctx, int32_t coef1, int32_t coef2, int32_t coef3);
uint8_t SFE_Bio_ReadAlgoRange(sfe_bio_ctx_t *ctx);
uint8_t SFE_Bio_ReadAlgoStepSize(sfe_bio_ctx_t *ctx);
uint8_t SFE_Bio_ReadAlgoSensitivity(sfe_bio_ctx_t *ctx);
uint8_t SFE_Bio_ReadAlgoSamples(sfe_bio_ctx_t *ctx);
uint8_t SFE_Bio_ReadMaximFastCoef(sfe_bio_ctx_t *ctx, int32_t coefArr[3]);

uint8_t SFE_Bio_AgcAlgoControl(sfe_bio_ctx_t *ctx, uint8_t enable);
uint8_t SFE_Bio_MaximFastAlgoControl(sfe_bio_ctx_t *ctx, uint8_t mode);

// Bootloader / Flash
bool SFE_Bio_SetNumPages(sfe_bio_ctx_t *ctx, uint8_t totalPages);
bool SFE_Bio_EraseFlash(sfe_bio_ctx_t *ctx);

// BPT (Blood Pressure Trend)
uint8_t SFE_Bio_IsPatientBPMedication(sfe_bio_ctx_t *ctx, uint8_t medication);
uint8_t SFE_Bio_ReadPatientBPMedication(sfe_bio_ctx_t *ctx); 
uint8_t SFE_Bio_WriteSystolicVals(sfe_bio_ctx_t *ctx, uint8_t sysVal1, uint8_t sysVal2, uint8_t sysVal3);
uint8_t SFE_Bio_ReadSystolicVals(sfe_bio_ctx_t *ctx, uint8_t userArray[]);
uint8_t SFE_Bio_WriteDiastolicVals(sfe_bio_ctx_t *ctx, uint8_t diasVal1, uint8_t diasVal2, uint8_t diasVal3);
uint8_t SFE_Bio_ReadDiastolicVals(sfe_bio_ctx_t *ctx, uint8_t userArray[]);
uint8_t SFE_Bio_WriteBPTAlgoData(sfe_bio_ctx_t *ctx, uint8_t bptCalibData[]);
uint8_t SFE_Bio_ReadBPTAlgoData(sfe_bio_ctx_t *ctx, uint8_t userArray[]);
uint8_t SFE_Bio_IsPatientResting(sfe_bio_ctx_t *ctx, uint8_t resting);
uint8_t SFE_Bio_ReadPatientResting(sfe_bio_ctx_t *ctx); 
uint8_t SFE_Bio_WriteSP02AlgoCoef(sfe_bio_ctx_t *ctx, int32_t intA, int32_t intB, int32_t intC);
uint8_t SFE_Bio_ReadSP02AlgoCoef(sfe_bio_ctx_t *ctx, int32_t userArray[]);

#ifdef __cplusplus
}
#endif

#endif