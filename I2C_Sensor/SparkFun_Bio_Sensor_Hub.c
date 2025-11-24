
#include "SparkFun_Bio_Sensor_Hub.h"
#include <string.h>

// Internal Family/Index Byte Defines
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
// Family Byte
#define DUMP_REGISTERS 0x43 

// Index Bytes
#define DUMP_REGISTER_MAX30101 0x03
#define DUMP_REGISTER_ACCELEROMETER 0x04

// Specific Internal Enums
#define SFE_BIO_AGC_GAIN_ID 0x00
#define SFE_BIO_AGC_STEP_SIZE_ID 0x01
#define SFE_BIO_AGC_SENSITIVITY_ID 0x02
#define SFE_BIO_AGC_NUM_SAMP_ID 0x03
#define SFE_BIO_MAXIMFAST_COEF_ID 0x0B

#define SFE_BIO_SET_TARG_PERC 0x00
#define SFE_BIO_SET_STEP_SIZE 0x00
#define SFE_BIO_SET_SENSITIVITY 0x00
#define SFE_BIO_SET_AVG_SAMPLES 0x00
#define SFE_BIO_SET_PULSE_OX_COEF 0x02

#define SFE_BIO_READ_AGC_PERCENTAGE 0x00
#define SFE_BIO_READ_AGC_STEP_SIZE 0x00
#define SFE_BIO_READ_AGC_SENSITIVITY 0x00
#define SFE_BIO_READ_AGC_NUM_SAMPLES 0x00
#define SFE_BIO_READ_MAX_FAST_COEF 0x02

#define SFE_BIO_READ_AGC_PERC_ID 0x00
#define SFE_BIO_READ_AGC_STEP_SIZE_ID 0x01
#define SFE_BIO_READ_AGC_SENSITIVITY_ID 0x02
#define SFE_BIO_READ_AGC_NUM_SAMPLES_ID 0x03
#define SFE_BIO_READ_MAX_FAST_COEF_ID 0x0B

#define SFE_BIO_ENABLE_AGC_ALGO 0x00
#define SFE_BIO_ENABLE_WHRM_ALGO 0x02
#define SFE_BIO_ENABLE_MAX30101 0x03
#define SFE_BIO_ENABLE_ACCELEROMETER 0x04
#define SFE_BIO_READ_ENABLE_MAX30101 0x03

#define SFE_BIO_NUM_SAMPLES 0x00
#define SFE_BIO_READ_DATA 0x00
#define SFE_BIO_SAMPLE_SIZE 0x00

#define SFE_BIO_WRITE_MAX30101 0x03
#define SFE_BIO_WRITE_ACCELEROMETER 0x04
#define SFE_BIO_READ_MAX30101 0x03
#define SFE_BIO_READ_ACCELEROMETER 0x04

#define SFE_BIO_SET_NUM_PAGES 0x02
#define SFE_BIO_ERASE_FLASH 0x03
#define SFE_BIO_BOOTLOADER_VERS 0x00
#define SFE_BIO_READ_SENSOR_HUB_VERS 0x03
#define SFE_BIO_READ_ALGO_VERS 0x07
#define SFE_BIO_READ_MCU_TYPE 0x00

#define SFE_BIO_EXIT_BOOTLOADER 0x00
#define SFE_BIO_SFE_BIO_RESET 0x02
#define SFE_BIO_ENTER_BOOTLOADER 0x08

#define SFE_BIO_SENSOR_DATA 0x01
#define SFE_BIO_ALGO_DATA 0x02
#define SFE_BIO_SENSOR_AND_ALGORITHM 0x03
#define SFE_BIO_SENSOR_ALGO_COUNTER 0x07

#define SFE_BIO_BPT_CONFIG 0x04
#define SFE_BIO_BPT_MEDICATION 0x00
#define SFE_BIO_SYSTOLIC_VALUE 0x01
#define SFE_BIO_DIASTOLIC_VALUE 0x02
#define SFE_BIO_BPT_CALIB_DATA 0x03
#define SFE_BIO_PATIENT_RESTING 0x05
#define SFE_BIO_AGC_SP02_COEFS 0x0B

// Internal Helper Functions (Forward Declarations)
static int _i2cWrite(sfe_bio_ctx_t *ctx, uint8_t *data, size_t len);
static int _i2cRead(sfe_bio_ctx_t *ctx, uint8_t *data, size_t len);
static uint8_t _enableWrite(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _enableByte);
static uint8_t _writeByte(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte);
static uint8_t _writeByteWithVal(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeVal);
static uint8_t _writeByteWithInt(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint16_t _val);
static uint8_t _writeLongBytes(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, int32_t _writeVal[], const size_t _size);
static uint8_t _writeBytes(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeVal[], size_t _size);
static uint8_t _readByte(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte);
static uint8_t _readByteWithWrite(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte);
static uint8_t _readMultipleBytesInt(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, const size_t _numOfReads, int32_t userArray[]);
static uint8_t _readMultipleBytes(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, const size_t _numOfReads, uint8_t userArray[]);
static uint8_t _readFillArray(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _numOfReads, uint8_t array[]);

static void configGpioOutput(mxc_gpio_regs_t* port, uint32_t mask) {
    mxc_gpio_cfg_t cfg;
    cfg.port = port;
    cfg.mask = mask;
    cfg.pad = MXC_GPIO_PAD_NONE;
    cfg.func = MXC_GPIO_FUNC_OUT;
    cfg.vssel = MXC_GPIO_VSSEL_VDDIO; // If this is undefined, try MXC_GPIO_VSSEL_VDDIOH or remove it if your SDK auto-selects
    MXC_GPIO_Config(&cfg);
}

static void configGpioInput(mxc_gpio_regs_t* port, uint32_t mask) {
    mxc_gpio_cfg_t cfg;
    cfg.port = port;
    cfg.mask = mask;
    cfg.pad = MXC_GPIO_PAD_PULL_UP; // Some SDKs use MXC_GPIO_PAD_PULLUP (no underscore)
    cfg.func = MXC_GPIO_FUNC_IN;
    cfg.vssel = MXC_GPIO_VSSEL_VDDIO;
    MXC_GPIO_Config(&cfg);
}

// ---------------- Public API ----------------

uint8_t SFE_Bio_Begin(sfe_bio_ctx_t *ctx, mxc_i2c_regs_t *wirePort, mxc_gpio_regs_t *resetPort, uint32_t resetPin, mxc_gpio_regs_t *mfioPort, uint32_t mfioPin)
{
    memset(ctx, 0, sizeof(sfe_bio_ctx_t));
    
    ctx->_i2cPort = wirePort;
    ctx->_address = SFE_BIO_ADDRESS;
    ctx->_sampleRate = 100;
    
    ctx->_resetPort = resetPort;
    ctx->_resetPin = resetPin;
    configGpioOutput(ctx->_resetPort, ctx->_resetPin);

    ctx->_mfioPort = mfioPort;
    ctx->_mfioPin = mfioPin;
    configGpioOutput(ctx->_mfioPort, ctx->_mfioPin);

    MXC_GPIO_OutSet(ctx->_mfioPort, ctx->_mfioPin); // High
    MXC_GPIO_OutClr(ctx->_resetPort, ctx->_resetPin); // Low
    MXC_Delay(MXC_DELAY_MSEC(10));
    MXC_GPIO_OutSet(ctx->_resetPort, ctx->_resetPin); // High
    MXC_Delay(MXC_DELAY_MSEC(1000));
    
    configGpioInput(ctx->_mfioPort, ctx->_mfioPin); 

    return _readByte(ctx, READ_DEVICE_MODE, 0x00); 
}

uint8_t SFE_Bio_BeginBootloader(sfe_bio_ctx_t *ctx, mxc_i2c_regs_t *wirePort, mxc_gpio_regs_t *resetPort, uint32_t resetPin, mxc_gpio_regs_t *mfioPort, uint32_t mfioPin)
{
    ctx->_i2cPort = wirePort;
    ctx->_address = SFE_BIO_ADDRESS;
    ctx->_resetPort = resetPort;
    ctx->_resetPin = resetPin;
    configGpioOutput(ctx->_resetPort, ctx->_resetPin);

    ctx->_mfioPort = mfioPort;
    ctx->_mfioPin = mfioPin;
    configGpioOutput(ctx->_mfioPort, ctx->_mfioPin);

    MXC_GPIO_OutClr(ctx->_mfioPort, ctx->_mfioPin); // Low
    MXC_GPIO_OutClr(ctx->_resetPort, ctx->_resetPin); // Low
    MXC_Delay(MXC_DELAY_MSEC(10));
    MXC_GPIO_OutSet(ctx->_resetPort, ctx->_resetPin); // High
    MXC_Delay(MXC_DELAY_MSEC(50)); 
    
    configGpioOutput(ctx->_resetPort, ctx->_resetPin);
    configGpioOutput(ctx->_mfioPort, ctx->_mfioPin);

    return _readByte(ctx, READ_DEVICE_MODE, 0x00); 
}

uint8_t SFE_Bio_ReadSensorHubStatus(sfe_bio_ctx_t *ctx)
{
    return _readByte(ctx, 0x00, 0x00);                         
}

uint8_t SFE_Bio_ConfigBpm(sfe_bio_ctx_t *ctx, uint8_t mode)
{
    uint8_t statusChauf = 0;
    if (mode != SFE_BIO_MODE_ONE && mode != SFE_BIO_MODE_TWO) return SFE_BIO_INCORR_PARAM;

    statusChauf = SFE_Bio_SetOutputMode(ctx, SFE_BIO_ALGO_DATA); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    statusChauf = SFE_Bio_SetFifoThreshold(ctx, 0x01); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    statusChauf = SFE_Bio_AgcAlgoControl(ctx, SFE_BIO_ENABLE); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    statusChauf = SFE_Bio_Max30101Control(ctx, SFE_BIO_ENABLE);
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    statusChauf = SFE_Bio_MaximFastAlgoControl(ctx, mode);
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    ctx->_userSelectedMode = mode;
    ctx->_sampleRate = SFE_Bio_ReadAlgoSamples(ctx);

    MXC_Delay(MXC_DELAY_MSEC(1000));
    return SFE_BIO_SUCCESS;
}

uint8_t SFE_Bio_ConfigSensor(sfe_bio_ctx_t *ctx)
{
    uint8_t statusChauf; 

    statusChauf = SFE_Bio_SetOutputMode(ctx, SFE_BIO_SENSOR_DATA); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    statusChauf = SFE_Bio_SetFifoThreshold(ctx, 0x01); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    statusChauf = SFE_Bio_Max30101Control(ctx, SFE_BIO_ENABLE); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    statusChauf = SFE_Bio_MaximFastAlgoControl(ctx, SFE_BIO_MODE_ONE); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    MXC_Delay(MXC_DELAY_MSEC(1000));
    return SFE_BIO_SUCCESS;
}

uint8_t SFE_Bio_ConfigSensorBpm(sfe_bio_ctx_t *ctx, uint8_t mode)
{
    uint8_t statusChauf; 
    if (mode != SFE_BIO_MODE_ONE && mode != SFE_BIO_MODE_TWO) return SFE_BIO_INCORR_PARAM;

    statusChauf = SFE_Bio_SetOutputMode(ctx, SFE_BIO_SENSOR_AND_ALGORITHM); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    statusChauf = SFE_Bio_SetFifoThreshold(ctx, 0x01); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    statusChauf = SFE_Bio_Max30101Control(ctx, SFE_BIO_ENABLE); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    statusChauf = SFE_Bio_MaximFastAlgoControl(ctx, mode); 
    if (statusChauf != SFE_BIO_SUCCESS) return statusChauf;

    ctx->_userSelectedMode = mode;
    ctx->_sampleRate = SFE_Bio_ReadAlgoSamples(ctx);

    MXC_Delay(MXC_DELAY_MSEC(1000));
    return SFE_BIO_SUCCESS;
}

bioData_t SFE_Bio_ReadBpm(sfe_bio_ctx_t *ctx)
{
    bioData_t libBpm = {0};

    // CORRECTION 1: Check if there is actually data to read!
    // SFE_Bio_NumSamplesOutFifo returns the number of samples available.
    uint8_t numSamples = SFE_Bio_NumSamplesOutFifo(ctx);
    if (numSamples == 0) {
        // No data available, return empty struct
        return libBpm; 
    }

    if (ctx->_userSelectedMode == SFE_BIO_MODE_ONE)
    {
        // CORRECTION 2: Read 7 Bytes (Status + 6 Data Bytes)
        // Original SFE_BIO_MAXFAST_ARRAY_SIZE is likely defined as 6. 
        // We need 7 to get the Finger Status (Byte 6).
        _readFillArray(ctx, READ_DATA_OUTPUT, SFE_BIO_READ_DATA, 7, ctx->bpmArr);

        // CORRECTION 3: Fix Indexing (Shift everything by 1)
        
        // Byte 0: General Hub Status
        libBpm.status = ctx->bpmArr[0]; 

        // Byte 1 & 2: Heart Rate
        libBpm.heartRate = ((uint16_t)ctx->bpmArr[1] << 8);
        libBpm.heartRate |= (ctx->bpmArr[2]);
        libBpm.heartRate /= 10;

        // Byte 3: Confidence
        libBpm.confidence = ctx->bpmArr[3];

        // Byte 4 & 5: SpO2
        libBpm.oxygen = ((uint16_t)ctx->bpmArr[4] << 8);
        libBpm.oxygen |= ctx->bpmArr[5];
        libBpm.oxygen /= 10;

        // Byte 6: Extended Status (Finger Status)
        // This contains the "0x03" you need to check for finger detection
        libBpm.extStatus = ctx->bpmArr[6]; 
    }
    else if (ctx->_userSelectedMode == SFE_BIO_MODE_TWO)
    {
        // Mode 2 usually adds R-Value and Extended Status differently. 
        // Assuming standard layout: Status(1) + HR(2) + Conf(1) + SpO2(2) + ExtStatus(1) + R-Val(2) ...
        // Total usually 10-12 bytes depending on version. 
        // Let's assume the SparkFun standard map for Mode 2:
        
        _readFillArray(ctx, READ_DATA_OUTPUT, SFE_BIO_READ_DATA, SFE_BIO_MAXFAST_ARRAY_SIZE + SFE_BIO_MAXFAST_EXTENDED_DATA, ctx->bpmArrTwo);

        libBpm.status = ctx->bpmArrTwo[0]; // Status first

        libBpm.heartRate = ((uint16_t)ctx->bpmArrTwo[1] << 8);
        libBpm.heartRate |= (ctx->bpmArrTwo[2]);
        libBpm.heartRate /= 10;

        libBpm.confidence = ctx->bpmArrTwo[3];

        libBpm.oxygen = ((uint16_t)ctx->bpmArrTwo[4] << 8);
        libBpm.oxygen |= ctx->bpmArrTwo[5];
        libBpm.oxygen /= 10.0;

        // In Mode 2, ExtStatus often comes immediately after SpO2 (Byte 6)
        libBpm.extStatus = ctx->bpmArrTwo[6]; 

        // R-Values then follow
        uint16_t tempVal = ((uint16_t)ctx->bpmArrTwo[7] << 8);
        tempVal |= ctx->bpmArrTwo[8];
        libBpm.rValue = tempVal;
        libBpm.rValue /= 10.0;
    }
    
    return libBpm;
}

bioData_t SFE_Bio_ReadSensor(sfe_bio_ctx_t *ctx)
{
    bioData_t libLedFifo = {0};
    _readFillArray(ctx, READ_DATA_OUTPUT, SFE_BIO_READ_DATA, SFE_BIO_MAX30101_LED_ARRAY, ctx->senArr);

    libLedFifo.irLed = ((uint32_t)ctx->senArr[0] << 16);
    libLedFifo.irLed |= ((uint32_t)ctx->senArr[1] << 8);
    libLedFifo.irLed |= ctx->senArr[2];

    libLedFifo.redLed = ((uint32_t)ctx->senArr[3] << 16);
    libLedFifo.redLed |= ((uint32_t)ctx->senArr[4] << 8);
    libLedFifo.redLed |= ctx->senArr[5];

    return libLedFifo;
}

bioData_t SFE_Bio_ReadSensorBpm(sfe_bio_ctx_t *ctx)
{
    bioData_t libLedBpm = {0};

    if (ctx->_userSelectedMode == SFE_BIO_MODE_ONE)
    {
        _readFillArray(ctx, READ_DATA_OUTPUT, SFE_BIO_READ_DATA, SFE_BIO_MAXFAST_ARRAY_SIZE + SFE_BIO_MAX30101_LED_ARRAY, ctx->bpmSenArr);

        libLedBpm.irLed = ((uint32_t)ctx->bpmSenArr[0] << 16);
        libLedBpm.irLed |= ((uint32_t)ctx->bpmSenArr[1] << 8);
        libLedBpm.irLed |= ctx->bpmSenArr[2];

        libLedBpm.redLed = ((uint32_t)ctx->bpmSenArr[3] << 16);
        libLedBpm.redLed |= ((uint32_t)ctx->bpmSenArr[4] << 8);
        libLedBpm.redLed |= ctx->bpmSenArr[5];

        libLedBpm.heartRate = ((uint16_t)ctx->bpmSenArr[12] << 8);
        libLedBpm.heartRate |= (ctx->bpmSenArr[13]);
        libLedBpm.heartRate /= 10;
        libLedBpm.confidence = ctx->bpmSenArr[14];
        libLedBpm.oxygen = ((uint16_t)ctx->bpmSenArr[15] << 8);
        libLedBpm.oxygen |= ctx->bpmSenArr[16];
        libLedBpm.oxygen /= 10;
        libLedBpm.status = ctx->bpmSenArr[17];
    }
    else if (ctx->_userSelectedMode == SFE_BIO_MODE_TWO)
    {
        _readFillArray(ctx, READ_DATA_OUTPUT, SFE_BIO_READ_DATA, SFE_BIO_MAXFAST_ARRAY_SIZE + SFE_BIO_MAX30101_LED_ARRAY + SFE_BIO_MAXFAST_EXTENDED_DATA,
                      ctx->bpmSenArrTwo);

        libLedBpm.irLed = ((uint32_t)ctx->bpmSenArrTwo[0] << 16);
        libLedBpm.irLed |= ((uint32_t)ctx->bpmSenArrTwo[1] << 8);
        libLedBpm.irLed |= ctx->bpmSenArrTwo[2];

        libLedBpm.redLed = ((uint32_t)ctx->bpmSenArrTwo[3] << 16);
        libLedBpm.redLed |= ((uint32_t)ctx->bpmSenArrTwo[4] << 8);
        libLedBpm.redLed |= ctx->bpmSenArrTwo[5];

        libLedBpm.heartRate = ((uint16_t)ctx->bpmSenArrTwo[12] << 8);
        libLedBpm.heartRate |= (ctx->bpmSenArrTwo[13]);
        libLedBpm.heartRate /= 10;
        libLedBpm.confidence = ctx->bpmSenArrTwo[14];
        libLedBpm.oxygen = ((uint16_t)ctx->bpmSenArrTwo[15] << 8);
        libLedBpm.oxygen |= ctx->bpmSenArrTwo[16];
        libLedBpm.oxygen /= 10;
        libLedBpm.status = ctx->bpmSenArrTwo[17];

        uint16_t tempVal = ((uint16_t)ctx->bpmSenArrTwo[18] << 8);
        tempVal |= ctx->bpmSenArrTwo[19];
        libLedBpm.rValue = tempVal;
        libLedBpm.rValue /= 10.0;
        libLedBpm.extStatus = ctx->bpmSenArrTwo[20];
    }
    return libLedBpm;
}

uint8_t SFE_Bio_SetPulseWidth(sfe_bio_ctx_t *ctx, uint16_t width)
{
    uint8_t bits;
    uint8_t regVal;

    if (width == 69) bits = 0;
    else if (width == 118) bits = 1;
    else if (width == 215) bits = 2;
    else if (width == 411) bits = 3;
    else return SFE_BIO_INCORR_PARAM;

    regVal = SFE_Bio_ReadRegisterMAX30101(ctx, SFE_BIO_CONFIGURATION_REGISTER);
    regVal &= SFE_BIO_PULSE_MASK;                                  
    regVal |= bits;                                        
    SFE_Bio_WriteRegisterMAX30101(ctx, SFE_BIO_CONFIGURATION_REGISTER, regVal); 

    return SFE_BIO_SUCCESS;
}

uint16_t SFE_Bio_ReadPulseWidth(sfe_bio_ctx_t *ctx)
{
    uint8_t regVal;
    regVal = SFE_Bio_ReadRegisterMAX30101(ctx, SFE_BIO_CONFIGURATION_REGISTER);
    regVal &= SFE_BIO_READ_PULSE_MASK;

    if (regVal == 0) return 69;
    else if (regVal == 1) return 118;
    else if (regVal == 2) return 215;
    else if (regVal == 3) return 411;
    else return SFE_BIO_ERR_UNKNOWN;
}

uint8_t SFE_Bio_SetSampleRate(sfe_bio_ctx_t *ctx, uint16_t sampRate)
{
    uint8_t bits;
    uint8_t regVal;

    if (sampRate == 50) bits = 0;
    else if (sampRate == 100) bits = 1;
    else if (sampRate == 200) bits = 2;
    else if (sampRate == 400) bits = 3;
    else if (sampRate == 800) bits = 4;
    else if (sampRate == 1000) bits = 5;
    else if (sampRate == 1600) bits = 6;
    else if (sampRate == 3200) bits = 7;
    else return SFE_BIO_INCORR_PARAM;

    regVal = SFE_Bio_ReadRegisterMAX30101(ctx, SFE_BIO_CONFIGURATION_REGISTER);
    regVal &= SFE_BIO_SAMP_MASK;                                   
    regVal |= (bits << 2);                                 
    SFE_Bio_WriteRegisterMAX30101(ctx, SFE_BIO_CONFIGURATION_REGISTER, regVal); 

    return SFE_BIO_SUCCESS;
}

uint16_t SFE_Bio_ReadSampleRate(sfe_bio_ctx_t *ctx)
{
    uint8_t regVal;

    regVal = SFE_Bio_ReadRegisterMAX30101(ctx, SFE_BIO_CONFIGURATION_REGISTER);
    regVal &= SFE_BIO_READ_SAMP_MASK;
    regVal = (regVal >> 2);

    if (regVal == 0) return 50;
    else if (regVal == 1) return 100;
    else if (regVal == 2) return 200;
    else if (regVal == 3) return 400;
    else if (regVal == 4) return 800;
    else if (regVal == 5) return 1000;
    else if (regVal == 6) return 1600;
    else if (regVal == 7) return 3200;
    else return SFE_BIO_ERR_UNKNOWN;
}

uint8_t SFE_Bio_SetAdcRange(sfe_bio_ctx_t *ctx, uint16_t adcVal)
{
    uint8_t regVal;
    uint8_t bits;

    if (adcVal <= 2048) bits = 0;
    else if (adcVal <= 4096) bits = 1;
    else if (adcVal <= 8192) bits = 2;
    else if (adcVal <= 16384) bits = 3;
    else return SFE_BIO_INCORR_PARAM;

    regVal = SFE_Bio_ReadRegisterMAX30101(ctx, SFE_BIO_CONFIGURATION_REGISTER);
    regVal &= SFE_BIO_ADC_MASK;
    regVal |= bits << 5;

    SFE_Bio_WriteRegisterMAX30101(ctx, SFE_BIO_CONFIGURATION_REGISTER, regVal);
    return SFE_BIO_SUCCESS;
}

uint16_t SFE_Bio_ReadAdcRange(sfe_bio_ctx_t *ctx)
{
    uint8_t regVal;
    regVal = SFE_Bio_ReadRegisterMAX30101(ctx, SFE_BIO_CONFIGURATION_REGISTER);
    regVal &= SFE_BIO_READ_ADC_MASK;
    regVal = (regVal >> 5); 

    if (regVal == 0) return 2048;
    else if (regVal == 1) return 4096;
    else if (regVal == 2) return 8192;
    else if (regVal == 3) return 16384;
    else return SFE_BIO_ERR_UNKNOWN;
}

uint8_t SFE_Bio_SetOperatingMode(sfe_bio_ctx_t *ctx, uint8_t selection)
{
    if (selection != SFE_BIO_EXIT_BOOTLOADER && selection != SFE_BIO_SFE_BIO_RESET && selection != SFE_BIO_ENTER_BOOTLOADER) 
        return SFE_BIO_INCORR_PARAM;

    uint8_t statusByte = _writeByte(ctx, SET_DEVICE_MODE, 0x00, selection);

    if (statusByte != SFE_BIO_SUCCESS) return statusByte;
    return _readByte(ctx, READ_DEVICE_MODE, 0x00); 
}

uint8_t SFE_Bio_GetMcuType(sfe_bio_ctx_t *ctx)
{
    uint8_t returnByte = _readByteWithWrite(ctx, IDENTITY, SFE_BIO_READ_MCU_TYPE, SFE_BIO_NO_WRITE);
    if (returnByte != SFE_BIO_SUCCESS) return SFE_BIO_ERR_UNKNOWN;
    else return returnByte;
}

int32_t SFE_Bio_GetBootloaderInf(sfe_bio_ctx_t *ctx)
{
    int32_t bootVers = 0;
    int32_t revNum[4] = {0};
    uint8_t status = _readMultipleBytesInt(ctx, BOOTLOADER_INFO, SFE_BIO_BOOTLOADER_VERS, 0x00, 4, revNum);

    if (!status) return SFE_BIO_ERR_UNKNOWN;
    
    bootVers |= ((int32_t)revNum[1] << 16);
    bootVers |= ((int32_t)revNum[2] << 8);
    bootVers |= revNum[3];
    return bootVers;
}

uint8_t SFE_Bio_Max30101Control(sfe_bio_ctx_t *ctx, uint8_t senSwitch)
{
    if (senSwitch != 0 && senSwitch != 1) return SFE_BIO_INCORR_PARAM;
    return _enableWrite(ctx, ENABLE_SENSOR, SFE_BIO_ENABLE_MAX30101, senSwitch);
}

uint8_t SFE_Bio_ReadMAX30101State(sfe_bio_ctx_t *ctx)
{
    return _readByte(ctx, READ_SENSOR_MODE, SFE_BIO_READ_ENABLE_MAX30101);
}

uint8_t SFE_Bio_AccelControl(sfe_bio_ctx_t *ctx, uint8_t accelSwitch)
{
    if (accelSwitch != 0 && accelSwitch != 1) return SFE_BIO_INCORR_PARAM;
    return _enableWrite(ctx, ENABLE_SENSOR, SFE_BIO_ENABLE_ACCELEROMETER, accelSwitch);
}

uint8_t SFE_Bio_SetOutputMode(sfe_bio_ctx_t *ctx, uint8_t outputType)
{
    if (outputType > SFE_BIO_SENSOR_ALGO_COUNTER) return SFE_BIO_INCORR_PARAM;
    return _writeByte(ctx, OUTPUT_MODE, SFE_BIO_SET_FORMAT, outputType);
}

uint8_t SFE_Bio_SetFifoThreshold(sfe_bio_ctx_t *ctx, uint8_t intThresh)
{
    return _writeByte(ctx, OUTPUT_MODE, SFE_BIO_WRITE_SET_THRESHOLD, intThresh);
}

uint8_t SFE_Bio_NumSamplesOutFifo(sfe_bio_ctx_t *ctx)
{
    return _readByte(ctx, READ_DATA_OUTPUT, SFE_BIO_NUM_SAMPLES);
}

uint8_t *SFE_Bio_GetDataOutFifo(sfe_bio_ctx_t *ctx, uint8_t data[])
{
    uint8_t samples = SFE_Bio_NumSamplesOutFifo(ctx);
    _readFillArray(ctx, READ_DATA_OUTPUT, SFE_BIO_READ_DATA, samples, data);
    return data;
}

uint8_t SFE_Bio_NumSamplesExternalSensor(sfe_bio_ctx_t *ctx)
{
    return _readByteWithWrite(ctx, READ_DATA_INPUT, SFE_BIO_SAMPLE_SIZE, SFE_BIO_WRITE_ACCELEROMETER);
}

void SFE_Bio_WriteRegisterMAX30101(sfe_bio_ctx_t *ctx, uint8_t regAddr, uint8_t regVal)
{
    _writeByteWithVal(ctx, WRITE_REGISTER, SFE_BIO_WRITE_MAX30101, regAddr, regVal);
}

void SFE_Bio_WriteRegisterAccel(sfe_bio_ctx_t *ctx, uint8_t regAddr, uint8_t regVal)
{
    _writeByteWithVal(ctx, WRITE_REGISTER, SFE_BIO_WRITE_ACCELEROMETER, regAddr, regVal);
}

uint8_t SFE_Bio_ReadRegisterMAX30101(sfe_bio_ctx_t *ctx, uint8_t regAddr)
{
    return _readByteWithWrite(ctx, READ_REGISTER, SFE_BIO_READ_MAX30101, regAddr);
}

uint8_t SFE_Bio_ReadRegisterAccel(sfe_bio_ctx_t *ctx, uint8_t regAddr)
{
    return _readByteWithWrite(ctx, READ_REGISTER, SFE_BIO_READ_ACCELEROMETER, regAddr);
}

uint8_t SFE_Bio_DumpRegisterMAX30101(sfe_bio_ctx_t *ctx, uint8_t regArray[])
{
    return _readFillArray(ctx, DUMP_REGISTERS, DUMP_REGISTER_MAX30101, 36, regArray);
}

uint8_t SFE_Bio_DumpRegisterAccelerometer(sfe_bio_ctx_t *ctx, uint8_t numReg, uint8_t regArray[])
{
    return _readFillArray(ctx, DUMP_REGISTERS, DUMP_REGISTER_ACCELEROMETER, numReg, regArray); 
}

uint8_t SFE_Bio_SetAlgoRange(sfe_bio_ctx_t *ctx, uint8_t perc)
{
    if (perc > 100) return SFE_BIO_INCORR_PARAM;
    return _writeByteWithVal(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_SET_TARG_PERC, SFE_BIO_AGC_GAIN_ID, perc);
}

uint8_t SFE_Bio_SetAlgoStepSize(sfe_bio_ctx_t *ctx, uint8_t step)
{
    if (step > 100) return SFE_BIO_INCORR_PARAM;
    return _writeByteWithVal(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_SET_STEP_SIZE, SFE_BIO_AGC_STEP_SIZE_ID, step);
}

uint8_t SFE_Bio_SetAlgoSensitivity(sfe_bio_ctx_t *ctx, uint8_t sense)
{
    if (sense > 100) return SFE_BIO_INCORR_PARAM;
    return _writeByteWithVal(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_SET_SENSITIVITY, SFE_BIO_AGC_SENSITIVITY_ID, sense);
}

uint8_t SFE_Bio_SetAlgoSamples(sfe_bio_ctx_t *ctx, uint8_t avg)
{
    return _writeByteWithVal(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_SET_AVG_SAMPLES, SFE_BIO_AGC_NUM_SAMP_ID, avg);
}

uint8_t SFE_Bio_SetMaximFastCoef(sfe_bio_ctx_t *ctx, int32_t coef1, int32_t coef2, int32_t coef3)
{
    int32_t coefArr[3] = {coef1, coef2, coef3};
    return _writeLongBytes(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_SET_PULSE_OX_COEF, SFE_BIO_MAXIMFAST_COEF_ID, coefArr, 3);
}

uint8_t SFE_Bio_ReadAlgoRange(sfe_bio_ctx_t *ctx)
{
    return _readByteWithWrite(ctx, READ_ALGORITHM_CONFIG, SFE_BIO_READ_AGC_PERCENTAGE, SFE_BIO_READ_AGC_PERC_ID);
}

uint8_t SFE_Bio_ReadAlgoStepSize(sfe_bio_ctx_t *ctx)
{
    return _readByteWithWrite(ctx, READ_ALGORITHM_CONFIG, SFE_BIO_READ_AGC_STEP_SIZE, SFE_BIO_READ_AGC_STEP_SIZE_ID);
}

uint8_t SFE_Bio_ReadAlgoSensitivity(sfe_bio_ctx_t *ctx)
{
    return _readByteWithWrite(ctx, READ_ALGORITHM_CONFIG, SFE_BIO_READ_AGC_SENSITIVITY, SFE_BIO_READ_AGC_SENSITIVITY_ID);
}

uint8_t SFE_Bio_ReadAlgoSamples(sfe_bio_ctx_t *ctx)
{
    return _readByteWithWrite(ctx, READ_ALGORITHM_CONFIG, SFE_BIO_READ_AGC_NUM_SAMPLES, SFE_BIO_READ_AGC_NUM_SAMPLES_ID);
}

uint8_t SFE_Bio_ReadMaximFastCoef(sfe_bio_ctx_t *ctx, int32_t coefArr[3])
{
    uint8_t status = _readMultipleBytesInt(ctx, READ_ALGORITHM_CONFIG, SFE_BIO_READ_MAX_FAST_COEF, SFE_BIO_READ_MAX_FAST_COEF_ID, 3, coefArr);
    coefArr[0] *= 100000;
    coefArr[1] *= 100000;
    coefArr[2] *= 100000;
    return status;
}

uint8_t SFE_Bio_AgcAlgoControl(sfe_bio_ctx_t *ctx, uint8_t enable)
{
    if (enable != 0 && enable != 1) return SFE_BIO_INCORR_PARAM;
    return _enableWrite(ctx, ENABLE_ALGORITHM, SFE_BIO_ENABLE_AGC_ALGO, enable);
}

uint8_t SFE_Bio_MaximFastAlgoControl(sfe_bio_ctx_t *ctx, uint8_t mode)
{
    if (mode != 0 && mode != 1 && mode != 2) return SFE_BIO_INCORR_PARAM;
    return _enableWrite(ctx, ENABLE_ALGORITHM, SFE_BIO_ENABLE_WHRM_ALGO, mode);
}

bool SFE_Bio_SetNumPages(sfe_bio_ctx_t *ctx, uint8_t totalPages)
{
    return _writeByteWithVal(ctx, BOOTLOADER_FLASH, SFE_BIO_SET_NUM_PAGES, 0x00, totalPages);
}

bool SFE_Bio_EraseFlash(sfe_bio_ctx_t *ctx)
{
    uint8_t cmd[2] = {BOOTLOADER_FLASH, SFE_BIO_ERASE_FLASH};
    _i2cWrite(ctx, cmd, 2);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t statusByte;
    _i2cRead(ctx, &statusByte, 1);
    
    return (statusByte == 0);
}

version_t SFE_Bio_ReadBootloaderVers(sfe_bio_ctx_t *ctx)
{
    version_t booVers = {0}; 
    uint8_t cmd[2] = {BOOTLOADER_INFO, SFE_BIO_BOOTLOADER_VERS};
    _i2cWrite(ctx, cmd, 2);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t data[4];
    _i2cRead(ctx, data, 4);
    
    if (data[0]) return booVers; // Error

    booVers.major = data[1];
    booVers.minor = data[2];
    booVers.revision = data[3];
    return booVers;
}

version_t SFE_Bio_ReadSensorHubVersion(sfe_bio_ctx_t *ctx)
{
    version_t bioHubVers = {0};
    uint8_t cmd[2] = {IDENTITY, SFE_BIO_READ_SENSOR_HUB_VERS};
    _i2cWrite(ctx, cmd, 2);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t data[4];
    _i2cRead(ctx, data, 4);

    if (data[0]) return bioHubVers; // Error

    bioHubVers.major = data[1];
    bioHubVers.minor = data[2];
    bioHubVers.revision = data[3];
    return bioHubVers;
}

version_t SFE_Bio_ReadAlgorithmVersion(sfe_bio_ctx_t *ctx)
{
    version_t libAlgoVers = {0};
    uint8_t cmd[2] = {IDENTITY, SFE_BIO_READ_ALGO_VERS};
    _i2cWrite(ctx, cmd, 2);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t data[4];
    _i2cRead(ctx, data, 4);

    if (data[0]) return libAlgoVers; 

    libAlgoVers.major = data[1];
    libAlgoVers.minor = data[2];
    libAlgoVers.revision = data[3];
    return libAlgoVers;
}

uint8_t SFE_Bio_IsPatientBPMedication(sfe_bio_ctx_t *ctx, uint8_t medication)
{
    if (medication != 0x01 && medication != 0x00) return SFE_BIO_INCORR_PARAM;
    return _writeByteWithVal(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_BPT_MEDICATION, medication);
}

uint8_t SFE_Bio_ReadPatientBPMedication(sfe_bio_ctx_t *ctx)
{
    return _readByteWithWrite(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_BPT_MEDICATION);
}

uint8_t SFE_Bio_WriteSystolicVals(sfe_bio_ctx_t *ctx, uint8_t sysVal1, uint8_t sysVal2, uint8_t sysVal3)
{
    uint8_t sysVals[3] = {sysVal1, sysVal2, sysVal3};
    return _writeBytes(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_SYSTOLIC_VALUE, sysVals, 3);
}

uint8_t SFE_Bio_ReadSystolicVals(sfe_bio_ctx_t *ctx, uint8_t userArray[])
{
    return _readMultipleBytes(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_SYSTOLIC_VALUE, 3, userArray);
}

uint8_t SFE_Bio_WriteDiastolicVals(sfe_bio_ctx_t *ctx, uint8_t diasVal1, uint8_t diasVal2, uint8_t diasVal3)
{
    uint8_t diasVals[3] = {diasVal1, diasVal2, diasVal3};
    return _writeBytes(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_DIASTOLIC_VALUE, diasVals, 3);
}

uint8_t SFE_Bio_ReadDiastolicVals(sfe_bio_ctx_t *ctx, uint8_t userArray[])
{
    return _readMultipleBytes(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_DIASTOLIC_VALUE, 3, userArray);
}

uint8_t SFE_Bio_WriteBPTAlgoData(sfe_bio_ctx_t *ctx, uint8_t bptCalibData[])
{
    return _writeBytes(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_BPT_CALIB_DATA, bptCalibData, 824);
}

uint8_t SFE_Bio_ReadBPTAlgoData(sfe_bio_ctx_t *ctx, uint8_t userArray[])
{
    return _readMultipleBytes(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_BPT_CALIB_DATA, 824, userArray);
}

uint8_t SFE_Bio_IsPatientResting(sfe_bio_ctx_t *ctx, uint8_t resting)
{ 
    if (resting != 0x00 && resting != 0x01) return SFE_BIO_INCORR_PARAM;
    return _writeByteWithVal(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_PATIENT_RESTING, resting);
}

uint8_t SFE_Bio_ReadPatientResting(sfe_bio_ctx_t *ctx)
{
    return _readByteWithWrite(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_PATIENT_RESTING);
}

uint8_t SFE_Bio_WriteSP02AlgoCoef(sfe_bio_ctx_t *ctx, int32_t intA, int32_t intB, int32_t intC)
{
    int32_t coefVals[3] = {intA, intB, intC};
    return _writeLongBytes(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_AGC_SP02_COEFS, coefVals, 3);
}

uint8_t SFE_Bio_ReadSP02AlgoCoef(sfe_bio_ctx_t *ctx, int32_t userArray[])
{ 
    return _readMultipleBytesInt(ctx, CHANGE_ALGORITHM_CONFIG, SFE_BIO_BPT_CONFIG, SFE_BIO_AGC_SP02_COEFS, 3, userArray);
}

// ---------------- Private / Static Helpers ----------------

static int _i2cWrite(sfe_bio_ctx_t *ctx, uint8_t *data, size_t len)
{
    mxc_i2c_req_t req;
    req.i2c = ctx->_i2cPort;
    req.addr = ctx->_address;
    req.tx_buf = data;
    req.tx_len = len;
    req.rx_buf = NULL;
    req.rx_len = 0;
    req.restart = 0;
    req.callback = NULL;

    return MXC_I2C_MasterTransaction(&req);
}

static int _i2cRead(sfe_bio_ctx_t *ctx, uint8_t *data, size_t len)
{
    mxc_i2c_req_t req;
    req.i2c = ctx->_i2cPort;
    req.addr = ctx->_address;
    req.tx_buf = NULL;
    req.tx_len = 0;
    req.rx_buf = data;
    req.rx_len = len;
    req.restart = 0;
    req.callback = NULL;

    return MXC_I2C_MasterTransaction(&req);
}

static uint8_t _enableWrite(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _enableByte)
{
    uint8_t cmd[3] = {_familyByte, _indexByte, _enableByte};
    _i2cWrite(ctx, cmd, 3);

    if (_familyByte == ENABLE_SENSOR && _indexByte == SFE_BIO_ENABLE_MAX30101)
        MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_ENABLE_CMD_DELAY));
    if (_familyByte == ENABLE_ALGORITHM && _indexByte == SFE_BIO_ENABLE_AGC_ALGO)
        MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_ALGO_CMD_DELAY_SHORT));
    if (_familyByte == ENABLE_ALGORITHM && _indexByte == SFE_BIO_ENABLE_WHRM_ALGO)
        MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_ALGO_CMD_DELAY_LONG));

    uint8_t statusByte;
    _i2cRead(ctx, &statusByte, 1);
    return statusByte;
}

static uint8_t _writeByte(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte)
{
    uint8_t cmd[3] = {_familyByte, _indexByte, _writeByte};
    _i2cWrite(ctx, cmd, 3);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t statusByte;
    _i2cRead(ctx, &statusByte, 1);
    return statusByte;
}

static uint8_t _writeByteWithInt(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint16_t _val)
{
    uint8_t cmd[5];
    cmd[0] = _familyByte;
    cmd[1] = _indexByte;
    cmd[2] = _writeByte;
    cmd[3] = (_val >> 8); 
    cmd[4] = _val;        

    _i2cWrite(ctx, cmd, 5);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t statusByte;
    _i2cRead(ctx, &statusByte, 1);
    return statusByte;
}

static uint8_t _writeByteWithVal(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeVal)
{
    uint8_t cmd[4] = {_familyByte, _indexByte, _writeByte, _writeVal};
    _i2cWrite(ctx, cmd, 4);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t statusByte;
    _i2cRead(ctx, &statusByte, 1);
    return statusByte;
}

static uint8_t _writeLongBytes(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, int32_t _writeVal[], const size_t _size)
{
    uint8_t *buffer = (uint8_t*)malloc(3 + (_size * 4));
    if(buffer == NULL) return SFE_BIO_ERR_UNKNOWN; 

    buffer[0] = _familyByte;
    buffer[1] = _indexByte;
    buffer[2] = _writeByte;

    for (size_t i = 0; i < _size; i++)
    {
        buffer[3 + (i*4) + 0] = (_writeVal[i] >> 24);
        buffer[3 + (i*4) + 1] = (_writeVal[i] >> 16);
        buffer[3 + (i*4) + 2] = (_writeVal[i] >> 8);
        buffer[3 + (i*4) + 3] = _writeVal[i];
    }

    _i2cWrite(ctx, buffer, 3 + (_size * 4));
    free(buffer);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t statusByte;
    _i2cRead(ctx, &statusByte, 1);
    return statusByte;
}

static uint8_t _writeBytes(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, uint8_t _writeVal[], size_t _size)
{
    uint8_t *buffer = (uint8_t*)malloc(3 + _size);
    if(buffer == NULL) return SFE_BIO_ERR_UNKNOWN; 

    buffer[0] = _familyByte;
    buffer[1] = _indexByte;
    buffer[2] = _writeByte;

    for (size_t i = 0; i < _size; i++)
    {
        buffer[3+i] = _writeVal[i];
    }

    _i2cWrite(ctx, buffer, 3 + _size);
    free(buffer);

    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t statusByte;
    _i2cRead(ctx, &statusByte, 1);
    return statusByte;
}

static uint8_t _readByte(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte)
{
    uint8_t statusByte;
    uint8_t cmd[2] = {_familyByte, _indexByte};

    _i2cWrite(ctx, cmd, 2);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t rxBuffer[2]; 
    _i2cRead(ctx, rxBuffer, 2);

    statusByte = rxBuffer[0];
    if (statusByte) return statusByte; 

    return rxBuffer[1];
}

static uint8_t _readByteWithWrite(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte)
{
    uint8_t statusByte;
    uint8_t cmd[3] = {_familyByte, _indexByte, _writeByte};

    _i2cWrite(ctx, cmd, 3);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t rxBuffer[2]; 
    _i2cRead(ctx, rxBuffer, 2);

    statusByte = rxBuffer[0];
    if (statusByte) return statusByte; 

    return rxBuffer[1];
}

static uint8_t _readFillArray(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _numOfReads, uint8_t array[])
{
    uint8_t statusByte;
    uint8_t cmd[2] = {_familyByte, _indexByte};

    _i2cWrite(ctx, cmd, 2);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t *rxBuf = (uint8_t*)malloc(_numOfReads + 1);
    if (!rxBuf) return SFE_BIO_ERR_UNKNOWN;

    _i2cRead(ctx, rxBuf, _numOfReads + 1);

    statusByte = rxBuf[0];
    if (statusByte)
    { 
        memset(array, 0, _numOfReads);
        free(rxBuf);
        return statusByte;
    }

    for (size_t i = 0; i < _numOfReads; i++)
    {
        array[i] = rxBuf[i+1];
    }
    free(rxBuf);
    return statusByte;
}

static uint8_t _readMultipleBytesInt(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, const size_t _numOfReads, int32_t userArray[])
{
    uint8_t statusByte;
    uint8_t cmd[3] = {_familyByte, _indexByte, _writeByte};

    _i2cWrite(ctx, cmd, 3);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    size_t totalBytes = (sizeof(int32_t) * _numOfReads) + 1; 
    uint8_t *rxBuf = (uint8_t*)malloc(totalBytes);
    if(!rxBuf) return SFE_BIO_ERR_UNKNOWN;

    _i2cRead(ctx, rxBuf, totalBytes);
    statusByte = rxBuf[0];

    if (statusByte) 
    {
        free(rxBuf);
        return statusByte;
    }
    else
    {
        for (size_t i = 0; i < _numOfReads; i++)
        {
            size_t offset = 1 + (i * 4);
            userArray[i] = (int32_t)rxBuf[offset] << 24;
            userArray[i] |= (int32_t)rxBuf[offset+1] << 16;
            userArray[i] |= (int32_t)rxBuf[offset+2] << 8;
            userArray[i] |= (int32_t)rxBuf[offset+3];
        }
        free(rxBuf);
        return statusByte;
    }
}

static uint8_t _readMultipleBytes(sfe_bio_ctx_t *ctx, uint8_t _familyByte, uint8_t _indexByte, uint8_t _writeByte, const size_t _numOfReads, uint8_t userArray[])
{
    uint8_t statusByte;
    uint8_t cmd[3] = {_familyByte, _indexByte, _writeByte};

    _i2cWrite(ctx, cmd, 3);
    MXC_Delay(MXC_DELAY_MSEC(SFE_BIO_CMD_DELAY));

    uint8_t *rxBuf = (uint8_t*)malloc(_numOfReads + 1);
    if(!rxBuf) return SFE_BIO_ERR_UNKNOWN;

    _i2cRead(ctx, rxBuf, _numOfReads + 1);
    statusByte = rxBuf[0];

    if (statusByte) 
    {
        free(rxBuf);
        return statusByte;
    }
    else
    {
        for (size_t i = 0; i < _numOfReads; i++)
        {
            userArray[i] = rxBuf[i+1];
        }
        free(rxBuf);
        return statusByte;
    }
}