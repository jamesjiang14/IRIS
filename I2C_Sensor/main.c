/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "i2c.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "board.h"
#include "mxc_errors.h"
#include "SparkFun_Bio_Sensor_Hub.h"
#include "gpio.h"


/***** Definitions *****/
#define I2C_MASTER MXC_I2C1 ///< I2C instance (Featherboard)

#define I2C_FREQ 4000 ///< I2C clock frequency
#define SENSOR_HUB_ADDR 0x55 //MAX32664 Sensor Hub Starting Address 
#define ADXL345_ADDR 0x1D //ADXL345 Device Starting Address

#define E_NO_ERROR 0 //Variable to check for errors over I2C initialization

// Max32664 Commands 
sfe_bio_ctx_t bioHub; 

#define SENSOR_I2C_PORT    MXC_I2C1
#define SENSOR_RESET_PORT  MXC_GPIO0       // Example: Reset pin on GPIO Port 0
#define SENSOR_RESET_PIN   MXC_GPIO_PIN_5  // Example: Reset pin is Pin 5
#define SENSOR_MFIO_PORT   MXC_GPIO0       // Example: MFIO pin on GPIO Port 0
#define SFE_MFIO_PIN       MXC_GPIO_PIN_6  // Example: MFIO pin is Pin 6

#define SUBCMD_EN 0x00 //enter application mode of sensor 
#define READ_DATA_CMD 0x12 //address for read data command 


//ADXL345 Commands
#define ADXL345_PWR_CTRL 0x2D //Set device in measurement mode  
#define ADXL345_DTA_FORM 0x31 //Manipulate format of data in terms of G's
#define ADXL345_BW_Rate 0x2C //manipulate sampling rate of accelerometer 
#define ADXL345_DTA 0x32 //Beginning address of readable data


// *****************************************************************************
int main(void)
{
    //Configure I2C
    printf("\n****************** I2C Configuration*******************\n");
    int error = MXC_I2C_Init(I2C_MASTER, 1, 0);
    if (error != E_NO_ERROR) {
        printf("I2C master configure failed with error %s\n", error);
        return error;
    }

    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);
    printf("\n****************** DFR0440 Haptic Module Control *******************\n");
    
    //DF0440 Haptic Driver Variables
    int HapticState = 0; //initialize haptic driver to low

    mxc_gpio_cfg_t HapDriver = {
    .port = MXC_GPIO1, //pointer to GPIO register
    .mask = MXC_GPIO_PIN_6, //pin mask
    .func = MXC_GPIO_FUNC_OUT, //function type
    .pad  = MXC_GPIO_PAD_NONE, //pad type
    .vssel = MXC_GPIO_VSSEL_VDDIOH, //voltage select
    .drvstr  = MXC_GPIO_DRVSTR_0 //drive strength
    };
    
    if (MXC_GPIO_Config(&HapDriver) != E_NO_ERROR) {
        printf("Error configuring GPIO\n");
        return -1;
    }

    MXC_GPIO_OutClr(HapDriver.port, HapDriver.mask);

    printf("GPIO Configured. Waiting for HapticState change...\n");

    /*
    //ADXL345 Initialization
    uint8_t init_data[2];


    //enable measurement mode
    MXC_I2C_RevA_WriteByte(ADXL345_PWR_CTRL, 0x08); // Place power control in normal measurement mode
    MXC_I2C_RevA_WriteByte(ADXL345_DTA_FORM, 0x01); // +/- 4G measurement mode
    MXC_I2C_RevA_WriteByte(ADXL345_BW_Rate, 0x0A); // set sampling rate to 100 Hz

    uint8_t reg = 0x32;
    uint8_t buffer[6];

     printf("\n****************** I2C HEART RATE SENSOR DEMO *******************\n");
     //Initialize Bio Sensor Hub
    uint32_t loopCount = 0;
    int MinHeartRate = 30; //Minimum heart rate for finger detection

    SFE_Bio_Begin(&bioHub, SENSOR_I2C_PORT, SENSOR_RESET_PORT, SENSOR_RESET_PIN, SENSOR_MFIO_PORT, SFE_MFIO_PIN);
    uint8_t status = SFE_Bio_SetOperatingMode(&bioHub, SFE_BIO_MODE_TWO); //Set sensor to heart rate mode
    if (status == 0){
        MXC_Delay(MXC_DELAY_MSEC(1000));
    }

    int BioConfig = SFE_Bio_ConfigBpm(&bioHub, SFE_BIO_MODE_ONE); //Configure sensor for BPM mode one
    */

    while(1) {
        

        //Haptic Code for toggle state
        if (HapticState == 1) {
            printf("Haptic State Active \n");
            MXC_GPIO_OutSet(HapDriver.port, HapDriver.mask);
        } 
        else {
            MXC_GPIO_OutClr(HapDriver.port, HapDriver.mask);
        }
        MXC_Delay(MXC_DELAY_MSEC(100));

        while(!PB_Get(0)){}

        printf("Pushbutton Pressed \n");
        if(HapticState == 0){ 
            HapticState = 1;
        }
        else {
            HapticState = 0;
        }
        while(PB_Get(0)){}

        /*
        //Accelerometer polling of X, Y, and Z data
        mxc_i2c_req_t req;
        req.i2c = I2C_MASTER;
        req.addr = ADXL345_ADDR;
        req.tx_buf = &reg;
        req.tx_len = 1;
        req.rx_buf = buffer;
        req.rx_len = 6;
        req.restart = 1;
        req.callback = NULL;

        int error = MXC_I2C_MasterTransaction(&req);
        if (error != E_NO_ERROR) {
            printf("I2C Read Error: %d\n", error);
            MXC_Delay(50000);
            continue;
        }

        int16_t x = (buffer[1] << 8) | buffer[0];
        int16_t y = (buffer[3] << 8) | buffer[2];
        int16_t z = (buffer[5] << 8) | buffer[4];

        printf("X=%d, Y=%d, Z=%d\n", x, y, z);

        MXC_Delay(100000); // 100 ms

        //Heart Rate Sensor Polling 
        // Read the processed BPM/SpO2 data from the sensor hub
        bioData_t data = SFE_Bio_ReadBpm(&bioHub); //Read BPM data from sensor

        // Check the Finger Detected status (Byte 6 of the FIFO buffer)
        if (data.status == 0x01 && data.heartRate >= MinHeartRate) {
            
            // --- DATA PRINT STATEMENT (Extracting the values) ---
            printf("[%lu] HR: %3u bpm | SpO2: %3u%% | Conf: %3u%% | Status: 0x%02X\n",
                   loopCount,
                   data.heartRate,      // Extracted from Bytes 0, 1
                   data.oxygen,         // Extracted from Bytes 3, 4
                   data.confidence,     // Extracted from Byte 2
                   data.status      // Extracted from Byte 5
            );
            MXC_Delay(1000);

        } else {
            // Finger not detected (Byte 6 is 0x00)
            printf("[%lu] Waiting for finger...\n", loopCount);
        }

        loopCount++;
        */
        }
}

    
    
