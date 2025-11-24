/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "i2c.h"
#include "adxl345.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "board.h"
#include "mxc_errors.h"
#include "SparkFun_Bio_Sensor_Hub.h"
#include "gpio.h"




/***** Definitions *****/
#define I2C_MASTER MXC_I2C1 ///< I2C instance (Featherboard)

#define I2C_FREQ 400000 ///< I2C clock frequency
#define SENSOR_HUB_ADDR 0x55 //MAX32664 Sensor Hub Starting Address 
#define ADXL345_ADDR 0x1D //ADXL345 Device Starting Address with SDO high

#define E_NO_ERROR 0 //Variable to check for errors over I2C initialization

// Max32664 Commands 
sfe_bio_ctx_t bioHub; 

#define SENSOR_I2C_PORT MXC_I2C1

#define SUBCMD_EN 0x00 //enter application mode of sensor 
#define READ_DATA_CMD 0x12 //address for read data command 


//ADXL345 Commands
adxl345_handle_t ADXL345;

#define ADXL345_PWR_CTRL 0x2D //Set device in measurement mode  
#define ADXL345_DTA_FORM 0x31 //Manipulate format of data in terms of G's
#define ADXL345_BW_Rate 0x2C //manipulate sampling rate of accelerometer 
#define ADXL345_DTA 0x32 //Beginning address of readable data


// *****************************************************************************
int main(void)
{ 
    MXC_Delay(SEC(1));

    //Configure I2C
    printf("\n****************** I2C *******************\n");
    int error = MXC_I2C_Init(I2C_MASTER, 1, 0);
    if (error != E_NO_ERROR) {
        printf("I2C master configure failed with error %s\n", error);
        return error;
    }

    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);
    
    /*
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
    */

    /*
    //ADXL345 Initialization
    uint8_t init_data[2];
    adxl345_init(&ADXL345);
    adxl345_set_addr_pin(&ADXL345, ADXL345_ADDR);

    adxl345_set_mode(&ADXL345, ADXL345_MODE_BYPASS);
    adxl345_set_interface(&ADXL345, ADXL345_INTERFACE_IIC);
    adxl345_set_range(&ADXL345, ADXL345_RANGE_8G);
    adxl345_set_rate(&ADXL345, ADXL345_RATE_400);

    uint8_t reg = 0x32;
    uint8_t buffer[6];
    */
    
    
     printf("\n****************** I2C HEART RATE SENSOR DEMO *******************\n");


     //Initialize Bio Sensor Hub
    uint32_t loopCount = 0;
    int MinHeartRate = 30; //Minimum heart rate for finger detection

    mxc_gpio_cfg_t MAX32664_RST = {
    .port = MXC_GPIO0, //pointer to GPIO register
    .mask = MXC_GPIO_PIN_5, //pin mask
    .func = MXC_GPIO_FUNC_OUT, //function type
    .pad  = MXC_GPIO_PAD_NONE, //pad type
    .vssel = MXC_GPIO_VSSEL_VDDIOH, //voltage select
    .drvstr  = MXC_GPIO_DRVSTR_0, //drive strength
    };
    
    if(MXC_GPIO_Config(&MAX32664_RST) != E_NO_ERROR) {
        printf("Error configuring GPIO\n");
        return -1;
    };

    mxc_gpio_cfg_t MAX32664_MFIO = {
    .port = MXC_GPIO0, //pointer to GPIO register
    .mask = MXC_GPIO_PIN_19, //pin mask
    .func = MXC_GPIO_FUNC_OUT, //function type
    .pad  = MXC_GPIO_PAD_NONE, //pad type
    .vssel = MXC_GPIO_VSSEL_VDDIOH, //voltage select
    .drvstr  = MXC_GPIO_DRVSTR_0 //drive strength
    };
   
    if(MXC_GPIO_Config(&MAX32664_MFIO) != E_NO_ERROR) {
        printf("Error configuring GPIO\n");
        return -1;
    }

    mxc_gpio_cfg_t MAX32664_MFIO_IN = {
        .port = MXC_GPIO0,
        .mask = MXC_GPIO_PIN_19,
        .func = MXC_GPIO_FUNC_IN,    // *** CRITICAL CHANGE: Set as Input ***
        .pad  = MXC_GPIO_PAD_PULL_UP, // Use an internal pullup for open-drain line
        .vssel = MXC_GPIO_VSSEL_VDDIOH,
    };

    if(MXC_GPIO_Config(&MAX32664_MFIO_IN) != E_NO_ERROR) {
        printf("Error reconfiguring MFIO to input\n");
        return -1;
    } 

    MXC_Delay(MXC_DELAY_MSEC(100)); // 100ms delay after reconfiguration

    int bio_init_status = SFE_Bio_Begin(&bioHub, SENSOR_I2C_PORT, MAX32664_RST.port, MAX32664_RST.mask, MAX32664_MFIO.port, MAX32664_MFIO.mask);
    if (bio_init_status != 0) {
        printf("ERROR: SFE_Bio_Begin failed with code: %d\n", bio_init_status);
        while(1); // Stop execution
    } else {
        printf("MAX32664 initialized successfully.\n");
    }
    uint8_t state = SFE_Bio_ReadMAX30101State(&bioHub);
    printf("State after Begin: %u\n", state);

    uint8_t dummy = 0;
    mxc_i2c_req_t req = {
        .i2c = SENSOR_I2C_PORT,
        .addr = 0x55,
        .tx_buf = &dummy,
        .tx_len = 1,
        .rx_buf = NULL,
        .rx_len = 0,
        .restart = 0,
    };
    int ping = MXC_I2C_MasterTransaction(&req);
    printf("MAX32664 ping => %d\n", ping);

    int BioConfig = SFE_Bio_ConfigBpm(&bioHub, SFE_BIO_MODE_TWO); //Configure sensor for BPM mode two
    if (BioConfig != 0) {
        printf("ERROR: SFE_Bio_ConfigBpm failed with code: %d\n", BioConfig);
        while(1); // Stop execution
    } else {
        printf("BPM Mode configured successfully.\n");
    }

    MXC_Delay(MXC_DELAY_SEC(4)); // 4 seconds delay after Begin

    dummy = 0;
    mxc_i2c_req_t req1 = {
        .i2c = SENSOR_I2C_PORT,
        .addr = 0x57,
        .tx_buf = &dummy,
        .tx_len = 1,
        .rx_buf = NULL,
        .rx_len = 0,
        .restart = 0,
    };
    ping = MXC_I2C_MasterTransaction(&req1);
    printf("MAX30101 ping => %d\n", ping);

    // Attempt to enable the MAX30101 sensor
    int8_t control_status = SFE_Bio_Max30101Control(&bioHub, 1);

    if (control_status == 0) {
        // Command sent successfully, now verify the state
        uint8_t sensor_state = SFE_Bio_ReadMAX30101State(&bioHub);

        MXC_Delay(MXC_DELAY_MSEC(100)); // 100ms delay after Begin
        
        if (sensor_state == 1) {
            printf("MAX30101 is enabled and should be running.\n");
            // If the LED is still off here, the issue is likely power or the MAX30101 itself.
        } else {
            printf("State read back is NOT enabled (State: %u).\n", sensor_state);
            // This indicates a subtle failure or a timing issue.
        }
    } else {
        printf("ERROR: Failed to send MAX30101 control command (Error: %d).\n", control_status);
        // This indicates a fundamental I2C communication issue with the MAX32664 hub.
    }

    /*
    mxc_gpio_cfg_t MAX32664_MFIO_IN = {
        .port = MXC_GPIO0,
        .mask = MXC_GPIO_PIN_19,
        .func = MXC_GPIO_FUNC_IN,    // *** CRITICAL CHANGE: Set as Input ***
        .pad  = MXC_GPIO_PAD_PULL_UP, // Use an internal pullup for open-drain line
        .vssel = MXC_GPIO_VSSEL_VDDIOH,
    };

    if(MXC_GPIO_Config(&MAX32664_MFIO_IN) != E_NO_ERROR) {
        printf("Error reconfiguring MFIO to input\n");
        return -1;
    }


    printf("MFIO successfully reconfigured for reading interrupts.\n");
    */
    

    while(1) {
        
        /*
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
        */
        
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
        */

        //Heart Rate Sensor Polling 
        // Read the processed BPM/SpO2 data from the sensor hub
        //if (MXC_GPIO_InGet(MAX32664_MFIO.port, MAX32664_MFIO.mask) == 0) {
            
        // 2. READ: Only read when data is ready
        bioData_t data = SFE_Bio_ReadBpm(&bioHub);

        // 3. FILTER: Check Status AND Confidence
        // Status 1 = Finger Detected
        // Confidence > 50% = Algorithm actually found a heartbeat
        if (data.status == 0x01 || data.status == 0x03) { // 0x03 is sometimes used for 'Object Ready'
            
            if(data.confidence > 50 && data.heartRate >= MinHeartRate) {
                printf("[%lu] VALID DATA - HR: %u bpm | SpO2: %u%% | Conf: %u%%\n",
                    loopCount, data.heartRate, data.oxygen, data.confidence);
            } 
            else {
                // Finger is there, but sensor is still calibrating
                printf("[%lu] Calibrating... Keep finger still. Conf: %u%%\n", loopCount, data.confidence);
            }
            
        } else {
            // Status indicates no finger
            printf("No Finger Detected.\n");
        }
        //}
        
        // Do not put a massive delay here, or you will miss the MFIO window.
        // A tiny delay is fine.
        MXC_Delay(MXC_DELAY_MSEC(250));
        loopCount++;
    }
               
        
}