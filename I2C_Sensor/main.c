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
//#define SENSOR_HUB_ADDR 0x55 //MAX32664 Sensor Hub Starting Address 
#define ADXL345_ADDR 0x1D //ADXL345 Device Starting Address with SDO high
#define ARDUINO_ADDR   0x55

#define E_NO_ERROR 0 //Variable to check for errors over I2C initialization

// Max32664 Commands 
sfe_bio_ctx_t bioHub; 

#define SENSOR_I2C_PORT    MXC_I2C1

#define SUBCMD_EN 0x00 //enter application mode of sensor 
#define READ_DATA_CMD 0x12 //address for read data command 


//ADXL345 Commands


#define ADXL345_PWR_CTRL 0x2D //Set device in measurement mode  
#define ADXL345_DTA_FORM 0x31 //Manipulate format of data in terms of G's
#define ADXL345_BW_Rate 0x2C //manipulate sampling rate of accelerometer 
#define ADXL345_DTA 0x32 //Beginning address of readable data

typedef struct {
    float heartRate;
    float confidence;
    float oxygen;
    uint8_t status;
    uint8_t extStatus;
    float rValue;
} bioData_t;



// *****************************************************************************
int main(void)
{ 
    MXC_Delay(SEC(1));

    struct adxl345_dev *adxl_dev = NULL;
    struct adxl345_init_param init_param;
    int32_t status = 0;
    
    // Acceleration readings
    float x_g, y_g, z_g;
    int16_t x_raw, y_raw, z_raw;

    printf("\n***** MAX78000 ADXL345 I2C Demo *****\n");

    // Initialize parameters for the ADXL345 driver
    init_param.i2c_inst = SENSOR_I2C_PORT;
    init_param.i2c_addr = ADXL345_ADDR;
    init_param.dev_type = ID_ADXL345;
    
    // Set preferred range and resolution (optional, defaults to +/- 2g, 10-bit)
    init_param.selected_range = ADXL345_RANGE_PM_8G; // Set to +/- 8g range
    init_param.full_resolution_set = ADXL345_FULL_RES; // Enable Full Resolution (13-bit)

    // 1. Initialize the ADXL345 device (initializes I2C and checks device ID)
    status = adxl345_init(&adxl_dev, init_param);
    
    if (status != 0) {
        printf("ERROR: ADXL345 Initialization Failed! (Status: %ld)\n", status);
        printf("Check I2C connections and slave address (0x%X).\n", ADXL345_ADDR);
        MXC_Delay(MXC_DELAY_SEC(5));
        return 1;
    }
    
    printf("ADXL345 initialized successfully.\n");
    
    // 2. Configure the sensor's measurement properties
    
    // Set range and resolution
    adxl345_set_range_resolution(adxl_dev, 
                                 init_param.selected_range, 
                                 init_param.full_resolution_set);
    
    // Put the device in Measure Mode (continuous polling)
    adxl345_set_power_mode(adxl_dev, 1);
    printf("ADXL345 set to Measure Mode.\n");

    /*
    //Configure I2C
    printf("\n****************** I2C Configuration*******************\n");
    int error = MXC_I2C_Init(I2C_MASTER, 1, 0);
    if (error != E_NO_ERROR) {
        printf("I2C master configure failed with error %s\n", error);
        return error;
    }

    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);
    */


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

   int bio_init_status = SFE_Bio_Begin(&bioHub, SENSOR_I2C_PORT, MAX32664_RST.port, MAX32664_RST.mask, MAX32664_MFIO.port, MAX32664_MFIO.mask);
    if (bio_init_status != 0) {
        printf("ERROR: SFE_Bio_Begin failed with code: %d\n", bio_init_status);
        while(1); // Stop execution
    } else {
        printf("MAX32664 initialized successfully.\n");
    }

    int BioConfig = SFE_Bio_ConfigBpm(&bioHub, SFE_BIO_MODE_ONE); //Configure sensor for BPM mode one
    if (BioConfig != 0) {
        printf("ERROR: SFE_Bio_ConfigBpm failed with code: %d\n", BioConfig);
        while(1); // Stop execution
    } else {
        printf("BPM Mode configured successfully.\n");
    }

    // Attempt to enable the MAX30101 sensor
    int8_t control_status = SFE_Bio_Max30101Control(&bioHub, 1);

    if (control_status == 0) {
        // Command sent successfully, now verify the state
        uint8_t sensor_state = SFE_Bio_ReadMAX30101State(&bioHub);
        
        if (sensor_state == 1) {
            printf("MAX30101 is enabled and should be running.\n");
            // If the LED is still off here, the issue is likely power or the MAX30101 itself.
        } else {
            printf("MAX30101 control command succeeded, but state read back is NOT enabled (State: %u).\n", sensor_state);
            // This indicates a subtle failure or a timing issue.
        }
    } else {
        printf("ERROR: Failed to send MAX30101 control command (Error: %d).\n", control_status);
        // This indicates a fundamental I2C communication issue with the MAX32664 hub.
    }
    */

    while(1) {
        
        adxl345_get_g_xyz(adxl_dev, &x_g, &y_g, &z_g);
        
        printf("G's: X: %+1.3f | Y: %+1.3f | Z: %+1.3f\n", x_g, y_g, z_g);
        
        /* // Option 2: Read and print raw 16-bit values (uncomment if preferred)
        adxl345_get_xyz(adxl_dev, &x_raw, &y_raw, &z_raw);
        printf("Raw: X: %d | Y: %d | Z: %d\n", x_raw, y_raw, z_raw);
        */

        // Delay before the next reading
        MXC_Delay(MXC_DELAY_MSEC(100));
    
        /*
        bioData_t data;
        memset(&data, 0, sizeof(bioData_t));

        // Request sizeof(bioData_t) bytes from Arduino
        mxc_i2c_req_t req;
        req.i2c            = I2C_MASTER;
        req.addr           = ARDUINO_ADDR;
        req.tx_buf         = NULL;       // pure read
        req.tx_len         = 0;
        req.rx_buf         = (uint8_t*)&data;
        req.rx_len         = sizeof(bioData_t);
        req.restart        = 0;

        int result = MXC_I2C_MasterTransaction(&req);

        if (result != E_NO_ERROR) {
            printf("I2C Read Error: %d\n", result);
        } else {
            // Print data just like Arduino Serial Monitor
            printf("Heartrate: %.2f\n", data.heartRate);
            printf("Confidence: %.2f\n", data.confidence);
            printf("Oxygen: %.2f\n", data.oxygen);
            printf("Status: %u\n", data.status);
            printf("Extended Status: %u\n", data.extStatus);
            printf("Blood Oxygen R value: %.3f\n\n", data.rValue);
        }

        MXC_Delay(250000); // 250ms
        */

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
        
        
        //Accelerometer polling of X, Y, and Z data
                

        /*
        //Heart Rate Sensor Polling 
        // Read the processed BPM/SpO2 data from the sensor hub
        bioData_t data = SFE_Bio_ReadBpm(&bioHub); //Read BPM data from sensor

        // Check the Finger Detected status (Byte 6 of the FIFO buffer)
        if (data.status == 0x01) {
            
            // --- DATA PRINT STATEMENT (Extracting the values) ---
            printf("[%lu] HR: %3u bpm | SpO2: %3u%% | Conf: %3u%% | Status: 0x%02X\n",
                   loopCount,
                   data.heartRate,      // Extracted from Bytes 0, 1
                   data.oxygen,         // Extracted from Bytes 3, 4
                   data.confidence,     // Extracted from Byte 2
                   data.status      // Extracted from Byte 5
            );
            MXC_Delay(MXC_DELAY_MSEC(250));

        } else {
            // Finger not detected (Byte 6 is 0x00)
            if (loopCount % 1000 == 0){
                printf("[%lu] Waiting for finger...\n", loopCount/1000);
            }
            loopCount++;
        }
        */ 
        }       
    
}

    
    
