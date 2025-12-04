#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "i2c.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "board.h"
#include "mxc_errors.h"
#include "gpio.h"
#include "sensorHub.h"

#define I2C_PORT MXC_I2C1
#define I2C_FREQ 400000
#define MAX32664_ADDR 0x55

#define NUM_SAMPLES 0x0F

#define RSTN_PORT MXC_GPIO0
#define RSTN_PIN  MXC_GPIO_PIN_19
#define MFIO_PORT MXC_GPIO0
#define MFIO_PIN  MXC_GPIO_PIN_11

mxc_gpio_cfg_t gpio_rstn;
mxc_gpio_cfg_t gpio_mfio;
volatile uint16_t hr;
volatile uint8_t conf;
volatile uint16_t spo2;
volatile uint8_t state;

void gpio_isr(void *cbdata)
{
    uint16_t hri;
    uint8_t confi;
    uint16_t spo2i;
    uint8_t statei;
    uint8_t status = read_bpm_algo(I2C_PORT, MAX32664_ADDR, &hri, &confi, &spo2i, &statei);
    if (status != E_NO_ERROR) {
        printf("Error Reading BPM Data, Status: %d\n", status);
    } else {
        hr = hri;
        conf = confi;
        spo2 = spo2i;
        state = statei;
    }

}


int main(void)
{ 
    uint8_t status;
    uint8_t mode;

    MXC_Delay(SEC(1));

    //Configure I2C
    printf("****************** I2C *******************\n");
    int error = MXC_I2C_Init(I2C_PORT, 1, 0);
    if (error != E_NO_ERROR) {
        printf("I2C master configure failed with error %s\n", error);
        return error;
    }
    MXC_I2C_SetFrequency(I2C_PORT, I2C_FREQ);

    // Configure RSTN 
    gpio_rstn.port = RSTN_PORT;
    gpio_rstn.mask = RSTN_PIN;
    gpio_rstn.pad = MXC_GPIO_PAD_NONE;
    gpio_rstn.func = MXC_GPIO_FUNC_OUT;
    gpio_rstn.vssel = MXC_GPIO_VSSEL_VDDIO;
    gpio_rstn.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&gpio_rstn);

    // Configure MFIO 
    gpio_mfio.port = MFIO_PORT;
    gpio_mfio.mask = MFIO_PIN;
    gpio_mfio.pad = MXC_GPIO_PAD_PULL_UP;
    gpio_mfio.func = MXC_GPIO_FUNC_IN;
    gpio_mfio.vssel = MXC_GPIO_VSSEL_VDDIO;
    gpio_mfio.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&gpio_mfio);

    MXC_Delay(MXC_DELAY_SEC(1));

    // Put Sensor Hub in Application Mode
    MXC_GPIO_OutSet(gpio_mfio.port, gpio_mfio.mask); // High
    MXC_GPIO_OutClr(gpio_rstn.port, gpio_rstn.mask); // Low
    MXC_Delay(MXC_DELAY_MSEC(10));
    MXC_GPIO_OutSet(gpio_rstn.port, gpio_rstn.mask); // High
    MXC_Delay(MXC_DELAY_MSEC(1500));


    // Ensure Sensor Hub is in Application Mode
    status = read_sensor_hub_mode(I2C_PORT, MAX32664_ADDR, &mode);
    if (status != E_NO_ERROR) {
        printf("Status of read_sensor_hub_mode: 0x%02X\n", status);
        return status;
    } else if(mode == 0x02) {
        printf("MAX32664 not in application mode. Mode: 0x%02X\n", mode);
        return mode;
    }

    printf("\nMAX32664 Initialization Begin\n");

    status = init_max32664(I2C_PORT, MAX32664_ADDR, NUM_SAMPLES);
    if (status != 0x00) {
        printf("Initialization Failed. Status: 0x%02x", status);
        return status;
    }

    printf("MAX32664 Initialization Complete\n");

    MXC_GPIO_RegisterCallback(&gpio_mfio, gpio_isr, NULL);
    MXC_GPIO_IntConfig(&gpio_mfio, MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(gpio_mfio.port, gpio_mfio.mask);
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(MFIO_PORT)));

    MXC_Delay(MXC_DELAY_MSEC(100));

    while(1) {
        MXC_Delay(MXC_DELAY_MSEC(100));

        uint16_t safe_hr;
        uint16_t safe_spo2;

        disable_interrupts(); 
        
        safe_hr = hr;
        safe_spo2 = spo2;
        
        uint8_t safe_conf = conf;
        uint8_t safe_state = state;
        
        enable_interrupts();

        if (safe_state == 0x03) {
            printf("Heart Rate: %.1f bpm, SpO2: %.1f%%, Conf: %d%%\n", (float)safe_hr/10.0, (float)safe_spo2/10.0, safe_conf);
        } else {
            printf("State: 0x%02X\n", safe_state);
        }

    }

    return 0;
}
