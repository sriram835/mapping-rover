#include "stm32f4xx.h"
#include "vl53l0x_api.h"
#include "my_platform.h"
#include "vl53l0x_i2c_platform.h"

volatile uint8_t model_id;








void I2C_start(){
	I2C1->CR1 |= (1U << 10);
	I2C1->CR1 |= (1U<<8);
	
	while (!(I2C1->SR1 & (1<< 0))){}
		
}

void I2C_write(uint8_t var){
	while(!(I2C1->SR1 & (1U << 7))){}
	I2C1->DR = var;
	while (!(I2C1->SR1 & (1<<2))){}
}

void I2C_send_addr(uint8_t addr, uint8_t read)
{
    I2C1->DR = (addr << 1) | read;

    while (!(I2C1->SR1 & (1U << 1))) {}

    /* Clear ADDR by reading SR1 followed by SR2 */
    volatile uint32_t temp;

    temp = I2C1->SR1;
    temp = I2C1->SR2;
}

uint8_t I2C_read_reg(uint8_t dev_addr, uint8_t reg)
{
    uint8_t data;

    /* START */
    I2C1->CR1 |= (1U << 8);

    while (!(I2C1->SR1 & (1U << 0))) {}

    /* Device address + WRITE */
    I2C1->DR = dev_addr << 1;

    while (!(I2C1->SR1 & (1U << 1))) {}

    /* Clear ADDR */
    volatile uint32_t temp;
    temp = I2C1->SR1;
    temp = I2C1->SR2;

    /* Send register address */
    while (!(I2C1->SR1 & (1U << 7))) {}

    I2C1->DR = reg;

    /* Wait until register byte has been transmitted */
    while (!(I2C1->SR1 & (1U << 2))) {}

    /* Repeated START */
    I2C1->CR1 |= (1U << 8);

    while (!(I2C1->SR1 & (1U << 0))) {}

    /* Device address + READ */
    I2C1->DR = (dev_addr << 1) | 1;

    while (!(I2C1->SR1 & (1U << 1))) {}

    /* Clear ADDR */
    temp = I2C1->SR1;
    temp = I2C1->SR2;

    /* Disable ACK because we're receiving one byte */
    I2C1->CR1 &= ~(1U << 10);

    /* STOP */
    I2C1->CR1 |= (1U << 9);

    /* Wait for received byte */
    while (!(I2C1->SR1 & (1U << 6))) {}

    data = I2C1->DR;

    /* Re-enable ACK */
    I2C1->CR1 |= (1U << 10);

    return data;
}

void I2C_stop(void){
	I2C1->CR1 |= (1U << 9);
}

VL53L0X_Dev_t dev = {
    .I2cDevAddr = 0x52,
    .comms_type = I2C,
    .comms_speed_khz = 100
};

VL53L0X_Error status;
VL53L0X_RangingMeasurementData_t ranging_data;
volatile uint16_t distance;
volatile uint8_t temp_model;


volatile 
	uint8_t buffer[10]; 
volatile uint8_t write_test;
volatile uint8_t read_test;

volatile uint8_t write_data[3] = {0x00, 0x11, 0x22};

volatile uint8_t read_1;
volatile uint8_t read_2;
volatile uint8_t read_3;
int main(void){
	uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
	init_func(); 
	SysTick_Init();
	VL53L0X_DataInit(&dev);
	VL53L0X_StaticInit(&dev);
	    VL53L0X_PerformRefSpadManagement(
        &dev,
        &refSpadCount,
        &isApertureSpads
    );

    VL53L0X_PerformRefCalibration(
        &dev,
        &VhvSettings,
        &PhaseCal
    );
	VL53L0X_SetDeviceMode(&dev,VL53L0X_DEVICEMODE_SINGLE_RANGING);
	
	
	while(1){
		        VL53L0X_PerformSingleRangingMeasurement(
            &dev,
            &ranging_data
        );
				distance = ranging_data.RangeMilliMeter;
				delay_ms(100);

	}
	
	
	
	/*
	status = VL53L0X_DataInit(&dev);

    status = VL53L0X_StaticInit(&dev);

    status = VL53L0X_SetDeviceMode(
        &dev,
        VL53L0X_DEVICEMODE_SINGLE_RANGING
    );
	    while (1)
    {
        VL53L0X_PerformSingleRangingMeasurement(
            &dev,
            &ranging_data
        );

        // Distance is in mm
        distance = ranging_data.RangeMilliMeter;

        // use distance here
    }
		*/

}