#include "vl53l0x_api.h"
#include "stm32f4xx.h"
#include "my_platform.h"
#include "vl53l0x_def.h"

volatile uint32_t ms_ticks = 0;
volatile uint32_t i2c_debug_stage;
volatile uint32_t i2c_debug_sr1;
volatile uint32_t i2c_debug_sr2;
volatile uint32_t i2c_debug_gpio;
volatile uint32_t i2c_debug_dev_addr;
volatile uint32_t i2c_debug_reg;
volatile uint32_t i2c_debug_count;
volatile uint32_t scl = 0;
volatile uint32_t sda = 0;


VL53L0X_Dev_t dev = {
    .I2cDevAddr = 0x52,
    .comms_type = I2C,
    .comms_speed_khz = 100
};


void init_func(void){
	   /* HSI is 16 MHz and is enabled by default after reset */
	    RCC->CR |= RCC_CR_HSION;

    while (!(RCC->CR & RCC_CR_HSIRDY))
    {
    }
		
    /*
     * -----------------------------------------------------
     * 2. Temporarily use HSI as SYSCLK
     *
     * This makes the function independent of whatever
     * clock source the startup code previously selected.
     * -----------------------------------------------------
     */

    RCC->CFGR &= ~RCC_CFGR_SW;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
    {
    }



    /* Flash: instruction cache, data cache, prefetch, 2 wait states */
    FLASH->ACR = FLASH_ACR_ICEN |
                 FLASH_ACR_DCEN |
                 FLASH_ACR_PRFTEN |
                 FLASH_ACR_LATENCY_2WS;


    /* Select HSI as PLL source */
    /* PLLSRC = 0 means HSI */

	/* APB1 = HCLK / 2 = 42 MHz */
	RCC->CFGR &= ~(RCC_CFGR_HPRE |
									RCC_CFGR_PPRE1 |
									RCC_CFGR_PPRE2);

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
	
	
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

		
    PWR->CR &= ~PWR_CR_VOS;

    /*
     * On STM32F401, VOS = 01 corresponds to Scale 2.
     */
    PWR->CR |= PWR_CR_VOS_1;
		
		
    /*
     * PLL configuration:
     *
     * HSI = 16 MHz
     * PLLM = 16  -> 16 / 16 = 1 MHz
     * PLLN = 336 -> 1 * 336 = 336 MHz
     * PLLP = 4   -> 336 / 4 = 84 MHz
     *
     * PLLQ = 7   -> 48 MHz USB clock
     */
    RCC->PLLCFGR = (16U << 0)  |       /* PLLM = 16 */
                   (336U << 6) |       /* PLLN = 336 */
                   (1U << 16)  |       /* PLLP = /4 */
                   (7U << 24);         /* PLLQ = 7 */
									 
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;


    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
		
		

    /* Wait for PLL to become ready */
    while (!(RCC->CR & RCC_CR_PLLRDY)) {}

    /* Switch SYSCLK to PLL */
			
    RCC->CFGR &= ~RCC_CFGR_SW;

    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait until PLL is actually selected as SYSCLK */
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
	RCC->APB1ENR |= (1U << 21);
	RCC->AHB1ENR |= (1U << 1);
	GPIOB->MODER &= ~((3U << 16) | (3U << 18));
	GPIOB->MODER |=  ((2U << 16) | (2U << 18));

	GPIOB->PUPDR &= ~((3U << 16) | (3U << 18));
	GPIOB->PUPDR |=  ((1U << 16) | (1U << 18));

	GPIOB->OTYPER |= (1U << 8) | (1U << 9);

	GPIOB->OSPEEDR &= ~((3U << 16) | (3U << 18));
	GPIOB->OSPEEDR |=  ((2U << 16) | (2U << 18));

	GPIOB->AFR[1] &= ~((0xFU << 0) | (0xFU << 4));
	GPIOB->AFR[1] |=  ((4U << 0) | (4U << 4));
	
	  /* Disable I2C while configuring */

	I2C1->CR1 |=  (1U << 15);  /* SWRST = 1 */
	I2C1->CR1 &= ~(1U << 15);

  I2C1->CR1 &= ~(1U << 0);
	

    /*
     * APB1 = 42 MHz
     */
    I2C1->CR2 |= (42U << 0);

    /*
     * Standard Mode, 100 kHz
     *
     * CCR = 42 MHz / (2 * 100 kHz)
     *     = 210
     */
    I2C1->CCR = 210;

    /*
     * Standard Mode:
     * TRISE = 42 + 1
     */
    I2C1->TRISE = 43;

    /* Enable I2C */
    I2C1->CR1 |= (1U << 0);
		
		// ----------------------- SysTick Init -------------------------------- //
		SysTick_Init();
		
		
		// -------------------------- Lidar Init ----------------------------- //
		
		uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;


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
		
		

	FixPoint1616_t signalRate = (FixPoint1616_t)(0.7 * 65536);
	FixPoint1616_t sigma = (FixPoint1616_t)(2 * 65536);
	uint32_t timing_budget = 500000;

	VL53L0X_SetLimitCheckValue(
	&dev,
	VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
	signalRate
	);

	VL53L0X_SetLimitCheckValue(
	&dev,
	VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
	sigma
	);

	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
	&dev,
	timing_budget
	);

		
	VL53L0X_SetDeviceMode(&dev,VL53L0X_DEVICEMODE_SINGLE_RANGING);
	
	
	// ---------------------------- Servo -------------------------- //
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	GPIOA->MODER &= ~(3U << (9 * 2));
	GPIOA->MODER |= (2U << (9 * 2));
	

	GPIOA->OTYPER &= ~(1U << 9);
	
	GPIOA->PUPDR &= ~(3U << (9 * 2));
	GPIOA->PUPDR |= (1U << (9 * 2));
	
	GPIOA->OSPEEDR &= ~(3U << (9 * 2));
	GPIOA->OSPEEDR |= (3U << (9 * 2));

	GPIOA->AFR[1] &= ~(0xFU << 4);
	GPIOA->AFR[1] |= (7U << 4);
		
	// ------------------------------ //

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->CR1 = 0;
	USART1->CR2 = 0;
	USART1->CR3 = 0;
	
	

	/*
	* -----------------------------------------------------
	* Baud rate
	*
	* PCLK2 = 84 MHz
	*
	* Desired baud = 1 MHz
	*
	* With oversampling by 16:
	*
	* USARTDIV = 84 MHz / 1 MHz
	*          = 84
	*
	* Therefore:
	*
	* BRR = 84 = 0x0054
	* -----------------------------------------------------
	*/

	USART1->BRR = 84;
	

	/*
	* -----------------------------------------------------
	* Half-duplex mode
	*
	* TX and RX internally connected.
	* -----------------------------------------------------
	*/

	USART1->CR3 |= USART_CR3_HDSEL;
	
	

	/*
	* -----------------------------------------------------
	* Enable transmitter and receiver
	* -----------------------------------------------------
	*/

	USART1->CR1 |= USART_CR1_TE;
	USART1->CR1 |= USART_CR1_RE;


	/*
	* Enable USART
	*/

	USART1->CR1 |= USART_CR1_UE;


	/*
	* Wait until USART is enabled
	*/

	while (!(USART1->CR1 & USART_CR1_UE)){}
}







void i2c_start(void){
	I2C1->CR1 |= (1<<10);  // Enable the ACK
	I2C1->CR1 |= (1<<8);  // Generate START	
	while (!(I2C1->SR1 & (1U << 0))) {}  // Wait for SB
}

void i2c_send_addr(uint8_t address){
	I2C1->DR = address;  //  send the address
	while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
	uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
}

void i2c_write(uint8_t data){
	while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
	I2C1->DR = data;
	while (!(I2C1->SR1 & (1<<2)));  // wait for BTF bit to set
}

void i2c_read(uint8_t Address, uint8_t* buffer, uint8_t size){
int remaining = size;
	
/**** STEP 1 ****/	
	if (size == 1)
	{
		/**** STEP 1-a ****/	
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
		
		/**** STEP 1-b ****/	
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit 
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit.... EV6 condition
		I2C1->CR1 |= (1<<9);  // Stop I2C

		/**** STEP 1-c ****/	
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		
		/**** STEP 1-d ****/	
		buffer[size-remaining] = I2C1->DR;  // Read the data from the DATA REGISTER
		
	}

/**** STEP 2 ****/		
	else 
	{
		/**** STEP 2-a ****/
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
		
		/**** STEP 2-b ****/
		uint8_t temp = I2C1->SR1 | I2C1->SR2;  // read SR1 and SR2 to clear the ADDR bit
		
		while (remaining>2)
		{
			/**** STEP 2-c ****/
			while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
			
			/**** STEP 2-d ****/
			buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer			
			
			/**** STEP 2-e ****/
			I2C1->CR1 |= 1<<10;  // Set the ACK bit to Acknowledge the data received
			
			remaining--;
		}
		
		// Read the SECOND LAST BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;
		
		/**** STEP 2-f ****/
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit 
		
		/**** STEP 2-g ****/
		I2C1->CR1 |= (1<<9);  // Stop I2C
		
		remaining--;
		
		// Read the Last BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer
	}
}
void i2c_stop(void)
{
    I2C1->CR1 |= (1 << 9);   // Generate STOP condition
}
void MPU_Write (uint8_t Address, uint8_t Reg, uint8_t Data)
{
	i2c_start();
	i2c_send_addr(Address);
	i2c_write (Reg);
	i2c_write (Data);
	i2c_stop ();
}

void MPU_Write_Multiple(uint8_t Address,
                        uint8_t Reg,
                        uint8_t *data,
                        uint8_t size)
{
    i2c_start();

    i2c_send_addr(Address);

    i2c_write(Reg);

    for (uint8_t i = 0; i < size; i++)
    {
        i2c_write(data[i]);
    }

    i2c_stop();
}

void MPU_Read (uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size)
{
	i2c_start ();
	i2c_send_addr(Address);
	i2c_write (Reg);
	i2c_start ();  // repeated start
	i2c_read (Address+0x01, buffer, size);
	i2c_stop ();
}


void SysTick_Handler(void)
{
    if (ms_ticks > 0){
			ms_ticks--;
		}
}

void delay_ms(uint32_t ms)
{
	ms_ticks = ms;
	while (ms_ticks > 0);
}



void SysTick_Init(void)
{
    /* 1 ms tick at 84 MHz */
    //SysTick->LOAD = 84000 - 1;
		SysTick->LOAD = 84000 - 1;

    /* Clear current counter value */
    SysTick->VAL = 0;

    /*
     * CLKSOURCE = 1 -> processor clock (HCLK)
     * TICKINT   = 1 -> enable SysTick interrupt
     * ENABLE    = 1 -> start counter
     */
    SysTick->CTRL =
        (1U << 2) |
        (1U << 1) |
        (1U << 0);
}




//int32_t I2C_write_multi(uint8_t dev_addr,
//                     uint8_t reg,
//                     uint8_t *pdata,
//                     uint32_t count)
//{
//    uint32_t i;
//    volatile uint32_t temp;

//    /* -------------------------------- */
//    /* Stage 1: BUSY                    */
//    /* -------------------------------- */
//    i2c_debug_stage = 1;

//    while (I2C1->SR2 & (1U << 1))
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;
//    }

//    /* -------------------------------- */
//    /* Stage 2: START                   */
//    /* -------------------------------- */
//    i2c_debug_stage = 2;

//    I2C1->CR1 |= (1U << 8);       // START

//    while (!(I2C1->SR1 & (1U << 0)))
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;
//    }

//    /* -------------------------------- */
//    /* Stage 3: ADDRESS                 */
//    /* -------------------------------- */
//    i2c_debug_stage = 3;

//    I2C1->DR = dev_addr << 1;

//    while (!(I2C1->SR1 & ((1U << 1) | (1U << 10))))
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;
//			  scl = (GPIOB->IDR >> 8) & 1;
//				sda = (GPIOB->IDR >> 9) & 1;
//    }

//    /* Address not acknowledged */
//    if (I2C1->SR1 & (1U << 10))
//    {
//        i2c_debug_stage = 4;       // AF = address failure
//        I2C1->CR1 |= (1U << 9);    // STOP
//				while (1){}
//        return 1;
//    }

//    /* Clear ADDR */
//    temp = I2C1->SR1;
//    temp = I2C1->SR2;

//    i2c_debug_stage = 5;

//    while (!(I2C1->SR1 & (1U << 7))) {}     // TXE

//    I2C1->DR = reg;

//    while (!(I2C1->SR1 & (1U << 2))) {}     // BTF

//    for (i = 0; i < count; i++)
//    {
//        while (!(I2C1->SR1 & (1U << 7))) {}

//        I2C1->DR = pdata[i];

//        while (!(I2C1->SR1 & (1U << 2))) {}
//    }

//    I2C1->CR1 |= (1U << 9);        // STOP

//    i2c_debug_stage = 6;
//		return 0;
//}

//int32_t I2C_read_multi(uint8_t dev_addr,
//                       uint8_t reg,
//                       uint8_t *pdata,
//                       uint32_t count)
//{
//    volatile uint32_t temp;

//    i2c_debug_dev_addr = dev_addr;
//    i2c_debug_reg = reg;
//    i2c_debug_count = count;

//    if (count == 0)
//        return 1;


//    /* ========================================================= */
//    /* STAGE 1: Wait for bus to become free                      */
//    /* ========================================================= */

//    i2c_debug_stage = 1;

//    while (I2C1->SR2 & (1U << 1))       // BUSY
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;
//    }


//    /* ========================================================= */
//    /* STAGE 2: Generate START                                   */
//    /* ========================================================= */

//    i2c_debug_stage = 2;

//    I2C1->CR1 |= (1U << 8);              // START

//    while (!(I2C1->SR1 & (1U << 0)))     // SB
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;
//    }


//    /* ========================================================= */
//    /* STAGE 3: Send device address + WRITE                      */
//    /* ========================================================= */

//    i2c_debug_stage = 3;

//    I2C1->DR = dev_addr << 1;             // 0x29 -> 0x52

//    while (!(I2C1->SR1 & ((1U << 1) | (1U << 10))))
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;
//    }

//    /* Address not acknowledged */
//    if (I2C1->SR1 & (1U << 10))           // AF
//    {
//        i2c_debug_stage = 4;

//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;

//        I2C1->CR1 |= (1U << 9);           // STOP
//			while(1){}
//        return 1;
//    }


//    /* ========================================================= */
//    /* STAGE 5: Clear ADDR                                       */
//    /* ========================================================= */

//    i2c_debug_stage = 5;

//    temp = I2C1->SR1;
//    temp = I2C1->SR2;


//    /* ========================================================= */
//    /* STAGE 6: Wait TXE                                         */
//    /* ========================================================= */

//    i2c_debug_stage = 6;

//    while (!(I2C1->SR1 & (1U << 7)))      // TXE
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;
//    }


//    /* ========================================================= */
//    /* STAGE 7: Send register address                            */
//    /* ========================================================= */

//    i2c_debug_stage = 7;

//    I2C1->DR = reg;


//    /* ========================================================= */
//    /* STAGE 8: Wait BTF                                         */
//    /* ========================================================= */

//    while (!(I2C1->SR1 & (1U << 2)))      // BTF
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;
//    }


//    /* ========================================================= */
//    /* STAGE 9: Repeated START                                   */
//    /* ========================================================= */

//    i2c_debug_stage = 9;

//    I2C1->CR1 |= (1U << 8);               // START

//    while (!(I2C1->SR1 & (1U << 0)))      // SB
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;
//			
//    }


//    /* ========================================================= */
//    /* STAGE 10: Send device address + READ                      */
//    /* ========================================================= */

//    i2c_debug_stage = 10;

//    I2C1->DR = (dev_addr << 1) | 1U;      // 0x29 -> 0x53

//		while (1)
//{
//    i2c_debug_sr1 = I2C1->SR1;
//    i2c_debug_sr2 = I2C1->SR2;

//    if (i2c_debug_sr1 & (1U << 1))       // ADDR
//        break;

//    if (i2c_debug_sr1 & (1U << 10))      // AF
//        break;

//    if (i2c_debug_sr1 & (1U << 6))       // RxNE
//        break;
//}

///*
//    while (!(I2C1->SR1 & ((1U << 1) | (1U << 10))))
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//				scl = (GPIOB->IDR >> 8) & 1;
//				sda = (GPIOB->IDR >> 9) & 1;
//    }
//*/
//    /* Address not acknowledged */
//    if (I2C1->SR1 & (1U << 10))           // AF
//    {
//        i2c_debug_stage = 11;

//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;

//        I2C1->CR1 |= (1U << 9);           // STOP
//      while(1){}  
//			return 1;
//    }


//    /* ========================================================= */
//    /* STAGE 12: Configure ACK/POS before clearing ADDR          */
//    /* ========================================================= */

//    i2c_debug_stage = 12;

//    if (count == 1)
//    {
//        I2C1->CR1 &= ~(1U << 10);         // ACK = 0
//    }
//    else if (count == 2)
//    {
//        I2C1->CR1 |= (1U << 11);          // POS = 1
//        I2C1->CR1 &= ~(1U << 10);         // ACK = 0
//    }
//    else
//    {
//        I2C1->CR1 |= (1U << 10);          // ACK = 1
//    }


//    /* ========================================================= */
//    /* STAGE 13: Clear ADDR                                     */
//    /* ========================================================= */

//    i2c_debug_stage = 13;

//    temp = I2C1->SR1;
//    temp = I2C1->SR2;


//    /* ========================================================= */
//    /* count == 1                                                */
//    /* ========================================================= */

//    if (count == 1)
//    {
//        i2c_debug_stage = 14;

//        I2C1->CR1 |= (1U << 9);           // STOP

//        while (!(I2C1->SR1 & (1U << 6)))  // RXNE
//        {
//            i2c_debug_sr1 = I2C1->SR1;
//            i2c_debug_sr2 = I2C1->SR2;
//            i2c_debug_gpio = GPIOB->IDR;
//        }

//        pdata[0] = I2C1->DR;
//    }


//    /* ========================================================= */
//    /* count == 2                                                */
//    /* ========================================================= */

//    else if (count == 2)
//    {
//        i2c_debug_stage = 15;

//        while (!(I2C1->SR1 & (1U << 2)))  // BTF
//        {
//            i2c_debug_sr1 = I2C1->SR1;
//            i2c_debug_sr2 = I2C1->SR2;
//            i2c_debug_gpio = GPIOB->IDR;
//        }

//        I2C1->CR1 |= (1U << 9);           // STOP

//        pdata[0] = I2C1->DR;
//        pdata[1] = I2C1->DR;
//    }


//    /* ========================================================= */
//    /* count >= 3                                                */
//    /* ========================================================= */

//    else
//    {
//        i2c_debug_stage = 16;

//        while (count > 3)
//        {
//            while (!(I2C1->SR1 & (1U << 6)))  // RXNE
//            {
//                i2c_debug_sr1 = I2C1->SR1;
//                i2c_debug_sr2 = I2C1->SR2;
//                i2c_debug_gpio = GPIOB->IDR;
//            }

//            *pdata++ = I2C1->DR;
//            count--;
//        }


//        i2c_debug_stage = 17;

//        while (!(I2C1->SR1 & (1U << 2)))      // BTF
//        {
//            i2c_debug_sr1 = I2C1->SR1;
//            i2c_debug_sr2 = I2C1->SR2;
//            i2c_debug_gpio = GPIOB->IDR;
//        }


//        I2C1->CR1 &= ~(1U << 10);             // ACK = 0

//        *pdata++ = I2C1->DR;                   // N-2

//        I2C1->CR1 |= (1U << 9);               // STOP

//        *pdata++ = I2C1->DR;                   // N-1


//        i2c_debug_stage = 18;

//        while (!(I2C1->SR1 & (1U << 6)))      // RXNE
//        {
//            i2c_debug_sr1 = I2C1->SR1;
//            i2c_debug_sr2 = I2C1->SR2;
//            i2c_debug_gpio = GPIOB->IDR;
//        }

//        *pdata = I2C1->DR;                     // N
//    }


//    /* ========================================================= */
//    /* STAGE 19: Restore peripheral state                        */
//    /* ========================================================= */

//    i2c_debug_stage = 19;

//    I2C1->CR1 &= ~(1U << 11);                 // POS = 0
//    I2C1->CR1 |=  (1U << 10);                 // ACK = 1

//    while (I2C1->CR1 & (1U << 9))
//    {
//        i2c_debug_sr1 = I2C1->SR1;
//        i2c_debug_sr2 = I2C1->SR2;
//        i2c_debug_gpio = GPIOB->IDR;
//    }


//    i2c_debug_stage = 20;

//    return 0;
//}





