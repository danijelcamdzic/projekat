/**
 * @file main.c
 * 
 * Project that uses STM32F030R8 microcontroller and IMU-9250 sensor
 * in order to determine the orientation of the sensor and display it's
 * orientation on the PC. It shows it's orientation in all 3 axises and has
 * no or neglegible drift over time. Processing Toolbox software is used to
 * receive the data from the microcontroller and display the box representing 
 * the sensor orientation.
 *
 *  Version: 2.3
 *  Created on: June 25th, 2020
 *      Author: Danijel Camdzic
 */
 
/* Include files which are necessary for the project */
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Definitions of the sensor register addresses */
#define AccelGyroAddress  			(0x69<<1)	 				/**< Sensor slave address of the accel+gyro */
#define WHO_AM_I  				(0x75)						/**< WHO_AM_I register of the accel and gyro sensor */
#define PWR_MGMT_1  				(0x6B)						/**< Register for power management */
#define ACCEL_XOUT_H  				(0x3B)						/**< Accelerometer X-axis high byte output */
#define GYRO_XOUT_H  				(0x43)						/**< Gyroscope X-axis high byte output */
#define INT_BYPASS_CONFIG_AD  			(0x37)						/**< Register for switching accel+gyro to magnetometer slave */

#define MagnetometerAddress  			(0x0C<<1)					/**< Sensor slave address of the magnetometer */
#define WIA					(0x00)						/**< Device ID address of the magnetometer */
#define STATUS1					(0x02)						/**< Status1 register of the magnetometer slave */
#define MAGN_XOUT_L  				(0x03)						/**< Magnetometer X-axis low byte output  */
#define STATUS2					(0x09)						/**< Status2 register of the magnetometer slave */
#define CNTL1_AD  				(0x0A)						/**< Control 1 register */
#define ASAX_REG 				(0x10)						/**< X-axis sensitivity adjustment value register */

#define ACCEL_GYRO_SLAVE			(0x00)						/**< Address to switch from magnetometer slave to accel+gyro slave */
#define MAGNETOMETER_SLAVE			(0x02)						/**< Address to switch from accel+gyro slave to magnetometer slave */

#define MagneticResolution			(4912/8190.0)					/**< Value necessary to find out the right magnetometer readings */

#define PI					(3.141592)					/**< Value of the mathematical constant PI */

#define TurnToMilisecond			(0.5)						/**< Value used to turn counter readings to miliseconds */

/* Type Definitions */
struct KalmanFilter 
{
	/* Covariance matrices */
  float Q[2];				
	float R;

	float Xk[2];										/**< State variables - position and velocity */
	float P[2][2];
	
	float NoDriftGyroRate;									/**< Velocity from the gyroscopes adjusted for drift */
	
} KalmanFilter; 

/* Variables */
struct KalmanFilter KalmanX;									/**< Kalman filter for X-axis */
struct KalmanFilter KalmanY;									/**< Kalman filter for Y-axis */
struct KalmanFilter KalmanZ;									/**< Kalman filter for Z-axis */

uint8_t buf[12] = {0};										/**< Array used for sendind messages to UART */
uint8_t data[8] = {0};										/**< Array used for sending data bytes */

int16_t X_AXIS = 0;										/**< Variable used to receive 2 bytes from the X-axis AccelZStartingments */
int16_t Y_AXIS = 0;										/**< Variable used to receive 2 bytes from the Y-axis AccelZStartingments  */
int16_t Z_AXIS = 0;										/**< Variable used to receive 2 bytes from the Z-axis AccelZStartingments  */
float X_OUTPUT = 0.0;										/**< Final X-axis value */
float Y_OUTPUT = 0.0;										/**< Final Y-axis value */
float Z_OUTPUT = 0.0;										/**< Final Z-axis value */

uint8_t ASAX = 0;										/**< X-axis adjustment value for the magnetometer */
uint8_t ASAY = 0;										/**< Y-axis adjustment value for the magnetometer */
uint8_t ASAZ = 0;										/**< Z-axis adjustment value for the magnetometer */

float pitch = 0.0;										/**< Pitch value */
float roll = 0.0;										/**< Roll value */
float yaw = 0.0;										/**< Yaw value */

float xaccel = 0.0;
float yaccel = 0.0;
float zaccel = 0.0;

float MagneticOffset[3] = {-0.5, 3.0, -61.5};							/**< Magnetic offset for the magnetometer sensor. Calculated during one
																								of the test runs from the function inside this project. */

/* Function Definitions */
void Clock_Init(void);
void GPIO_Init(void);
void TIMER3_Init(void);
void USART2_Init(void);
void I2C1_Init(void);
void I2C_Read(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *pdata);
void I2C_Write(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *pdata);

void KalmanFilterXYZ(struct KalmanFilter *Kalman, float GyroRate, float AccelAngle, float dt);
void InitKalmanXYZ(void);
void WakeUpSensor(void);
void GetMagnetometerAdjustment(void);
void SwitchSlaveDevice(uint8_t command);
void CalculateStartingAngle(void);
void ReadAccelerometer(void);
void ReadGyroscope(void);
void ReadMagnetometer(void);
void CalibrateMagnetometer(void);
void SendToUART(uint8_t *pbuf);

/* Function Bodies */
/* Initializations function and I2C protocol functions*/
/**
 * @brief Clock_init
 * 
 * Function that intializes the clock frequency and the source of clock for the STM32F030R8 microncontroller.
 *
 */
void Clock_Init(void){
	/* Use PLL as Clock Source and Mul by 12 */
	RCC->CR &= ~(RCC_CR_PLLON);						/**< Turn off PLL */
	while(RCC->CR & RCC_CR_PLLRDY);						/**< Wait until PLL is ready */
	RCC->CFGR |= RCC_CFGR_PLLMUL12;						/**< Multiply PLL value by 12. Must not exceed 48 Mhz (Max frequency)! */
	RCC->CR |= RCC_CR_PLLON;						/**< Turn on PLL */
	while(!(RCC->CR & RCC_CR_PLLRDY));					/**< Wait until PLL is ready in order to avoid messing stuff up */
	
	FLASH->ACR |= FLASH_ACR_LATENCY;					/**< Add ONE wait state for the flash acces time because 24 Mhz <= SYSCLOCK <= 48MHz */
	RCC->CFGR &= ~(RCC_CFGR_PPRE);						/**< Clear PCLK prescaler settings for safety */
	RCC->CFGR |= RCC_CFGR_PPRE_DIV16;					/**< HCLK divided by 16 */
	RCC->CFGR &= ~(RCC_CFGR_HPRE);						/**< Clear HCLK prescaler settings */
	while(!(RCC->CR & RCC_CR_PLLRDY));					/**< Wait until PLL is ready */
	RCC->CFGR &= ~(RCC_CFGR_SW);						/**< Clear System clcok switch settings */			
	RCC->CFGR |= RCC_CFGR_SW_PLL;						/**< Select PLL as source */
	RCC->CFGR &= ~(RCC_CFGR_PPRE);						/**< Clear preslacer settings set before for safety */	
}

/**
 * @brief GPIO_Init
 * 
 * Function that initializes the GPIO pins.
 *
 */
void GPIO_Init(void) {
	/* Enable clock for GPIOs */
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOFEN;

	GPIOA->MODER |= GPIO_MODER_MODER5_0; 					/**< Output mode (01) for bit 5 */
}

/**
 * @brief TIMER3_Init
 * 
 * Function that initializes the TIMER3 peripheral with it's frequency and up count max.
 *
 */
void TIMER3_Init(void) {
	/* Configuring Timer (TIM3) */
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->SR = 0;								/**< Resetting the SR register of TIM3 */
	TIM3->PSC = (uint16_t)(24000-1);					/**< SYSCLK/(PSC+1) = TIM3_frequency */
	TIM3->ARR = (uint16_t)1999;						/**< Counts to 1999 and then goes to 0. */
	
	/* In case I need interrupts */
	//TIM3->DIER |= (1 << 0);
	//NVIC_SetPriority(TIM3_IRQn, 2); 					/**< Priority level 2 */
	//NVIC_EnableIRQ(TIM3_IRQn);
}

/**
 * @brief USART2_Init
 * 
 * Function that initializes UART for the STM32F030R8 microcontroller to be used to transfer data to the PC in order
 * to visualize orientation. 
 *
 */
void USART2_Init(void) {
	/* Configuring UART (USART2) */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;																						/**< Enable clock for the USART2 */		
	
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL2 & 0x100) | (GPIO_AFRL_AFSEL3 & 0x1000);	
	GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; 		/**< Alternate function for pins 2 and 3 */ 
	
	USART2->CR1 &= ~(USART_CR1_UE);																									/**< Disable UART */ 
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;																			/**< Enable transmit and receive */ 
	
	USART2->BRR = (uint32_t)417;																										/**< 48MHz/115200 = 416.66 -> 417 */  
	
	USART2->CR1 |= USART_CR1_UE;																										/**< Enable UART */ 
}

/**
 * @brief I2C1_Init
 * 
 * Function that initializes I2C for the STM32F030R8 microcontroller to be used to communicate with the MPU-9250 sensor.
 *
 */
void I2C1_Init(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;																							/**< Enable clock for the GPIOB (Pins for I2C1) */ 
	
	GPIOB->AFR[1] |= (GPIO_AFRH_AFSEL8 & 0x01) | (GPIO_AFRH_AFSEL9 & 0x10);					
	GPIOB->MODER |= GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_1;										/**< Alternate function for the pins 8 and 9 */ 
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR8_1 | (GPIO_OSPEEDR_OSPEEDR8_1 >> 1)| GPIO_OSPEEDER_OSPEEDR9_1 | (GPIO_OSPEEDR_OSPEEDR9_1 >> 1);	/**< High speed */ 
	GPIOB->OTYPER |= GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9;											/**< Open drain type */ 
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR9_0;
	
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;																							/**< Enable clock for I2C1 */ 
	
	I2C1->CR1 &= ~(I2C_CR1_PE);																											/**< Disable I2C1 peripheral */ 
	I2C1->TIMINGR = 0x2000090E & 0xF0FFFFFFU;																				/**< Adjust timing */ 
	I2C1->CR1 |= I2C_CR1_PE;																												/**< Enable I2C1 peripheral */ 
}

/**
 * @brief I2C_Read
 * 
 * Function that performs the read cycle over I2C protocol to the sensor MPU-9250.
 *
 */
void I2C_Read(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *pdata) {
	/* Wait in case of busy */
	while(I2C1->ISR & I2C_ISR_BUSY);
	
	/* Sending Device Address */
	I2C1->CR2 &= ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_NBYTES | I2C_CR2_START | 
									I2C_CR2_STOP | I2C_CR2_SADD | I2C_CR2_RD_WRN );
	I2C1->CR2 |= (Slave_Address & I2C_CR2_SADD);
	I2C1->CR2 |= ((0x01 << 16U) & I2C_CR2_NBYTES);
	/* Sending start bit */
	I2C1->CR2 |= I2C_CR2_START;
	
	/* Waiting TXIS to be Set */
	while(!(I2C1->ISR & I2C_ISR_TXIS));
	
	/* Sending Register Address */
	I2C1->TXDR = Register_Address;
	
	/* Waiting TC to be Set */
	while(!(I2C1->ISR & I2C_ISR_TC));
	
	/* Sending Device Address and Restart */
	I2C1->CR2 &= ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_START | I2C_CR2_STOP | I2C_CR2_RD_WRN );
	I2C1->CR2 |= I2C_CR2_AUTOEND;
	I2C1->CR2 |= I2C_CR2_RD_WRN;
	I2C1->CR2 |= I2C_CR2_START;
	
	/* Waiting RXNE to be Set */
	while(!(I2C1->ISR & I2C_ISR_RXNE));
	
	/* Reading data from the receive register */
	*pdata = I2C1->RXDR;
	
	/* Waiting for STOPF to detect STOP bit */
	while(!(I2C1->ISR & I2C_ISR_STOPF));
	
	/* Clear STOP bit */
	I2C1->ICR |= I2C_ICR_STOPCF;
	
	/* Clear configuration */
	I2C1->CR2 &= ~(I2C_CR2_SADD|I2C_CR2_HEAD10R|I2C_CR2_NBYTES|I2C_CR2_RELOAD|I2C_CR2_RD_WRN);
}

/**
 * @brief I2C_Write
 * 
 * Function that performs the write cycle over I2C protocol to the sensor MPU-9250.
 *
 */
void I2C_Write(uint8_t Slave_Address, uint8_t Register_Address, uint8_t *pdata) {
	/* Wait in case of busy */
	while(I2C1->ISR & I2C_ISR_BUSY);
	
	/* Sending Device Address */
	I2C1->CR2 &= ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD | I2C_CR2_NBYTES | I2C_CR2_START | 
								I2C_CR2_STOP | I2C_CR2_SADD | I2C_CR2_RD_WRN );
	I2C1->CR2 |= (Slave_Address & I2C_CR2_SADD);
	I2C1->CR2 |= ((0x01 << 16U) & I2C_CR2_NBYTES);
	I2C1->CR2 |= I2C_CR2_RELOAD;
	I2C1->CR2 |= I2C_CR2_START;
	
	/* Wait until TXIS is Set */
	while(!(I2C1->ISR & I2C_ISR_TXIS));
	
	/* Send Memory Address */
	I2C1->TXDR = Register_Address;
	
	/* Wait until TCR is Set */
	while(!(I2C1->ISR & I2C_ISR_TCR));
	
	I2C1->CR2 &= ~(I2C_CR2_AUTOEND | I2C_CR2_RELOAD);
	I2C1->CR2 |= I2C_CR2_AUTOEND;
	
	/* Wait until TXIS is Set */
	while(!(I2C1->ISR & I2C_ISR_TXIS));
	I2C1->TXDR = *pdata;
	
	/* Wait until STOPF is Set and clear it */
	while(!(I2C1->ISR & I2C_ISR_STOPF));
	I2C1->ICR |= I2C_ICR_STOPCF;
	
	/* Clear configuration */
	I2C1->CR2 &= ~(I2C_CR2_SADD|I2C_CR2_HEAD10R|I2C_CR2_NBYTES|I2C_CR2_RELOAD|I2C_CR2_RD_WRN);
}


/* Sensor and data functions */
/**
 * @brief KalmanFilterXYZ
 * 
 * Function takes the structure of the Kalman filter of the appropriate axis, the position calculated by the 
 * accelerometer data, velocity calculated from the gyroscope data and the interval of time that had passed
 * between the AccelZStartingments. It is a void function because all the calculations are stored back into the Kalman
 * filter structure to be used again in the next calculation. 
 *
 */
void KalmanFilterXYZ(struct KalmanFilter *Kalman, float GyroRate, float AccelAngle, float dt) {
	/* State prediction. This step uses the previous value to predict the next value */
	Kalman->Xk[0] = Kalman->Xk[0] + dt * GyroRate - dt * Kalman->Xk[1];
	
	/* Eliminating the drift from the gyroscope velocity readings */
	Kalman->NoDriftGyroRate = GyroRate - Kalman->Xk[1];
	
	/* Error covariance prediction. This step uses the prevoious error covariance to determine the current */
	Kalman->P[0][0] = Kalman->P[0][0] - dt * Kalman->P[1][0] -dt * (Kalman->P[0][1] -dt * Kalman->P[1][1])+ Kalman->Q[0];
	Kalman->P[0][1] = Kalman->P[0][1] - dt * Kalman->P[1][1];
	Kalman->P[1][0] = Kalman->P[1][0] - dt * Kalman->P[1][1];
	Kalman->P[1][1] = Kalman->P[1][1] + Kalman->Q[1];
	
	/* Kalman gain computation. It is later used as weight to determine the estimation of the position */
	float K[2];
  K[0] = Kalman->P[0][0] / ((float)(Kalman->P[0][0] + Kalman->R));
  K[1] = Kalman->P[1][0] / ((float)(Kalman->P[0][0] + Kalman->R));
	
	/* Estimate computation. In this step, the algorithm determines the difference between the actual reading and 
	the estimated readings */
	float difference = AccelAngle - Kalman->Xk[0];
	Kalman->Xk[0] += K[0] * difference;
  Kalman->Xk[1] += K[1] * difference;
	
	/* Error covariance computation. Larger Pk signifies a bigger error in estimation */
	float P0 = Kalman->P[0][0];
	float P1 = Kalman->P[0][1];
	
	Kalman->P[0][0] -= K[0] * P0;
	Kalman->P[0][1] -= K[0] * P1;
	Kalman->P[1][0] -= K[1] * P0;
	Kalman->P[1][1] -= K[1] * P1;
}

/**
 * @brief InitKalmanXYZ
 * 
 * Function initializes the Kalman filter structures with appropriate values. Some values were chosen
 * based on experiments and judgement. They were given subjectively.
 *
 */
void InitKalmanXYZ(void){
	/* Kalman filter structure for the X-axis */
	KalmanX.Q[0] = 0.001;
	KalmanX.Q[1] = 0.003;
	KalmanX.R = 0.003;
	KalmanX.Xk[0] = 0.0;
	KalmanX.Xk[1] = 0.0;
	KalmanX.NoDriftGyroRate = 0.0;
	KalmanX.P[0][0] = 0.0;
	KalmanX.P[0][1] = 0.0;
	KalmanX.P[1][0] = 0.0;
	KalmanX.P[1][1] = 0.0;
	
	/* Kalman filter structure for the Y-axis */
	KalmanY.Q[0] = 0.001;
	KalmanY.Q[1] = 0.003;
	KalmanY.R = 0.003;
	KalmanY.Xk[0] = 0.0;
	KalmanY.Xk[1] = 0.0;
	KalmanY.NoDriftGyroRate = 0.0;
	KalmanY.P[0][0] = 0.0;
	KalmanY.P[0][1] = 0.0;
	KalmanY.P[1][0] = 0.0;
	KalmanY.P[1][1] = 0.0;
	
	/* Kalman filter structure for the Z-axis */
	KalmanZ.Q[0] = 0.001;
	KalmanZ.Q[1] = 0.003;
	KalmanZ.R = 0.003;
	KalmanZ.Xk[0] = 0.0;
	KalmanZ.Xk[1] = 0.0;
	KalmanZ.NoDriftGyroRate = 0.0;
	KalmanZ.P[0][0] = 0.0;
	KalmanZ.P[0][1] = 0.0;
	KalmanZ.P[1][0] = 0.0;
	KalmanZ.P[1][1] = 0.0;
}

/**
 * @brief WakeUpSensor
 * 
 * Function checks the connection to the MPU-9250 sensor by reading the WHO_AM_I register and then writes to 
 * power management register to turn off sleep mode.
 *
 */
void WakeUpSensor(void) {
	/* Reading the WHO_AM_I register of the Accel-Gyro Slave */
	I2C_Read(AccelGyroAddress, WHO_AM_I, data);
	if(data[0] != 0x68) {
		strcpy((char*)buf, "Invalid WHO_AM_I Register value!\r\n");
		SendToUART(buf);
	}
	
	data[0] = 0x00;
	/* Writing to the PWR_MGMT_1 register to turn off the sleep mode */
	I2C_Write(AccelGyroAddress, PWR_MGMT_1, data);
}

/**
 * @brief GetMagnetometerAdjustment
 * 
 * Function checks the connection to the magnetometer sensor, reads the magnetometer adjustment values
 * and sets the appropriate mode of operation for the sensor to be read later. It uses the timer 3 peripheral to 
 * time the waiting necessary to switch between modes of operation. 
 *
 */
void GetMagnetometerAdjustment(void) {
	/* Reading the WIA register (ID of Magnetometer) */
	I2C_Read(MagnetometerAddress, WIA, data);
	if (data[0] != 0x48) {
		strcpy((char*)buf, "Incorrect WIA ID!\r\n");
		SendToUART(buf);
	}
	
	data[0] = 0x00;
	/* Writing to the CNTL1_AD register (POWER-DOWN MODE)*/
	I2C_Write(MagnetometerAddress, CNTL1_AD, data);
	
	/* Starting the timer 3 peripheral and counting for 500 ms to change the mode of operation */
	TIM3->CNT = 0x00;
	TIM3->CR1 |= TIM_CR1_CEN;			/**< Enabling the timer 3 peripheral */					
	while(1){
		if(TIM3->CNT > 1000)
			break;
	}
	TIM3->CR1 &= ~TIM_CR1_CEN;			/**< Disabling the timer 3 peripheral */	
	TIM3->CNT = 0x00;
	
	data[0] = 0x0F;
	/* Writing to the CNTL1_AD register (FUSE-ROM-MODE)*/
	I2C_Write(MagnetometerAddress, CNTL1_AD, data);
	
	TIM3->CNT = 0x00;
	TIM3->CR1 |= TIM_CR1_CEN;									
	while(1){
		if(TIM3->CNT > 1000)
			break;
	}
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM3->CNT = 0x00;
	
	/* Reading the adjustment values of the magnetometer */
	for(uint8_t i = 0; i < 3; i++) 
	{
		I2C_Read(MagnetometerAddress, (ASAX_REG + i), (data + i));
	}
	
	ASAX = data[0];
	ASAY = data[1];
	ASAZ = data[2];
	
	data[0] = 0x00;
	/* Writing to the CNTL1_AD register (POWER-DOWN MODE)*/
	I2C_Write(MagnetometerAddress, CNTL1_AD, data);
	
	TIM3->CNT = 0x00;
	TIM3->CR1 |= TIM_CR1_CEN;									
	while(1){
		if(TIM3->CNT > 1000)
			break;
	}
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM3->CNT = 0x00;
	
	data[0] = 0x02;
	/* Writing to the CNTL1_AD register (Continuous Mode 2)*/
	I2C_Write(MagnetometerAddress, CNTL1_AD, data);
	
	TIM3->CNT = 0x00;
	TIM3->CR1 |= TIM_CR1_CEN;									
	while(1){
		if(TIM3->CNT > 1000)
			break;
	}
	TIM3->CR1 &= ~TIM_CR1_CEN;
	TIM3->CNT = 0x00;
	
	//data[0] = 0xFF;
	/* Checking value of CNTL1_AD */
	//I2C_Read(MagnetometerAddress, CNTL1_AD, data);
}

/**
 * @brief SwitchSlaveDevice
 * 
 * Function switches between the AccelGyro sensor and Magnetometer sensor based on the command it receives. 
 *
 */
void SwitchSlaveDevice(uint8_t command) {
	data[0] = command;
	/* Writing to the INT_BYPASS_CONFIG_AD register */
	I2C_Write(AccelGyroAddress, INT_BYPASS_CONFIG_AD, data);
}

/**
 * @brief CalculateStartingAngle
 * 
 * Function that calculates the static noise error of the accelerometer and gyroscope sensor by doing 100 reading of each
 * while the sensor sits still in order to find the average difference of the readings and then subtract it from the actual
 * readings.
 *
 */
void CalculateStartingAngle(void) {
/* ------------------------------------------ Calculating the ERROR  ---------------------------------------- */
	uint8_t counter = 0;
	float AccelXStarting = 0.0;
	float AccelYStarting = 0.0;
	float AccelZStarting = 0.0;
	
	while (counter < 100)				/**< Read 100 times */	
	{
		/* --------------------------------------- ACCELEROMETER ERROR --------------------------------------------- */
		for(uint8_t i = 0; i < 6; i++) 
		{
			I2C_Read(AccelGyroAddress, (ACCEL_XOUT_H + i), (data + i));
		}
		X_AXIS = (data[0] << 8) | data[1];
		Y_AXIS = (data[2] << 8) | data[3];
		Z_AXIS = (data[4] << 8) | data[5];
		
		AccelXStarting += X_AXIS/16384.0;
		AccelYStarting += Y_AXIS/16384.0;
		AccelZStarting += Z_AXIS/16384.0;
		
		counter++;
	}
	AccelXStarting = AccelXStarting/100.0;
	AccelYStarting = AccelYStarting/100.0;
	AccelZStarting = AccelZStarting/100.0;	
	
	float AccelAngleX = atan(AccelYStarting / sqrt(pow(AccelXStarting, 2) + pow(AccelZStarting, 2)))*180/PI;			/**< Roll y, z*/
	float AccelAngleY = atan(-1*(AccelXStarting) / sqrt(pow(AccelYStarting, 2) + pow(AccelZStarting, 2)))*180/PI;			/**< Pitch x, z */
	
	roll = AccelAngleX;
	pitch = AccelAngleY;
}


/**
 * @brief ReadAccelerometer
 * 
 * Function that reads the accelerometer data by starting from the X-axis high byte and itterating through 5 more registers
 * which represent the rest of the data registers in order X-Y-Z.
 *
 */
void ReadAccelerometer(void) {
	for(uint8_t i = 0; i < 6; i++) 
	{
		I2C_Read(AccelGyroAddress, (ACCEL_XOUT_H + i), (data + i));
	}
}

/**
 * @brief ReadGyroscope
 * 
 * Function that reads the gyroscope data by starting from the X-axis high byte and itterating through 5 more registers
 * which represent the rest of the data registers in order X-Y-Z.
 *
 */
void ReadGyroscope(void) {
	for(uint8_t i = 0; i < 6; i++) 
	{
		I2C_Read(AccelGyroAddress, (GYRO_XOUT_H + i), (data + i));
	}
}

/**
 * @brief ReadMagnetometer
 * 
 * Function that reads the magnetometer data by starting from the X-axis low byte and itterating through 6 more registers
 * which represent the rest of the data registers in order X-Y-Z. The 7th register is the STATUS2 register which need to be read 
 * to complete the read cycle as per datasheet of the MPU-9250 sensor.
 *
 */
void ReadMagnetometer(void) {
	/* Don't want to check the DataReady register because the entire system is slowed then */
	/*data[0] = 0x00;
	while((data[0] & 0x01) == 0) {
			I2C_Read(MagnetometerAddress, STATUS1, data);
	}*/
	
	for(uint8_t i = 0; i < 7; i++) 
		{
			I2C_Read(MagnetometerAddress, (MAGN_XOUT_L + i), (data + i));
		}
}

/**
 * @brief CalibrateMagnetometer
 * 
 * Function that calibrated the magnetometer by doing 60000 readings from the sensor that is moved in
 * all direction in order to find the maximum and minimums for all axises and adjust for hard-iron distortion.
 * The values have already been calculated and stored inside an array because the process takes too long and is
 * unnecessary to do at every start of  the program. It was done in one of the test runs and the values have been found.
 *
 */
void CalibrateMagnetometer(void) {
	uint32_t i = 0;
	uint32_t calibration_count = 60000;
	float MAGX_MAX = 0.0;
	float MAGY_MAX = 0.0;
	float MAGZ_MAX = 0.0;
	float MAGX_MIN = 0.0;
	float MAGY_MIN = 0.0;
	float MAGZ_MIN = 0.0;
	
	/* Turn on the LED to singify the start of the calibration */
	GPIOA->ODR ^= GPIO_ODR_5;
	
	for(i = 0; i < calibration_count; i++) {				/**< Do 60000 readings */	
    		ReadMagnetometer(); 
		
		/* Find the appropriate values */
		X_AXIS = (data[1] << 8) | data[0];
		Y_AXIS = (data[3] << 8) | data[2];
		Z_AXIS = (data[5] << 8) | data[4];
		
		X_AXIS = (X_AXIS << 2);
		Y_AXIS = (Y_AXIS << 2);
		Z_AXIS = (Z_AXIS << 2);
		
		X_OUTPUT = ((float)(X_AXIS))/4.0;
		Y_OUTPUT = ((float)(Y_AXIS))/4.0;
		Z_OUTPUT = ((float)(Z_AXIS))/4.0;

		/* Finding the maximum and minimum of each axis */
    		if(X_OUTPUT > MAGX_MAX) 
			MAGX_MAX = X_OUTPUT;
		if(X_OUTPUT < MAGX_MIN) 
			MAGX_MIN = X_OUTPUT;
		if(Y_OUTPUT > MAGY_MAX) 
			MAGY_MAX = Y_OUTPUT;
		if(Y_OUTPUT < MAGY_MIN) 
			MAGY_MIN = Y_OUTPUT;
		if(Z_OUTPUT > MAGZ_MAX) 
			MAGZ_MAX = Z_OUTPUT;
		if(Z_OUTPUT < MAGZ_MIN) 
			MAGZ_MIN = Z_OUTPUT;
	}
	
	/* Store the magnetic offsets from the hard iron distortion in order to subtract from the actual readings */
	MagneticOffset[0] = ((float)MAGX_MAX + (float)MAGX_MIN)/2;
	MagneticOffset[1] = ((float)MAGY_MAX + (float)MAGY_MIN)/2;
	MagneticOffset[2] = ((float)MAGZ_MAX + (float)MAGZ_MIN)/2;
	
	/* Turn off the LED to singify the end of the calibration */
	GPIOA->ODR ^= GPIO_ODR_5;
}

/**
 * @brief SendToUART
 * 
 * Function that sends the string data over UART.
 *
 */
void SendToUART(uint8_t *pbuf) {
		uint32_t size = strlen((char*) pbuf);
		for (uint32_t i=0; i<size; i++){
			USART2->TDR = pbuf[i];					/**< Send through the Transmit Data Register */	
			while(!(USART2->ISR & USART_ISR_TC));			/**< Check transmission complete measure in order to continue */	
		}
}

/**
 * @brief main
 * 
 * Function that does all the initializations of the peripherals, the microcontroller and the sensor. It features a
 * forever loop inside of which the data from the accelerometer, gyroscope and magnetometer sensor is acquired and refined 
 * in order to represent the actual readings suitable for real-life orientation determination.
 *
 */
int main(void)
{
	/* Initialize clock to max frequency of 48 MHz */
	Clock_Init();
	
	/* Initialize the general input output pins */
	GPIO_Init();
	
	/* Initialize the timer 3 */
	TIMER3_Init();
	
	/* Initialize UART */
	USART2_Init();
	
	/* Initialize I2C1 */
	I2C1_Init();
	
	/* Initiliaze Kalman filters with starting values */
	InitKalmanXYZ();
	
	/* Initialize sensor by writing in the power management register */
	WakeUpSensor();
	
	/* Switch to magnetometer slave in order to get the adjustment values */
	SwitchSlaveDevice(MAGNETOMETER_SLAVE);
	
	GetMagnetometerAdjustment();
	
	/* Calibrate the adjustment values to show true readings */
	float calibrationX = (((ASAX-128)*0.5)/128.0+1.0);
	float calibrationY = (((ASAY-128)*0.5)/128.0+1.0);
	float calibrationZ = (((ASAZ-128)*0.5)/128.0+1.0); 
	
	/* Function which calibrated the magnetometer by reading 1000 times from it while it's being moved
	in all directions to find the appropriate offset. Not necessary to include as the values have already been
  found in one of the test runs	*/
	//CalibrateMagnetometer();
	
	MagneticOffset[0] = MagneticOffset[0]*MagneticResolution*calibrationX;
	MagneticOffset[1] = MagneticOffset[1]*MagneticResolution*calibrationY;
	MagneticOffset[2] = MagneticOffset[2]*MagneticResolution*calibrationZ;
	
	/* Continue with the accel+gyro sensor */
	SwitchSlaveDevice(ACCEL_GYRO_SLAVE);
	
	/* Calculate the sensor noise by reading from the sensor while it's sitting still and find the deviations */
	CalculateStartingAngle();
	
	/* Variables used to hold the gyroscope angles */
	float GyroAngleX = 0.0;
	float GyroAngleY = 0.0;
	float GyroAngleZ = 0.0;
	
	float yaw_tempf = 0.0;										/**< Used to reduce the noise of the magnetometer yaw calculation */
	float vrijeme = 0.0;
	
	float repeat = 0.0;
	int measure = 0;
	float yaw_starting = 0.0;
	
	float xdistance = 0.0;
	float ydistance = 0.0;
	float zdistance = 0.0;
	
	/* Enabling the timer 3 to count */
	TIM3->CNT = 0x00;
	TIM3->CR1 |= TIM_CR1_CEN;									/**< Writing TIM_CR1_CEN to CR1 register of the TIM3 enables the counting */
	
	
	/*                                        --FOREVER LOOP--                                                     */
  while (1)
  {
		/*                           --Reading the ACCELEROMETER--                                             */
		ReadAccelerometer();
		
		/* Storing the data inside the variables */
		X_AXIS = (data[0] << 8) | data[1];
		Y_AXIS = (data[2] << 8) | data[3];
		Z_AXIS = (data[4] << 8) | data[5];
		
		/* Adjusting for the noise and transforming the data to represent true values */
		X_OUTPUT = X_AXIS/16384.0;
		Y_OUTPUT = Y_AXIS/16384.0;
		Z_OUTPUT = Z_AXIS/16384.0;
		
		xaccel = X_OUTPUT;
		yaccel = Y_OUTPUT;
		zaccel = Z_OUTPUT;
		
		/* Calculating the roll and pitch values from the accelerometer data */
		//float AccelAngleY = atan2(X_OUTPUT, Z_OUTPUT)*180/PI;																				/**< Pitch */
		//float AccelAngleX = atan2(Y_OUTPUT, Z_OUTPUT)*180/PI;																				/**< Roll */
    float AccelAngleX = atan(Y_OUTPUT / sqrt(pow(X_OUTPUT, 2) + pow(Z_OUTPUT, 2)))*180/PI;					/**< Roll y, z*/
		float AccelAngleY = atan(-1*X_OUTPUT / sqrt(pow(Y_OUTPUT, 2) + pow(Z_OUTPUT, 2)))*180/PI;			/**< Pitch x, z */
		
		
		/*                             --Reading the GYROSCOPE--                                            */
		/* Getting the time it took for the data aquisition from the counter in order to integrate the gyroscope data */
		float current_time = (float)(TIM3->CNT)*TurnToMilisecond;   
		float dt = (current_time) / 1000.0;
		vrijeme += dt;
		TIM3->CNT = 0;													/**< Reseting the counter value */
		
		ReadGyroscope();
		
		X_AXIS = (data[0] << 8) | data[1];
		Y_AXIS = (data[2] << 8) | data[3];
		Z_AXIS = (data[4] << 8) | data[5];
		
		/* Adjusting for the noise and transforming the data to represent true values */
		float GyroRateX = X_AXIS/131.0;
		float GyroRateY = Y_AXIS/131.0;
		float GyroRateZ = Z_AXIS/131.0;
		
		/* Integrate the data from the gyroscope in order to turn the angular velocity to angular position */
		GyroAngleX += GyroRateX*dt;
		GyroAngleY += GyroRateY*dt;
		GyroAngleZ += GyroRateZ*dt;
		
		/*                           --Reading the Magnetometer--                                            */
		float MAGX = 0.0;
		float MAGY = 0.0;
		float MAGZ = 0.0;
		
		SwitchSlaveDevice(MAGNETOMETER_SLAVE);
		
		ReadMagnetometer();
		
		/* Reading the 14-bit data from the magnetometer slave */
		X_AXIS = (data[1] << 8) | data[0];
		Y_AXIS = (data[3] << 8) | data[2];
		Z_AXIS = (data[5] << 8) | data[4];
		
		X_AXIS = (X_AXIS << 2);
		Y_AXIS = (Y_AXIS << 2);
		Z_AXIS = (Z_AXIS << 2);
		
		X_OUTPUT = ((float)(X_AXIS))/4.0;
		Y_OUTPUT = ((float)(Y_AXIS))/4.0;
		Z_OUTPUT = ((float)(Z_AXIS))/4.0;
		
		/* Transform the data to represent the true readings by calibrating and subtracting offset */
		MAGX = X_OUTPUT * MagneticResolution * calibrationX - MagneticOffset[0];
		MAGY = Y_OUTPUT * MagneticResolution * calibrationY - MagneticOffset[1];
		MAGZ = -1*(Z_OUTPUT * MagneticResolution * calibrationZ - MagneticOffset[2]);					/**< Z-axis is in the reversed
																																													direction from the accel-gyro sensor */
		
		SwitchSlaveDevice(ACCEL_GYRO_SLAVE); 
		
		/*                           --Calculating PITCH, ROLL and YAW--                                 */
		
		/* Using Kalman filtering to obtain the non-noise-plagued data. It receives the position from the accelerometer calculated
		pitch and yaw values and the velocity it calculated from the gyroscope readings. The accelerometer data is plagued by noise
		while the gyroscope data is plagued by drift. The Kalman filter predicts the future readings based on the current and the 
		past ones and should eliminate any noise or drift present in the data */
		KalmanFilterXYZ(&KalmanX, GyroRateX, AccelAngleX, dt);
		KalmanFilterXYZ(&KalmanY, GyroRateY, AccelAngleY, dt);
		
		/* Determining roll and pitch from the gyroscope data with removed drift (Kalman filtering) */
		roll += KalmanX.NoDriftGyroRate*dt;
		pitch += KalmanY.NoDriftGyroRate*dt;
		
		/* Tilt compensation */
		/* Obtaining the yaw value from the magnetometer data based on the pitch and roll values and the read magnetometer data */
		float Hy = MAGY*cos(pitch/(180/PI)) + MAGZ*sin(pitch/(180/PI));
		float Hx = MAGY*sin(roll/(180/PI))*sin(pitch/(180/PI))+MAGX*cos(roll/(180/PI)) - MAGZ*sin(roll/(180/PI))*cos(pitch/(180/PI));
		
		/* Temporary yaw value which is really noise, obtained from the equation for the tilt compensation */
		float yaw_temp = atan2(Hy, Hx) * 180/PI;
		
		/* Using complementary filter to reduce the noise from the yaw value */
		yaw_tempf = 0.5*yaw_tempf + 0.5*yaw_temp;
		
		/* Using Kalman filter to reduce more noise and eliminate drift from the Z-axis gyroscope reading */
		KalmanFilterXYZ(&KalmanZ, GyroRateZ, yaw_tempf, dt);

		/* Possibilities of calculating the yaw angle. The uncommented one was the one that gave the best results */
		//yaw = 0;																				/**< 0 to see only roll and pitch */
		//yaw = GyroAngleZ;																/**< Only calculated by the gyroscope reading */
		//yaw = yaw_tempf;																/**< Using only the complementary filter */
		yaw += KalmanZ.NoDriftGyroRate*dt;								/**< Using complementary filter and Kalman filter */
		
		if(measure == 0) {
			if(repeat < 200){
				repeat += 1;
			}
			else {
				measure = 1;
				yaw_starting = yaw;
			}
		}
		
		//yaw = 0.81*GyroAngleZ + 0.19*yaw_tempf;					/**< Using the complementary filter with gyroscope data */
		//yaw_tempf2 = 0.7*yaw_tempf2 + 0.3*yaw;
		
	  	/* Only if certain threshold is exceeded, mesure distance drawn */
		if((xaccel*xaccel + yaccel*yaccel + zaccel*zaccel) > 1.25) {
			GPIOA->ODR |= GPIO_ODR_5;
			xdistance += xaccel*dt*dt;
			ydistance += yaccel*dt*dt;
			zdistance += zaccel*dt*dt;
		}
		else {
			GPIOA->ODR &= ~GPIO_ODR_5;
			xdistance = 0.0;
			ydistance = 0.0;
			zdistance = 0.0;
		}
		
		/*                                    --VISUALIZATION--                                          */
		/* Sending the data to be read inside the Processing Toolbox */
		if(measure == 1) {
			strcpy((char*)buf, "A,");
			SendToUART(buf);
			sprintf((char*)buf, "%f,", vrijeme);
			SendToUART(buf);
			sprintf((char*)buf, "%f,", roll);
			SendToUART(buf);
			sprintf((char*)buf, "%f,", pitch);
			SendToUART(buf);
			sprintf((char*)buf, "%f,", (yaw - yaw_starting));
			SendToUART(buf);
			sprintf((char*)buf, "%f,", xdistance*100);
			SendToUART(buf);
			sprintf((char*)buf, "%f,", ydistance*100);
			SendToUART(buf);
			sprintf((char*)buf, "%f\r\n", zdistance*100);
			SendToUART(buf);
		}
		
		
		/*if(measure == 1) {
			sprintf((char*)buf, "%f", roll);
			SendToUART(buf);s
			strcpy((char*)buf, "/");
			SendToUART(buf);
			sprintf((char*)buf, "%f", pitch);
			SendToUART(buf);
			strcpy((char*)buf, "/");
			SendToUART(buf);
			sprintf((char*)buf, "%f\n", (yaw - yaw_starting));
			SendToUART(buf);
		}*/
  }
}
