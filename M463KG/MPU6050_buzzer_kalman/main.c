#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "NuMicro.h"

#define MPU6050_SLAVE_ADDR      0x68
#define GYRO_XOUT 				0x43
#define ACCEL_XOUT_H 			0x3B
#define WHO_AM_I				0x75
#define PWR_MGMT_1				0x6B

#define M_PI acos(-1)

static volatile uint32_t g_u32RTCTInt = 0;

float Q_angle = 0.1; 
float Q_bias = 0.3; 
float R_measure = 0.15; 

float angle = 0;
float bias = 0; 
float P[2][2] = {{0, 0}, {0, 0}};

void RTC_IRQHandler(void)
{
    if(RTC_GET_TICK_INT_FLAG() == 1)
    {
        RTC_CLEAR_TICK_INT_FLAG();

        g_u32RTCTInt = 1;
    }
}

void SYS_Init(void)
{
	SYS_UnlockReg();

    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LXTEN_Msk|CLK_PWRCTL_HXTEN_Msk|CLK_PWRCTL_HIRC48MEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_LXTSTB_Msk|CLK_STATUS_HXTSTB_Msk|CLK_STATUS_HIRC48MSTB_Msk);
    CLK_DisablePLL();
    CLK->PLLCTL = (CLK->PLLCTL & ~(0x008FFFFFUL)) | 0x0008421EUL;
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
    CLK->PLLCTL |= CLK_PLLFNCTL1_PD_Msk;
    CLK->PLLFNCTL0 = (CLK->PLLFNCTL0 & ~(0x0FFFFFFFUL)) | 0x0000842BUL;
    CLK->PLLCTL |= CLK_PLLFNCTL1_PD_Msk;
    CLK->PLLFNCTL1 = (CLK->PLLFNCTL1 & ~(0xF8000000UL)) | 0x80000000UL;
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV2 | CLK_PCLKDIV_PCLK1DIV2);
	
	CLK_EnableModuleClock(EPWM0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);
		
	CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, MODULE_NoMsk);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, MODULE_NoMsk);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048, MODULE_NoMsk);

    SystemCoreClockUpdate();
		
	SYS->GPB_MFP0 = SYS_GPB_MFP0_PB3MFP_EPWM0_CH2 | SYS_GPB_MFP0_PB2MFP_EPWM0_CH3;
	SYS->GPB_MFP3 = SYS_GPB_MFP3_PB13MFP_UART0_TXD | SYS_GPB_MFP3_PB12MFP_UART0_RXD;
    SYS->GPC_MFP0 = SYS_GPC_MFP0_PC1MFP_I2C0_SCL | SYS_GPC_MFP0_PC0MFP_I2C0_SDA;
    SYS->GPF_MFP0 = SYS_GPF_MFP0_PF1MFP_ICE_CLK | SYS_GPF_MFP0_PF0MFP_ICE_DAT;

	CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    CLK_EnableModuleClock(RTC_MODULE);

    SYS_LockReg();
}

double kalman_filter(double newAngle, double newRate, double dt){
	double rate = newRate - bias;
	angle += dt * rate;
		
	P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    double S = P[0][0] + R_measure;
    double K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    double y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    double P00_temp = P[0][0];
    double P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

uint8_t getID(){
	I2C_WriteByte(I2C0, MPU6050_SLAVE_ADDR, WHO_AM_I);
	CLK_SysTickDelay(100000);
	
	return I2C_ReadByte(I2C0, MPU6050_SLAVE_ADDR);
}

void *MPU6050_ReadAcc(double milis){
	uint8_t acc_array[6];
	static double output[3];
	uint16_t combineX=0, combineY=0, combineZ=0;
	float x=0, y=0, z=0;
	uint32_t timer;
	
	I2C_WriteByte(I2C0, MPU6050_SLAVE_ADDR, ACCEL_XOUT_H);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, MPU6050_SLAVE_ADDR, acc_array, 6);
	combineX = (int16_t)(acc_array[0] << 8 | acc_array[1]);
  	combineY = (int16_t)(acc_array[2] << 8 | acc_array[3]);
  	combineZ = (int16_t)(acc_array[4] << 8 | acc_array[5]);
	
	x = (float)combineX;
	y = (float)combineY;
	z = (float)combineZ;
	
	double dt = (double)(milis - timer) / 1000;
	timer = milis;
	
	double filteredAccX = kalman_filter(x, x, dt);
	double filteredAccY = kalman_filter(y, y, dt);
	double filteredAccZ = kalman_filter(z, z, dt);
	
	output[0] = filteredAccX;
	output[1] = filteredAccY;
	output[2] = filteredAccZ;
	
	return output;
}

double *MPU6050_ReadGyro(double milis){
	uint8_t gyro_array[6];
	static double output[3];
	uint16_t combineX=0, combineY=0, combineZ=0;
	float x=0, y=0, z=0;
	uint32_t timer;
	
	I2C_WriteByte(I2C0, MPU6050_SLAVE_ADDR, GYRO_XOUT);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, MPU6050_SLAVE_ADDR, gyro_array, 6);
	combineX = (int16_t)(gyro_array[0] << 8 | gyro_array[1]);
  	combineY = (int16_t)(gyro_array[2] << 8 | gyro_array[3]);
  	combineZ = (int16_t)(gyro_array[4] << 8 | gyro_array[5]);
	
	float gyroScaleFactor = 131.0; 

  	x = (float)combineX / gyroScaleFactor;
  	y = (float)combineY / gyroScaleFactor;
  	z = (float)combineZ / gyroScaleFactor;
	
	double dt = (double)(milis - timer) / 1000;
	timer = milis;
	
	double filteredGyroX = kalman_filter(x, x, dt);
	double filteredGyroY = kalman_filter(y, y, dt);
	double filteredGyroZ = kalman_filter(z, z, dt);
	
	output[0] = filteredGyroX;
	output[1] = filteredGyroY;
	output[2] = filteredGyroZ;
	
	return output;
}

double pitchAngle(double milis){
	double pitch;
	
	double *acc = MPU6050_ReadAcc(milis);
	pitch = -atan(acc[0]/sqrt(acc[1]*acc[1]+acc[2]*acc[2]))*1/(M_PI/180);
	
	return pitch;
}

double rollAngle(double milis){
	double roll = 0;
	
	double *acc = MPU6050_ReadAcc(milis);
	roll = atan(acc[0]/sqrt(acc[1]*acc[1]+acc[2]*acc[2]))*1/(M_PI/180);
	
	return roll;
}

double verticalVelocity(double milis){
	double *acc = MPU6050_ReadAcc(milis);
	double pitch = pitchAngle(milis);
	double roll = rollAngle(milis);
	
	double AccZ_Inertial = 0;
	double vertical_v = 0;
	
	AccZ_Inertial = -acc[0]*sin(pitch)+acc[1]*sin(roll)*cos(pitch)+acc[2]*cos(roll)*cos(pitch);
	vertical_v = vertical_v + AccZ_Inertial * 0.004;

	return vertical_v;
}

void BuzzerMod1(){ 
	EPWM_ConfigOutputChannel(EPWM0, 3, 10000, 100);
	EPWM_EnableOutput(EPWM0, EPWM_CH_3_MASK);		
	EPWM_Start(EPWM0, EPWM_CH_3_MASK);
				
	CLK_SysTickDelay(1000000);
				
	EPWM_ConfigOutputChannel(EPWM0, 3, 10000, 0);
	EPWM_EnableOutput(EPWM0, EPWM_CH_3_MASK);		 
	EPWM_Start(EPWM0, EPWM_CH_3_MASK);
				
	CLK_SysTickDelay(1000000);
				
	EPWM_ConfigOutputChannel(EPWM0, 3, 10000, 100);
	EPWM_EnableOutput(EPWM0, EPWM_CH_3_MASK);		
	EPWM_Start(EPWM0, EPWM_CH_3_MASK);
				
	CLK_SysTickDelay(1000000);
				
	EPWM_ConfigOutputChannel(EPWM0, 3, 10000, 0);
	EPWM_EnableOutput(EPWM0, EPWM_CH_3_MASK);		
	EPWM_Start(EPWM0, EPWM_CH_3_MASK);
}

void BuzzerMod2(){  
	EPWM_ConfigOutputChannel(EPWM0, 3, 10000, 100);
	EPWM_EnableOutput(EPWM0, EPWM_CH_3_MASK);		
	EPWM_Start(EPWM0, EPWM_CH_3_MASK);
				
	CLK_SysTickDelay(1000000);
				
	EPWM_ConfigOutputChannel(EPWM0, 3, 10000, 0);
	EPWM_EnableOutput(EPWM0, EPWM_CH_3_MASK);		
	EPWM_Start(EPWM0, EPWM_CH_3_MASK);
}

int main(){
	S_RTC_TIME_DATA_T sInitTime, sReadRTC;
    uint32_t u32Sec;
    uint8_t u8IsNewDateTime = 0;
	
	SYS_Init();
	
	SYS_ResetModule(UART0_RST);
    UART_Open(UART0, 115200); 
	
	I2C_Open(I2C0, 100000); 
		
	printf("PROGRAM BASLADI\r\n");
	
	RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);
    NVIC_EnableIRQ(RTC_IRQn);
	
	if(RTC_Open(&sInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
				return -1;
    }
		
	RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    RTC_SetTickPeriod(RTC_TICK_1_SEC);
		
	u32Sec = 0;
    g_u32RTCTInt = 0;
	double milis = 0;
		
	GPIO_SetMode(PB, BIT2, GPIO_MODE_OUTPUT);
		
	if(getID() == 104){
		uint8_t array[2] = {PWR_MGMT_1, 0x00};
				
		I2C_WriteByte(I2C0, MPU6050_SLAVE_ADDR, PWR_MGMT_1);
		CLK_SysTickDelay(100000);
				
		I2C_WriteMultiBytes(I2C0, MPU6050_SLAVE_ADDR, array, 2);
		I2C_WriteByte(I2C0, MPU6050_SLAVE_ADDR, PWR_MGMT_1);
				
		BuzzerMod1();
	}
	else{
		BuzzerMod2();
	}
		
    while(1){ 
		if(g_u32RTCTInt == 1)
        {
            g_u32RTCTInt = 0;

            RTC_GetDateAndTime(&sReadRTC);
			//printf("sec: %d\n", sReadRTC.u32Minute*60+sReadRTC.u32Second);
			milis = sReadRTC.u32Minute*60+sReadRTC.u32Second;
		}
				
		double *gyro = MPU6050_ReadGyro(milis);
		printf("%.2f", gyro[0]);
			
		double *acc = MPU6050_ReadAcc(milis);
		printf("Filtered: %.2f\n", acc[0]);
				
		double pitch = pitchAngle(milis);
		printf("pitch angle:%.2f\n", pitch);
				
			
      //CLK_SysTickDelay(300);
	}	
}