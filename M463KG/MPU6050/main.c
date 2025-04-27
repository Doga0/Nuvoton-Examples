#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define MPU6050_SLAVE_ADDR      0x68
#define GYRO_XOUT 				0x43
#define ACCEL_XOUT_H 			0x3B
#define WHO_AM_I				0x75
#define PWR_MGMT_1				0x6B

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

    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);

    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, MODULE_NoMsk);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048, MODULE_NoMsk);

    SystemCoreClockUpdate();
		
		SYS->GPB_MFP3 = SYS_GPB_MFP3_PB13MFP_UART0_TXD | SYS_GPB_MFP3_PB12MFP_UART0_RXD;
    SYS->GPC_MFP0 = SYS_GPC_MFP0_PC1MFP_I2C0_SCL | SYS_GPC_MFP0_PC0MFP_I2C0_SDA;
    SYS->GPF_MFP0 = SYS_GPF_MFP0_PF1MFP_ICE_CLK | SYS_GPF_MFP0_PF0MFP_ICE_DAT;

    SYS_LockReg();
}

uint8_t getID(){
	I2C_WriteByte(I2C0, MPU6050_SLAVE_ADDR, WHO_AM_I);
	CLK_SysTickDelay(100000);
	
	return I2C_ReadByte(I2C0, MPU6050_SLAVE_ADDR);
}

void MPU6050_ReadAcc(){
	uint8_t acc_array[6];
	uint16_t combineX=0, combineY=0, combineZ=0;
	float x=0, y=0, z=0;
	
	I2C_WriteByte(I2C0, MPU6050_SLAVE_ADDR, ACCEL_XOUT_H);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, MPU6050_SLAVE_ADDR, acc_array, 6);
	combineX = (int16_t)(acc_array[0] << 8 | acc_array[1]);
  	combineY = (int16_t)(acc_array[2] << 8 | acc_array[3]);
  	combineZ = (int16_t)(acc_array[4] << 8 | acc_array[5]);
	
	x = (float)combineX;
	y = (float)combineY;
	z = (float)combineZ;
	
	printf("AccX: %.2f, AccY: %.2f, AccZ: %.2f\n", x,y,z);
}

void MPU6050_ReadGyro(){
	uint8_t gyro_array[6];
	uint16_t combineX=0, combineY=0, combineZ=0;
	float x=0, y=0, z=0;
	
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
	
	printf("GyroX: %.2f, GyroY: %.2f, GyroZ: %.2f\n", x,y,z);
}

int main(){
	SYS_Init();
	
    UART_Open(UART0, 115200); 
	
	I2C_Open(I2C0, 100000); 
		
	printf("PROGRAM BASLADI\r\n");
		
	if(getID()==104){
		uint8_t array[2] = {PWR_MGMT_1, 0x00};
			
		I2C_WriteByte(I2C0, MPU6050_SLAVE_ADDR, PWR_MGMT_1);
		CLK_SysTickDelay(100000);
				
		I2C_WriteMultiBytes(I2C0, MPU6050_SLAVE_ADDR, array, 2);
		I2C_WriteByte(I2C0, MPU6050_SLAVE_ADDR, PWR_MGMT_1);		
	}
	
    while(1){ 
		MPU6050_ReadGyro();
      	CLK_SysTickDelay(300);
	}	
}