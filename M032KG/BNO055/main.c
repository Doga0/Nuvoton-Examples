#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
#include <math.h>


#define BNO055_SLAVE_ADDR 		0x29
#define CHIP_ID								0x00
#define ACC_DATA							0x08
#define GYR_DATA							0x14
#define MAG_DATA							0x10
#define OPR_MODE							0x3D
#define OPR_MODE_AMG					0x07 
#define OPR_MODE_NDOF					0x0C
#define UNIT_SEL							0x3B
#define UNITS									0x82 
#define ACC_DATA_Z 						0x0C
#define CALIB_STAT						0x35
#define EUL_DATA_X						0x1A
#define LIA_DATA_X						0x28

#define M_PI acos(-1)

#define LED_G  PC4

float Q_angle = 0.1;  
float Q_bias = 0.3; 
float R_measure = 0.15; 

float angle = 0;
float bias = 0; 
float P[2][2] = {{0, 0}, {0, 0}};

static volatile uint32_t g_u32RTCTInt = 0;


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

    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LXTEN_Msk|CLK_PWRCTL_HXTEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_LXTSTB_Msk|CLK_STATUS_HXTSTB_Msk);
    CLK_DisablePLL();
    CLK->PLLCTL = (CLK->PLLCTL & ~(0x000FFFFFUL)) | 0x0008C03EUL;
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(ISP_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);

    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, MODULE_NoMsk);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048, MODULE_NoMsk);
		
    SystemCoreClockUpdate();

	SYS->GPB_MFPH = SYS_GPB_MFPH_PB13MFP_UART0_TXD | SYS_GPB_MFPH_PB12MFP_UART0_RXD;
    SYS->GPC_MFPL = SYS_GPC_MFPL_PC1MFP_I2C0_SCL | SYS_GPC_MFPL_PC0MFP_I2C0_SDA;
    SYS->GPF_MFPL = SYS_GPF_MFPL_PF1MFP_ICE_CLK | SYS_GPF_MFPL_PF0MFP_ICE_DAT;
  
	CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    CLK_EnableModuleClock(RTC_MODULE);
		
    SYS_LockReg();
}

uint16_t getChipID(){
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, CHIP_ID);
	CLK_SysTickDelay(100000);
	
	return I2C_ReadByte(I2C0, BNO055_SLAVE_ADDR);
}

void op_mode(){
	uint8_t array[2] = {OPR_MODE, OPR_MODE_NDOF};
	
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, OPR_MODE);
	CLK_SysTickDelay(100000);
	I2C_WriteMultiBytes(I2C0, BNO055_SLAVE_ADDR, array, 2);
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, OPR_MODE);
	printf("2: %x\n", I2C_ReadByte(I2C0, BNO055_SLAVE_ADDR));
}

void unit_selection(){
	uint8_t array[2] = {UNIT_SEL, UNITS};
	
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, UNIT_SEL);
	CLK_SysTickDelay(100000);
	I2C_WriteMultiBytes(I2C0, BNO055_SLAVE_ADDR, array, 2);
	CLK_SysTickDelay(100000);
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, UNIT_SEL);
	printf("3: %x\n", I2C_ReadByte(I2C0, BNO055_SLAVE_ADDR));
}

void BNO055_ERROR(){
	
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, 0x3A);
	printf("error: %x\n", I2C_ReadByte(I2C0, BNO055_SLAVE_ADDR));
}

uint8_t* calibration_status(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag){
	uint8_t calib = I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, CALIB_STAT);
	static uint8_t output[4];
	//printf("\nCALIB STATUS: %x\n", I2C_ReadByte(I2C0, BNO055_SLAVE_ADDR));
	
	if (sys != NULL) {
    *sys = (calib >> 6) & 0x03;
  	}
  	if (gyro != NULL) {
  	  *gyro = (calib >> 4) & 0x03;
  	}
  	if (accel != NULL) {
  	  *accel = (calib >> 2) & 0x03;
  	}
  	if (mag != NULL) {
  	  *mag = calib & 0x03;
  	}
	
	output[0] = *sys;
	output[1] = *gyro;
	output[2] = *accel;
	output[3] = *mag;
	
	return output;
}

void BNO055_init(){
	if(getChipID() ==  160){
				BuzzerMod1();
	}
	else{
			BuzzerMod2();
	}
	uint8_t sys, gyro, accel, mag;
		
	printf("ID: %d\n", getChipID());
	unit_selection();
	op_mode();
	uint8_t* calData = calibration_status(&sys, &gyro, &accel, &mag);
	printf("SYS: %d, 				ACC: %d\n ", calData[0], calData[2]);
	BNO055_ERROR();
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

float* BNO055_readAcc(double milis, int kalman){
	uint8_t acc_array[6];
	static float output[3];
	uint16_t combineX=0, combineY=0, combineZ=0;
	float x=0, y=0, z=0;
	uint32_t timer;
	
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, ACC_DATA);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, BNO055_SLAVE_ADDR, acc_array, 6);
	combineX = ((int16_t)acc_array[0])  | (((int16_t)acc_array[1]) << 8);
  	combineY = ((int16_t)acc_array[2])  | (((int16_t)acc_array[3]) << 8);
  	combineZ = ((int16_t)acc_array[4])  | (((int16_t)acc_array[5]) << 8);
	
	x = (float)combineX;
	y = (float)combineY;
	z = (float)combineZ;
	
	if(kalman==0){
		
		output[0] = x;
		output[1] = y;
		output[2] = z;
	
		return output;
		
	}
	else{
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
}

double accZ(){
	uint8_t acc_array[2];
	uint16_t combineZ=0;
	double z=0;
	uint32_t timer;
	
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, ACC_DATA_Z);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, BNO055_SLAVE_ADDR, acc_array, 2);
	combineZ = ((int16_t) acc_array[0])  | (((int16_t)acc_array[1])<< 8);
	
	z = (double)combineZ;
	
	return z;
}

double offsetAccZ(){
	uint8_t acc_array[2];
	uint16_t combineZ=0;
	double z=0;
	uint32_t timer;
	
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, 0x59);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, BNO055_SLAVE_ADDR, acc_array, 2);
	combineZ = (int16_t)(acc_array[0] << 8 | acc_array[1]);
	
	z = (double)combineZ;
	
	return z;
}

double* BNO055_readGyro(double milis){
	uint8_t gyro_array[6];
	static double output[3];
	uint16_t combineX=0, combineY=0, combineZ=0;
	float x=0, y=0, z=0;
	uint32_t timer;
	
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, GYR_DATA);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, BNO055_SLAVE_ADDR, gyro_array, 6);
	combineX = ((int16_t)gyro_array[0])  | (((int16_t)gyro_array[1]) << 8);
  	combineY = ((int16_t)gyro_array[2])  | (((int16_t)gyro_array[3]) << 8);
  	combineZ = ((int16_t)gyro_array[4])  | (((int16_t)gyro_array[5]) << 8);
	
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

double* BNO055_readMag(double milis){
	uint8_t mag_array[6];
	static double output[3];
	uint16_t combineX=0, combineY=0, combineZ=0;
	float x=0, y=0, z=0;
	uint32_t timer;
	
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, MAG_DATA);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, BNO055_SLAVE_ADDR, mag_array, 6);
	combineX = ((int16_t)mag_array[0])  | (((int16_t)mag_array[1]) << 8);
  	combineY = ((int16_t)mag_array[2])  | (((int16_t)mag_array[3]) << 8);
  	combineZ = ((int16_t)mag_array[4])  | (((int16_t)mag_array[5]) << 8);
	

  	x = (float)combineX;
  	y = (float)combineY;
  	z = (float)combineZ;
	
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

float* BNO055_readEUL(double milis){
	uint8_t eul_array[6];
	static float output[3];
	uint16_t combineX=0, combineY=0, combineZ=0;
	float x=0, y=0, z=0;
	uint32_t timer;
	
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, EUL_DATA_X);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, BNO055_SLAVE_ADDR, eul_array, 6);
	
	combineX = (int16_t)((eul_array[1] << 8) | eul_array[0]);
  	combineY = (int16_t)((eul_array[3] << 8) | eul_array[2]);
  	combineZ = (int16_t)((eul_array[5] << 8) | eul_array[4]);

  	x = (float)combineX;
  	y = (float)combineY;
  	z = (float)combineZ;
	
	double dt = (double)(milis - timer) / 1000;
	timer = milis;
	
	double filteredX = kalman_filter(x, x, dt);
	double filteredY = kalman_filter(y, y, dt);
	double filteredZ = kalman_filter(z, z, dt);
	
	output[0] = filteredX/16;
	output[1] = filteredY/16;
	output[2] = filteredZ/16;
	
	return output;
}

float* BNO055_readLIA(double milis){
	uint8_t lia_array[6];
	static float output[3];
	uint16_t combineX=0, combineY=0, combineZ=0;
	float x=0, y=0, z=0;
	uint32_t timer;
	
	I2C_WriteByte(I2C0, BNO055_SLAVE_ADDR, LIA_DATA_X);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, BNO055_SLAVE_ADDR, lia_array, 6);
	combineX = ((int16_t)lia_array[0])  | (((int16_t)lia_array[1]) << 8);
  	combineY = ((int16_t)lia_array[2])  | (((int16_t)lia_array[3]) << 8);
  	combineZ = ((int16_t)lia_array[4])  | (((int16_t)lia_array[5]) << 8);
	
	x = (float)combineX;
	y = (float)combineY;
	z = (float)combineZ;
	
	double dt = (double)(milis - timer) / 1000;
	timer = milis;
	
	double filteredX = kalman_filter(x, x, dt);
	double filteredY = kalman_filter(y, y, dt);
	double filteredZ = kalman_filter(z, z, dt);
	
	output[0] = filteredX;
	output[1] = filteredY;
	output[2] = filteredZ;
		
	output[0] = x;
	output[1] = y;
	output[2] = z;
	
	return output;
}

float vertical_v, velocity;
float verticalVelocity(float *acc, float pitch, float roll, double milis){
	float AccZ_Inertial = 0, vv;
	uint32_t timer;
	
	AccZ_Inertial = -sin(pitch*(3.142/180))*acc[0]+cos(pitch*(3.142/180))*sin(roll*(3.142/180))*acc[1]+cos(pitch*(3.142/180))*cos(roll*(3.142/180))*acc[2];
	vertical_v = vertical_v + AccZ_Inertial * 0.004;
	
	double dt = (double)(milis - timer) / 1000;
	timer = milis;
	
	vv = kalman_filter(vertical_v, vertical_v, dt);
	
	return vv*0.01;
}



void BuzzerMod1(){ 
	EPWM_ConfigOutputChannel(EPWM0, 3, 10000, 100);
	EPWM_EnableOutput(EPWM0, EPWM_CH_3_MASK);		
	EPWM_Start(EPWM0, EPWM_CH_3_MASK);
				
	CLK_SysTickDelay(1000000);
				
	EPWM_ConfigOutputChannel(EPWM0, 3, 10000, 0);
	EPWM_EnableOutput(E PWM0, EPWM_CH_3_MASK);		 
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
	
	printf("PROGRAM BASLADI\n");
		
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
		
	GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);
		
	BNO055_init();
		
		
	while(1){
			
		if(g_u32RTCTInt == 1)
        {
            g_u32RTCTInt = 0;

            RTC_GetDateAndTime(&sReadRTC);
			//printf("sec: %d\n", sReadRTC.u32Minute*60+sReadRTC.u32Second);
			milis = sReadRTC.u32Minute*60+sReadRTC.u32Second;
		}
				
		//float *acc = BNO055_readAcc(milis, 1);
		//printf("Acc x: %.2f\n", acc[0]/100);
		//printf("Acc y: %.2f\n", acc[1]/100);
		//printf("Acc z: %.2f\n", acc[2]/100);
				
		//double accz = accZ();
		//printf("Z:%f\n", accz/100);
				
				
		//printf("x: %.2f     y: %.2f     z: %.2f\n", eul[0], eul[1], eul[2]);
		//float *lia = BNO055_readLIA(milis);
				
		float *eul = BNO055_readEUL(milis);
		printf("pitch: %.2f\n", eul[2]);
				
		if(eul[2]<=30 && eul[2]>=-20){
			LED_G = 1;
		}
		else
			LED_G = 0;
				
		CLK_SysTickDelay(10);	
	}
}