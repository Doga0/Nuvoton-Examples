#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "NuMicro.h"

#define HP303_SLAVE_ADDR           0x77
#define HP303_PRS_MSB_REG          0x00
#define HP303_TEMP_MSB_REG		   0x03
//#define CONT_PRS									 0x85
#define CONT_PRS				   0x87
#define CONT_TMP				   0x06
#define MEAS_CFG				   0x08
#define PRS_CFG					   0x06
#define PM_RATE_PRC 			   0x11 // mr: 2, osr: 2
#define ID						   0x0D
#define COEF					   0x10

#define NUM_OF_COEFFS			   18
#define NUM_OF_SCAL_FACTS			8

#define LED_G  PC4
#define PRS_OSR						1

const int32_t scaling_facts[NUM_OF_SCAL_FACTS] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};


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
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);

    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, MODULE_NoMsk);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, MODULE_NoMsk);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048, MODULE_NoMsk);

    SystemCoreClockUpdate();

		SYS->GPB_MFPH = SYS_GPB_MFPH_PB13MFP_UART0_TXD | SYS_GPB_MFPH_PB12MFP_UART0_RXD;
    SYS->GPB_MFPL = SYS_GPB_MFPL_PB5MFP_TM0;
    SYS->GPC_MFPL = SYS_GPC_MFPL_PC1MFP_I2C0_SCL | SYS_GPC_MFPL_PC0MFP_I2C0_SDA;
    SYS->GPF_MFPL = SYS_GPF_MFPL_PF1MFP_ICE_CLK | SYS_GPF_MFPL_PF0MFP_ICE_DAT;
		
    SYS_LockReg();
}

uint8_t HP303B_PRS_CFG[2] = {PRS_CFG, PM_RATE_PRC};

void pressureConfig() {
	
	I2C_WriteByte(I2C0, HP303_SLAVE_ADDR, PRS_CFG);
	
	I2C_WriteMultiBytes(I2C0, HP303_SLAVE_ADDR, HP303B_PRS_CFG, 2);
	I2C_WriteByte(I2C0, HP303_SLAVE_ADDR, PRS_CFG);
	
	CLK_SysTickDelay(100000);
	
}


uint8_t HP303B_BKG_MODE[2] = {MEAS_CFG, CONT_PRS};

uint32_t *readCoeffs(void){
	
	uint8_t buffer[NUM_OF_COEFFS];
	uint32_t* coeffs = (uint32_t*)malloc(9 * sizeof(uint32_t));
	
	if(coeffs == NULL) {
        
        return NULL;
    }
	
	I2C_WriteByte(I2C0, HP303_SLAVE_ADDR, MEAS_CFG);
	I2C_WriteMultiBytes(I2C0, HP303_SLAVE_ADDR, HP303B_BKG_MODE, 2);
	
	I2C_WriteByte(I2C0, HP303_SLAVE_ADDR, COEF);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, HP303_SLAVE_ADDR, buffer, NUM_OF_COEFFS);
	
	uint32_t m_c0Half =    ((uint32_t)buffer[0] << 4)
				| (((uint32_t)buffer[1] >> 4) & 0x0F);
	if(m_c0Half & ((uint32_t)1 << 11))
	{
		m_c0Half -= (uint32_t)1 << 12;
	}
	m_c0Half = m_c0Half / 2U;

	uint32_t m_c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
	if(m_c1 & ((uint32_t)1 << 11))
	{
		m_c1 -= (uint32_t)1 << 12;
	}

	uint32_t m_c00 =   ((uint32_t)buffer[3] << 12)
			| ((uint32_t)buffer[4] << 4)
			| (((uint32_t)buffer[5] >> 4) & 0x0F);
	if(m_c00 & ((uint32_t)1 << 19))
	{
		m_c00 -= (uint32_t)1 << 20;
	}

	uint32_t m_c10 =   (((uint32_t)buffer[5] & 0x0F) << 16)
			| ((uint32_t)buffer[6] << 8)
			| (uint32_t)buffer[7];
	if(m_c10 & ((uint32_t)1<<19))
	{
		m_c10 -= (uint32_t)1 << 20;
	}

	uint32_t m_c01 =   ((uint32_t)buffer[8] << 8)
			| (uint32_t)buffer[9];
	if(m_c01 & ((uint32_t)1 << 15))
	{
		m_c01 -= (uint32_t)1 << 16;
	}

	uint32_t m_c11 =   ((uint32_t)buffer[10] << 8)
			| (uint32_t)buffer[11];
	if(m_c11 & ((uint32_t)1 << 15))
	{
		m_c11 -= (uint32_t)1 << 16;
	}

	uint32_t m_c20 =   ((uint32_t)buffer[12] << 8)
			| (uint32_t)buffer[13];
	if(m_c20 & ((uint32_t)1 << 15))
	{
		m_c20 -= (uint32_t)1 << 16;
	}

	uint32_t m_c21 =   ((uint32_t)buffer[14] << 8)
			| (uint32_t)buffer[15];
	if(m_c21 & ((uint32_t)1 << 15))
	{
		m_c21 -= (uint32_t)1 << 16;
	}

	uint32_t m_c30 =   ((uint32_t)buffer[16] << 8)
			| (uint32_t)buffer[17];
	if(m_c30 & ((uint32_t)1 << 15))
	{
		m_c30 -= (uint32_t)1 << 16;
	}
	
	coeffs[0] = m_c0Half;
	coeffs[1] = m_c1;
	coeffs[2] = m_c00;
	coeffs[3] = m_c10;
	coeffs[4] = m_c01;
	coeffs[5] = m_c11;
	coeffs[6] = m_c20;
	coeffs[7] = m_c21;
	coeffs[8] = m_c30;
	
	return coeffs;
}

float HP303_rawPressure(void)
{
	uint8_t prs_Array[3];
	uint32_t prs_Combine = 0;
	float prs = 0;
	
	I2C_WriteByte(I2C0, HP303_SLAVE_ADDR, MEAS_CFG);
	I2C_WriteMultiBytes(I2C0, HP303_SLAVE_ADDR, HP303B_BKG_MODE, 2);
	
	I2C_WriteByte(I2C0, HP303_SLAVE_ADDR, HP303_PRS_MSB_REG);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, HP303_SLAVE_ADDR, prs_Array, 3);
	prs_Combine = (uint32_t)prs_Array[0] << 16 | (uint32_t)prs_Array[1] << 8 | (uint32_t)prs_Array[2];
	
	if(prs && ((uint32_t)1 << 23))
	{
		prs -= (uint32_t)1 << 24;
	}
	
	prs = (float)prs_Combine;
	return prs;
}

//uint8_t HP303_BKG_MODE_TMP[2] = {MEAS_CFG, CONT_TMP};

uint32_t HP303_rawTemperature(){
	uint8_t temp_Array[3];
	uint32_t temp_Combine = 0;
	float temp = 0;
	
	I2C_WriteByte(I2C0, HP303_SLAVE_ADDR, MEAS_CFG);
	I2C_WriteMultiBytes(I2C0, HP303_SLAVE_ADDR, HP303B_BKG_MODE, 2);
		
	I2C_WriteByte(I2C0, HP303_SLAVE_ADDR, HP303_TEMP_MSB_REG);
	CLK_SysTickDelay(100000);
	
	I2C_ReadMultiBytes(I2C0, HP303_SLAVE_ADDR, temp_Array, 3);
	temp_Combine = (uint32_t)temp_Array[0] << 16 | (uint32_t)temp_Array[1] << 8 | (uint32_t)temp_Array[2];
	
	temp = (float)temp_Combine;
	return temp;
}

uint32_t HP303_readTemperature(uint8_t temp_osr, int isMeasure){
	uint32_t *coeffs = readCoeffs();
	double temp = HP303_rawTemperature();
	uint32_t temp_scal;
	
	if(isMeasure){
		temp /= scaling_facts[temp_osr];
	
		temp_scal = temp;
	
		temp = coeffs[0] + coeffs[1] * temp;
	
		return (uint32_t)temp;
	}
	else{
		temp /= scaling_facts[temp_osr];
	
		temp_scal = (uint32_t)temp;
		
		return temp_scal;
	}
}

float HP303_readPressure(uint8_t prs_osr){
	
	uint32_t *coeffs = readCoeffs();
	float prs = HP303_rawPressure();
	uint32_t temp_scal = HP303_readTemperature(prs_osr, 0);
	
	prs /= scaling_facts[prs_osr];
	
	uint32_t p_comp = coeffs[2] + prs * (coeffs[3] + prs * (coeffs[6] + prs * coeffs[8])) + temp_scal 
	* coeffs[4] + temp_scal * prs *(coeffs[5] + prs * coeffs[7]);

	return (float)p_comp;
}

float calcAltitude(){
	
	float altitude;
	float prs = HP303_readPressure(PRS_OSR);
	altitude = 44307.69 * (1 - pow((prs*0.01)/1013.25, 0.190284));
	
	return altitude;
}

uint8_t HP303_getID(void){
	
	I2C_WriteByte(I2C0, HP303_SLAVE_ADDR, ID);
	CLK_SysTickDelay(100000);
	
	uint8_t id = I2C_ReadByte(I2C0, HP303_SLAVE_ADDR);
	
	return id;
}

/*void buzzer_test1(){
	PWM_ConfigOutputChannel(PWM0, 3, 10000, 100);
	PWM_EnableOutput(PWM0, PWM_CH_3_MASK);		
	PWM_Start(PWM0, PWM_CH_3_MASK);
				
	CLK_SysTickDelay(1000000);
				
	PWM_ConfigOutputChannel(PWM0, 3, 10000, 0);
	PWM_EnableOutput(PWM0, PWM_CH_3_MASK);		
	PWM_Start(PWM0, PWM_CH_3_MASK);
}

void buzzer_test2(){
	PWM_ConfigOutputChannel(PWM0, 3, 10000, 100);
	PWM_EnableOutput(PWM0, PWM_CH_3_MASK);		
	PWM_Start(PWM0, PWM_CH_3_MASK);
				
	CLK_SysTickDelay(1000000);
				
	PWM_ConfigOutputChannel(PWM0, 3, 10000, 0);
	PWM_EnableOutput(PWM0, PWM_CH_3_MASK);		
	PWM_Start(PWM0, PWM_CH_3_MASK);
				
	CLK_SysTickDelay(1000000);
				
	PWM_ConfigOutputChannel(PWM0, 3, 10000, 100);
	PWM_EnableOutput(PWM0, PWM_CH_3_MASK);		
	PWM_Start(PWM0, PWM_CH_3_MASK);
				
	CLK_SysTickDelay(1000000);
				
	PWM_ConfigOutputChannel(PWM0, 3, 10000, 0);
	PWM_EnableOutput(PWM0, PWM_CH_3_MASK);		
	PWM_Start(PWM0, PWM_CH_3_MASK);
}*/

float calcDerivative(double prs, int count){
	double prev_prs;
	int prev_count;
	
	float derivative = (prs-prev_prs/count-prev_count);
	
	prev_prs = prs;
	prev_count = count;
	
	return derivative;
}

int main(){
	SYS_Init();
	
    UART_Open(UART0, 115200); 
	I2C_Open(I2C0, 100000);
	
	printf("\nPROGRAM BASLADI\n");
	GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);
	
	pressureConfig();
	
	int i=0;
	
	while(1){ 
			
		float altitude = calcAltitude();
		float prs = HP303_readPressure(PRS_OSR)*0.01;
		printf("Basinc:%.2f \n", prs);
		//printf("Yukseklik:%.2f\n", calcAltitude());
		//printf("i:%d\n", i++);
			
		//printf("sicaklik: %d \n", (int)HP303_temperature());
		CLK_SysTickDelay(300);
	}
}