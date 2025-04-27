#include <stdio.h>
#include "NuMicro.h"
#include <stdbool.h>


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
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, MODULE_NoMsk);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, MODULE_NoMsk);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048, MODULE_NoMsk);
    SystemCoreClockUpdate();

    SYS->GPB_MFP1 = SYS_GPB_MFP1_PB5MFP_TM0;
    SYS->GPB_MFP3 = SYS_GPB_MFP3_PB13MFP_UART0_TXD | SYS_GPB_MFP3_PB12MFP_UART0_RXD;
    SYS->GPC_MFP0 = SYS_GPC_MFP0_PC1MFP_I2C0_SCL | SYS_GPC_MFP0_PC0MFP_I2C0_SDA;
    SYS->GPF_MFP0 = SYS_GPF_MFP0_PF1MFP_ICE_CLK | SYS_GPF_MFP0_PF0MFP_ICE_DAT;
	
    SYS_LockReg();
}

#define TH08_SLAVE_ADDR             	0x40
#define TH08_READ_TEMP_CMD			    0xE3
#define TH08_WRITE_USER_REG1_CMD		0xE6
#define TH08_READ_USER_REG1_CMD		  	0xE7
#define TH08_HTRE_OPEN_MSG		      	0x3e
#define TH08_HTRE_CLOSE_MSG		      	0x3a

uint8_t TH08_HTRE_MSG[2]= {TH08_WRITE_USER_REG1_CMD,0};

void TH08_HTRE(bool s){
	if(s){
		TH08_HTRE_MSG[1] = TH08_HTRE_OPEN_MSG;
		I2C_WriteByte(I2C0, TH08_SLAVE_ADDR, TH08_READ_USER_REG1_CMD);
		printf("Eski register degeri %x \r\n",I2C_ReadByte(I2C0, TH08_SLAVE_ADDR));
		I2C_WriteMultiBytes(I2C0, TH08_SLAVE_ADDR,TH08_HTRE_MSG,2);
		I2C_WriteByte(I2C0, TH08_SLAVE_ADDR, TH08_READ_USER_REG1_CMD);
		printf("Yeni register degeri %x \r\n",I2C_ReadByte(I2C0, TH08_SLAVE_ADDR));
	}else if(!s){
		TH08_HTRE_MSG[1] = TH08_HTRE_CLOSE_MSG;
		I2C_WriteByte(I2C0, TH08_SLAVE_ADDR, TH08_READ_USER_REG1_CMD);
		printf("Eski user register degeri %x \r\n",I2C_ReadByte(I2C0, TH08_SLAVE_ADDR));
		I2C_WriteMultiBytes(I2C0, TH08_SLAVE_ADDR,TH08_HTRE_MSG,2);
		I2C_WriteByte(I2C0, TH08_SLAVE_ADDR, TH08_READ_USER_REG1_CMD);
		printf("Yeni user regidter degeri %x \r\n",I2C_ReadByte(I2C0, TH08_SLAVE_ADDR));
	}
}

float TH08_readTemp(void)
{
	uint8_t temp_Array[2];
	uint16_t temp_Combine;
	float temp = 0;
	I2C_WriteByte(I2C0, TH08_SLAVE_ADDR, TH08_READ_TEMP_CMD);
	CLK_SysTickDelay(100000);
	I2C_ReadMultiBytes(I2C0, TH08_SLAVE_ADDR, temp_Array, 2);
	temp_Combine = (temp_Array[0] << 8) | temp_Array[1];    // 2 byte'lik veriyi 16 bit degerindeki tek degiskene combine edildi.
	temp = (float)temp_Combine;
  	return 175.72*temp/65536-46.85;
}

int main(){
	SYS_Init();
    UART_Open(UART0, 115200); 
	I2C_Open(I2C0, 100000); 
	printf("PROGRAM BASLADI\r\n");
	TH08_HTRE(FALSE);
	CLK_SysTickDelay(50000000);
    while(1){ 
		printf("Sicaklik %f santigrad derece\n", TH08_readTemp());
		CLK_SysTickDelay(300);
	}
}
