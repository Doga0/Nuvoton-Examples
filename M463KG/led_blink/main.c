#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

void SYS_Init(void)
{
		 SYS_UnlockReg();

    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk|CLK_PWRCTL_HIRCEN_Msk|CLK_PWRCTL_LXTEN_Msk|CLK_PWRCTL_HXTEN_Msk|CLK_PWRCTL_HIRC48MEN_Msk);

    /* Waiting for clock source ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk|CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_LXTSTB_Msk|CLK_STATUS_HXTSTB_Msk|CLK_STATUS_HIRC48MSTB_Msk);

    /* Disable PLL first to avoid unstable when setting PLL */
    CLK_DisablePLL();

    /* Set PLL frequency */
    CLK->PLLCTL = (CLK->PLLCTL & ~(0x008FFFFFUL)) | 0x0008421EUL;

    /* Waiting for PLL ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Disable PLLFN first to avoid unstable when setting PLLFN */
    CLK->PLLCTL |= CLK_PLLFNCTL1_PD_Msk;

    /* Set PLLFN frequency */
    CLK->PLLFNCTL0 = (CLK->PLLFNCTL0 & ~(0x0FFFFFFFUL)) | 0x0000842BUL;

    /* Disable PLLFN first to avoid unstable when setting PLLFN */
    CLK->PLLCTL |= CLK_PLLFNCTL1_PD_Msk;

    /* Set PLLFN frequency */
    CLK->PLLFNCTL1 = (CLK->PLLFNCTL1 & ~(0xF8000000UL)) | 0x80000000UL;

    /* Set HCLK clock */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK-related clock */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV2 | CLK_PCLKDIV_PCLK1DIV2);

    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);

    /* Set IP clock */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, MODULE_NoMsk);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048, MODULE_NoMsk);
		
		/* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;
    /* Update System Core Clock */
    SystemCoreClockUpdate();
		
		SYS->GPA_MFP1 = SYS_GPA_MFP1_PA5MFP_GPIO;
    SYS->GPB_MFP0 = SYS_GPB_MFP0_PB1MFP_I2C1_SCL | SYS_GPB_MFP0_PB0MFP_I2C1_SDA;
    SYS->GPF_MFP0 = SYS_GPF_MFP0_PF1MFP_ICE_CLK | SYS_GPF_MFP0_PF0MFP_ICE_DAT;

    /* Lock protected registers */
    SYS_LockReg();
}


int main(){
		SYS_Init();
	
    UART_Open(UART0, 115200); 
	
		printf("PROGRAM BASLADI\r\n");
		
		GPIO_SetMode(PA, BIT5, GPIO_MODE_OUTPUT);
		
		PA5 = 1;
		
    while(1);
}