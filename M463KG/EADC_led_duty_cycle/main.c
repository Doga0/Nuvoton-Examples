#include <stdio.h>
#include "NuMicro.h"

static volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;

void EADC00_IRQHandler(void)
{
  g_u32AdcIntFlag = 1;
  EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);     
}

void SYS_Init(void)
{
  SET_XT1_OUT_PF2();
  SET_XT1_IN_PF3();

  CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);
  CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);
  CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);
  CLK_SetCoreClock(200000000);
	
  CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                  CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
  CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

  CLK_EnableModuleClock(UART0_MODULE);
  CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
  CLK_EnableModuleClock(EADC0_MODULE);
	CLK_EnableModuleClock(EPWM1_MODULE); 
  CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_PLL_DIV2, CLK_CLKDIV0_EADC0(12));
	CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PCLK1, MODULE_NoMsk); ////

  SET_UART0_RXD_PB12();
  SET_UART0_TXD_PB13();

  SET_EADC0_CH6_PB6();
  SET_EPWM1_CH3_PC9(); 
    
  GPIO_DISABLE_DIGITAL_PATH(PB, BIT3 | BIT2 | BIT1 | BIT0);
}

void UART0_Init(void)
{
  UART_Open(UART0, 115200);
}

void EADC_FunctionTest(void){

  int32_t  ai32ConversionData = 0;
  uint32_t u32TimeOutCnt = 0;
  int32_t  i32Err;

  while(1)
  {
    i32Err = EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);
        
    EADC_ConfigSampleModule(EADC0, 4, EADC_ADINT0_TRIGGER, 6);
        
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

    EADC_ENABLE_INT(EADC0, BIT0);
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT4);
    NVIC_EnableIRQ(EADC00_IRQn);

    g_u32AdcIntFlag = 0;
    g_u32COVNUMFlag = 0;
    EADC_START_CONV(EADC0, BIT4);

    __WFI();
						
    EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT4);

		ai32ConversionData = EADC_GET_CONV_DATA(EADC0,4);
						
    printf("%d\n", ai32ConversionData);
						
		uint32_t dutyCycle = (uint32_t)(ai32ConversionData/40.95);
						
		EPWM_ConfigOutputChannel(EPWM1, 3, 10000, dutyCycle);
						
		EPWM_EnableOutput(EPWM1, EPWM_CH_3_MASK);
				
		EPWM_Start(EPWM1, EPWM_CH_3_MASK);
						
		CLK_SysTickDelay(1000);
						
    EADC_ConfigSampleModule(EADC0, 4, EADC_SOFTWARE_TRIGGER, 0);
				
    ai32ConversionData = 0;
  }
}

int32_t main(void)
{
    SYS_UnlockReg();
    SYS_Init();
    SYS_LockReg();

    UART0_Init();

    EADC_FunctionTest();
	
    while(1);
}