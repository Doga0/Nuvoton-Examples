#include <stdio.h>
#include "NuMicro.h"

#define LED_B PC4
#define pot PB2

volatile uint32_t g_u32AdcIntFlag;

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* ADC clock source is PCLK1, set divider to 1 */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set PB.0 - PB.3 to input mode */
    GPIO_SetMode(PB, BIT0|BIT1|BIT2|BIT3, GPIO_MODE_INPUT);
    /* Configure the PB.0 - PB.3 ADC analog input pins.  */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk)) |
                    (SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB1MFP_ADC0_CH1 | SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3);
    /* Disable the PB.0 - PB.3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT0|BIT1|BIT2|BIT3);

    /* Lock protected registers */
    SYS_LockReg();
}

void ADC_FunctionTest(){
{
    int32_t i32ConversionData;
		/* Set the ADC operation mode as continuous scan, input mode as single-end and
		enable the analog input channel 0, 1, 2 and 3 */
		ADC_POWER_ON(ADC);
		GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_CONTINUOUS, BIT0|BIT1|BIT2|BIT3);

    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Enable the sample module interrupt */
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);  /* Enable sample module A/D interrupt. */
    NVIC_EnableIRQ(ADC_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 0 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    ADC_START_CONV(ADC);

    /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
    while(g_u32AdcIntFlag == 0);

		while(1){
			i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 0);
			//printf("\nDATA1: %d\n", i32ConversionData);
			
			if(i32ConversionData > 1500)
				PC4 = 1;
			else if(i32ConversionData <= 1500)
				PC4 = 0;
			CLK_SysTickDelay(100);
		}
		
    /* Wait ADC interrupt of next round (g_u32AdcIntFlag will be set at IRQ_Handler function) */
    g_u32AdcIntFlag = 0;
    while(g_u32AdcIntFlag == 0);

		/* Stop A/D conversion */
    ADC_STOP_CONV(ADC);

    /* Disable the sample module interrupt */
    ADC_DISABLE_INT(ADC, ADC_ADF_INT);
  }
}

void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
}


void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    UART_Open(UART0, 115200);
}

int main()
{
    SYS_Init();	
		
		UART0_Init();
		
		printf("\nSystem clock rate: %d Hz", SystemCoreClock);
		
		ADC_FunctionTest();
		
		/* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    while (1);		
}

