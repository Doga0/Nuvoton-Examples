
#include <stdio.h>
#include "NuMicro.h"


static volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;

int32_t main(void);
void EADC_FunctionTest(void);
void EADC00_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/* EADC interrupt handler                                                                                  */
void EADC00_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(200000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable EADC0 module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Set EADC0 clock divider as 12 */
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_PLL_DIV2, CLK_CLKDIV0_EADC0(12));

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pins for EADC0 channels. */
    SET_EADC0_CH6_PB6();
    /*SET_EADC0_CH11_PB11();
    SET_EADC0_CH14_PB14();
    SET_EADC0_CH15_PB15();
		*/
    /* Disable digital input path of EADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT3 | BIT2 | BIT1 | BIT0);

}

void UART0_Init(void)
{
    UART_Open(UART0, 115200);
}

/* EADC function test                                                                                      */
void EADC_FunctionTest(void)
{
    int32_t  ai32ConversionData = 0;
    uint32_t u32TimeOutCnt = 0;
    int32_t  i32Err;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    while(1)
    {
            /* Set input mode as single-end and enable the A/D converter */
            i32Err = EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

            /* Check EADC global error code. */
            if (i32Err != 0)
            {
                if (i32Err == EADC_CAL_ERR)
                {
                    printf("EADC has calibration error.\n");
                    return;
                }
                else if (i32Err == EADC_CLKDIV_ERR)
                {
                    printf("EADC clock frequency is configured error.\n");
                    return;
                }
                else
                {
                    printf("EADC has operation error.\n");
                    return;
                }
            }

            /* Configure the sample 4 module for analog input channel 10 and enable ADINT0 trigger source */
            EADC_ConfigSampleModule(EADC0, 4, EADC_ADINT0_TRIGGER, 6);
            
            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 4 interrupt */
            EADC_ENABLE_INT(EADC0, BIT0);//Enable sample module  A/D ADINT0 interrupt.
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT4);//Enable sample module 4 interrupt.
            NVIC_EnableIRQ(EADC00_IRQn);

            /* Reset the ADC indicator and trigger sample module 4 to start A/D conversion */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EADC_START_CONV(EADC0, BIT4);

            __WFI();

            /* Disable the sample module 7 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT4);

            /* Wait conversion done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(EADC_GET_DATA_VALID_FLAG(EADC0, (BIT4)) != (BIT4))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for EADC conversion done time-out!\n");
                    return;
                }
            }

            /* Get the conversion result of the sample module */
						ai32ConversionData = EADC_GET_CONV_DATA(EADC0,4);

            printf("%d\n", ai32ConversionData);

        /* Reset the sample module 4, 5, 6, 7 for analog input channel and disable ADINT0 trigger source */
        EADC_ConfigSampleModule(EADC0, 4, EADC_SOFTWARE_TRIGGER, 0);
		
        /* Clear the conversion result of the sample module */
        ai32ConversionData = 0;
    }
}


int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset EADC module */
    SYS_ResetModule(EADC0_RST);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}