#include <stdio.h>
#include "NuMicro.h"

#define LED_B  PC4

#define btn PB4

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

int main()
{
    SYS_Init();

    UART_Open(UART0, 115200);
		GPIO_SetMode(PC, BIT4, GPIO_MODE_OUTPUT);
    /* Set PB.4 to GPIO */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk)) | (SYS_GPB_MFPL_PB4MFP_GPIO);
    /* Set PB.4 to GPIO intput */
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 4, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(GPIO_PAPB_IRQn);

    /* Set de-bounce function */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_512);
    GPIO_ENABLE_DEBOUNCE(PB, BIT4);  

    while (1);		
}

void GPABGH_IRQHandler(void)
{
    PC4 ^= 1;
		GPIO_CLR_INT_FLAG(PB, BIT4);
}

