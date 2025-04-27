#include <stdio.h>
#include "NuMicro.h"
#include "HopeDuino_SPI.h"
#include "HopeDuino_LoRa.h"

#define Rx_Size_array 255

uint32_t time_interval = 5; 		//sorgunun yapilacagi aralik
uint32_t repeat_interval = 1;		//veri gonderilirken kac kez data gonderilecegi adedi
uint32_t receive_timeout = 5;		//cevabin beklenme suresi
uint32_t receive_try_count = 2;		//cevabin beklenme adedi

uint32_t device_count = 1; //5

uint32_t Rx_Size  = 255;
uint8_t gelen_byte;
uint8_t gelen_string[Rx_Size_array];
//uint8_t tx_buf[Rx_Size_array];
uint8_t bitsayici = 0,i=0;
byte rx_buf[Rx_Size_array];

uint8_t muhur = 1;
//uint8_t veri_send = 0;
uint32_t counter = 0,counter_say = 1;
uint8_t sec5_sayici = 0;
uint8_t wait_bit = 1,sec5_sayici_baslat = 0;

void TMR0_IRQHandler()
{
	if(counter_say) counter++;
	if(counter >= time_interval)
	{
		counter = 0;
		//veri_send = 1;
		counter_say = 0;
	}
	
	if(sec5_sayici_baslat) sec5_sayici++;
	if(sec5_sayici >=receive_timeout)
	{
		wait_bit =0;
		sec5_sayici_baslat = 0;
	}
	
	TIMER_ClearIntFlag(TIMER0);
}


void UART0_IRQHandler()
{
	if(UART_GET_INT_FLAG(UART0,UART_INTSTS_RDAINT_Msk))
	{
		while(UART_IS_RX_READY(UART0))
		{
			gelen_byte = UART_READ(UART0);
			printf("gelen_byte : %c  bitsayici : %d\r\n",gelen_byte,bitsayici);
			if(gelen_byte == 0x0D)
			{
				printf("gelen string : %s \r\n",gelen_string);
				UART_Write(UART0,gelen_string,bitsayici);
				if(gelen_string[0] == 't' && gelen_string[1] == 'i' && gelen_string[2] == 'm' && gelen_string[3] == 'e')
				{
					time_interval = ((gelen_string[5]-48)*100) + ((gelen_string[6]-48)*10) + ((gelen_string[7]-48)*1);
				}
				
				
				for(i = 0; i < Rx_Size;i++)
				{
					gelen_string[i] = '\0';
				}
				bitsayici = 0;
			}
			else
			{
				gelen_string[bitsayici++] = gelen_byte;
			}
		}
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

    CLK_EnableModuleClock(SPI0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);

    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, MODULE_NoMsk);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LIRC, MODULE_NoMsk);
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL1_WWDTSEL_HCLK_DIV2048, MODULE_NoMsk);

    SystemCoreClockUpdate();

		SYS->GPA_MFP0 = SYS_GPA_MFP0_PA3MFP_SPI0_SS | SYS_GPA_MFP0_PA2MFP_SPI0_CLK | SYS_GPA_MFP0_PA1MFP_QSPI0_MISO0 | SYS_GPA_MFP0_PA0MFP_SPI0_MOSI;
    SYS->GPB_MFP3 = SYS_GPB_MFP3_PB13MFP_UART0_TXD | SYS_GPB_MFP3_PB12MFP_UART0_RXD;
    SYS->GPF_MFP0 = SYS_GPF_MFP0_PF1MFP_ICE_CLK | SYS_GPF_MFP0_PF0MFP_ICE_DAT;

   SYS_LockReg();
}

byte i,j,k,l,tmp,i1;
uint8_t reg_val;

uint32_t counter2 = 0;
int main()
{
	SYS_Init();
	
	UART_Open(UART0, 9600);
	UART_ClearIntFlag(UART0,UART_INTSTS_RLSINT_Msk);
	UART_EnableInt(UART0,UART_INTEN_RDAIEN_Msk);
	NVIC_EnableIRQ(UART0_IRQn);
	printf("program basladi\r\n");
	
	Modulation	 = LORA;
	COB			 		 = RFM95;
	Frequency 	 = 868000;
	OutputPower	 = 27;			   //20dBm OutputPower
	PreambleLength = 6;			   //6 symbols preamble
	FixedPktLength = false;		   //explicit header mode for LoRa
	PayloadLength  = 4;
	CrcDisable	 = true ;
	
	//for LORA parameter
	SFSel 		 = SF9;			//128 chips/symble
	BWSel 		 = BW125K;
	CRSel 		 = CR4_5;


	vInitialize();
  	vGoRx(); 

	TIMER_Open(TIMER0,TIMER_PERIODIC_MODE,1);
	TIMER_EnableInt(TIMER0);
	NVIC_EnableIRQ(TMR0_IRQn);
	TIMER_Start(TIMER0);
	
	while(1)
	{

			if(muhur)
			{
				muhur = 0;
				vGoRx();
			}
					
			tmp = bGetMessage(rx_buf);
			printf("TMP: %d\n", tmp);
			if(tmp!=0)
			{
				printf("Gelen Veri : %s\r\n",rx_buf);
							
				for(i = 0; i < Rx_Size; i++)
				{
						rx_buf[i] = '\0';
				}
			}
					
		//counter_say = 1;
	}
}

