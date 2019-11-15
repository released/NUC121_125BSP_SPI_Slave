/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Toggle PB.4 to turn on/off LED.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"

//#define ENABLE_SPI_NORMAL
#define ENABLE_SPI_PDMA

#define USPI_SLAVE_TX_DMA_CH  						(0)
#define USPI_SLAVE_RX_DMA_CH  						(1)
//#define USPI_SLAVE_OPENED_CH   						((1 << USPI_SLAVE_TX_DMA_CH) | (1 << USPI_SLAVE_RX_DMA_CH))
#define USPI_SLAVE_OPENED_CH   						(1 << USPI_SLAVE_RX_DMA_CH)

#define USPI_SLAVE_DATA_NUM						(1)
uint8_t g_au8USlaveRxBuffer[USPI_SLAVE_DATA_NUM]={0};

void convertDecToBin(int n)
{
	int k = 0;
	unsigned char *p = (unsigned char*)&n;
	int val2 = 0;
	int i = 0;
	for(k = 0; k <= 1; k++)
	{
		val2 = *(p+k);
		for (i = 7; i >= 0; i--)
		{
			if(val2 & (1 << i))
				printf("1");
			else
				printf("0");
		}
		printf(" ");
	}
}

#if defined (ENABLE_SPI_PDMA)

void USPI_Slave_PDMA_Init(void)
{	
    PDMA_Open(USPI_SLAVE_OPENED_CH);

	//RX
    PDMA_SetTransferCnt(USPI_SLAVE_RX_DMA_CH, PDMA_WIDTH_8, USPI_SLAVE_DATA_NUM);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(USPI_SLAVE_RX_DMA_CH, (uint32_t)&USPI0->RXDAT, PDMA_SAR_FIX, (uint32_t)g_au8USlaveRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(USPI_SLAVE_RX_DMA_CH, PDMA_USCI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(USPI_SLAVE_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
//    PDMA->DSCT[USPI_SLAVE_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;
	PDMA_DisableInt(USPI_SLAVE_RX_DMA_CH,PDMA_INT_TEMPTY);
	
    USPI_ENABLE_PDMA(USPI0);;

    PDMA_EnableInt(USPI_SLAVE_RX_DMA_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA_IRQn);	
	
}

void PDMA_IRQHandler(void)
{	
    uint32_t status = PDMA_GET_INT_STATUS();
	
    if (status & PDMA_INTSTS_ABTIF_Msk)   /* abort */
    {
        PDMA_CLR_ABORT_FLAG(PDMA_GET_ABORT_STS());
    }
    else if (status & PDMA_INTSTS_TDIF_Msk)     /* done */
    {
        if((PDMA_GET_TD_STS() & USPI_SLAVE_OPENED_CH) == USPI_SLAVE_OPENED_CH)
        {
		    /* Clear PDMA transfer done interrupt flag */
		    PDMA_CLR_TD_FLAG(USPI_SLAVE_OPENED_CH);

			printf("USPI Slave PDMA: 0x%2X\r\n",g_au8USlaveRxBuffer[0]);

			//insert process
			USPI_DISABLE_PDMA(USPI0);
		
		    PDMA_SetTransferCnt(USPI_SLAVE_RX_DMA_CH, PDMA_WIDTH_8, USPI_SLAVE_DATA_NUM);			
   			PDMA_SetTransferAddr(USPI_SLAVE_RX_DMA_CH, (uint32_t)&USPI0->RXDAT, PDMA_SAR_FIX, (uint32_t)g_au8USlaveRxBuffer, PDMA_DAR_INC);			
		    /* Set request source; set basic mode. */
		    PDMA_SetTransferMode(USPI_SLAVE_RX_DMA_CH, PDMA_USCI0_RX, FALSE, 0);
		    USPI_ENABLE_PDMA(USPI0);
        } 
    }
    else if (status & (PDMA_INTSTS_REQTOF0_Msk | PDMA_INTSTS_REQTOF1_Msk))     /* Check the DMA time-out interrupt flag */
    {
        PDMA_CLR_TMOUT_FLAG(USPI_SLAVE_RX_DMA_CH);
		
    }
    else
    {

    }

}
#endif

#if defined (ENABLE_SPI_NORMAL)
void USCI_IRQHandler(void)
{
    uint32_t u32RxData = 0;

    /* Clear TX end interrupt flag */
    USPI_CLR_PROT_INT_FLAG(USPI0, USPI_PROTSTS_RXENDIF_Msk);

    /* Waiting for RX is not empty */
    while (USPI_GET_RX_EMPTY_FLAG(USPI0) == 1);

    /* Check RX EMPTY flag */
    while (USPI_GET_RX_EMPTY_FLAG(USPI0) == 0)
    {
        /* Read RX Buffer */
        u32RxData = USPI_READ_RX(USPI0);
		printf("USPI Slave : 0x%2X\r\n",u32RxData);

//		printf("RX : 0x%2X\r\n",u32RxData);
//		convertDecToBin(u32RxData);
//		printf("\r\n");

		
		PB4 ^= 1;
    }

}
#endif


void USCI_SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI_SPI0                                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI0 as a slave, USCI_SPI0 clock rate = f_PCLK1,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI_Open(USPI0, USPI_SLAVE, USPI_MODE_0, 8, 0);
	
    /* Configure USCI_SPI_SS pin as low-active. */
//    USPI0->CTLIN0 = (USPI0->CTLIN0 & ~USPI_CTLIN0_ININV_Msk) | USPI_CTLIN0_ININV_Msk;

#if defined (ENABLE_SPI_NORMAL)
    USPI_EnableInt(USPI0, USPI_RXEND_INT_MASK);
    NVIC_EnableIRQ(USCI_IRQn);
#endif

#if defined (ENABLE_SPI_PDMA)
    USPI_Slave_PDMA_Init();
#endif



}

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC/2 and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC_DIV2, CLK_CLKDIV0_UART(1));

    CLK_EnableModuleClock(PDMA_MODULE);
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Update core clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_UART0_RXD;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk)) | SYS_GPB_MFPL_PB1MFP_UART0_TXD;


 	//USCI_SPI0_SS (PC.1)
 	//USCI_SPI0_CLK (PC.0)
 	//USCI_SPI0_MOSI (PC.3)
 	//USCI_SPI0_MISO (PC.2)

    /* Set USCI_SPI0 multi-function pins */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC1MFP_Msk))  | (SYS_GPC_MFPL_PC1MFP_USCI0_CTL0);   /* PC.1  */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC0MFP_Msk))  | (SYS_GPC_MFPL_PC0MFP_USCI0_CLK);    /* PC.0  */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC3MFP_Msk))  | (SYS_GPC_MFPL_PC3MFP_USCI0_DAT0);   /* PC.3  */
//    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC2MFP_Msk))  | (SYS_GPC_MFPL_PC2MFP_USCI0_DAT1);   /* PC.2  */

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void delay_loop(void)
{
    __IO uint32_t j;

    for (j = 0; j < 60000; j++);

    for (j = 0; j < 60000; j++);

    for (j = 0; j < 60000; j++);

    for (j = 0; j < 60000; j++);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

    PB4 = 1;
    GPIO_SetMode(PB, BIT4, GPIO_MODE_OUTPUT);

    USCI_SPI_Init();
	
    while (1)
    {
//		delay_loop();
//		PB4 ^= 1;
    }

}
