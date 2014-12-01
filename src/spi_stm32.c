/*
 * History:
 *
 * 27.08.2014 - Michal Budzon - Initial
 */
 
 #include "spi_stm32.h"

static void spiSSInit(){
    GPIO_InitTypeDef GPIOB_InitStruct; // GPIO pin used as SS
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Init SS
    GPIOB_InitStruct.GPIO_Pin   = GPIO_Pin_12; 
    GPIOB_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;               // configured as alternate function
    GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;            // IO speed
    GPIOB_InitStruct.GPIO_OType = GPIO_OType_PP;               // output type as push pull mode
    GPIOB_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;                // pullup resistors on the IO pins
    GPIO_Init(GPIOB, &GPIOB_InitStruct);
    
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void spiInit(){
    SPI_InitTypeDef SPI_InitStruct;
    GPIO_InitTypeDef GPIOB_InitStruct; // GPIO pins used as TX and RX
    
        //Init Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,  ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    // Init SPI's GPIO
    GPIOB_InitStruct.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
    GPIOB_InitStruct.GPIO_Mode  = GPIO_Mode_AF;                // configured as alternate function 
    GPIOB_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;            // IO speed
    GPIOB_InitStruct.GPIO_OType = GPIO_OType_PP;               // output type as push pull mode 
    GPIOB_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;            // pullup resistors on the IO pins
    GPIO_Init(GPIOB, &GPIOB_InitStruct);                       // GPIO_Init() function sets the GPIO registers
    
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13,  GPIO_AF_SPI2); //
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,  GPIO_AF_SPI2); //
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15,  GPIO_AF_SPI2); //
    
    //Init SPI
    SPI_StructInit(&SPI_InitStruct);
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_Low;
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStruct.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStruct.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;
    
    SPI_Init(SPI2, &SPI_InitStruct);
    SPI_Cmd(SPI2, ENABLE); 

    spiSSInit();
    spiTxByte(0x00); // send dummy byte
}

uint8_t spiTxByte (const uint8_t byte)
{
    
    SPI2->DR = byte; // write data to be transmitted to the SPI data register

    while( !(SPI2->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
    while( !(SPI2->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
    while(   SPI2->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
    
    return SPI2->DR;
}

uint8_t spiRxByte (void)
{
    return spiTxByte (0x00);
}

void spiActive(void)
{
   GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

void spiInactive(void)
{
	 GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
