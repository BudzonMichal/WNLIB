#ifndef SPI_STM32_H_
#define SPI_STM32_H_

#include "STM32F4xx.h"

void spiInit(void);
uint8_t spiTxByte (const uint8_t byte);
uint8_t spiRxByte (void);
void spiActive(void);
void spiInactive(void);

#endif // SPI_STM32_H_
