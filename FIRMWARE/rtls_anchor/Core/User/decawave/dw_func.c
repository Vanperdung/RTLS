#include "main.h"
#include "spi.h"
#include "dw_types.h"

extern SPI_HandleTypeDef hspi2;

uint16_t buffer[64];

static void spiWrite(dwDeviceTypes_t *dev, const void *header, size_t headerLength,
                     const void *data, size_t dataLength)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
    memcpy(buffer, header, headerLength);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)buffer, headerLength, HAL_MAX_DELAY);
    memcpy(buffer, data, dataLength);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)buffer, dataLength, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
}

static void spiRead(dwDeviceTypes_t *dev, const void *header, size_t headerLength,
                    void *data, size_t dataLength)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
    memcpy(buffer, header, headerLength);
    HAL_SPI_Transmit(&hspi2, (uint8_t *)buffer, headerLength, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, (uint8_t *)buffer, dataLength, HAL_MAX_DELAY);
    memcpy(data, buffer, dataLength);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
}

static void spiSetSpeed(dwDeviceTypes_t *dev, dwSpiSpeed_t speed)
{
    if (speed == dwSpiSpeedLow)
    {
    	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        HAL_SPI_Init(&hspi2);
    }
    else
    {
    	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
        HAL_SPI_Init(&hspi2);
    }
}

static void reset(dwDeviceTypes_t *dev)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);
}

void dwReset(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);
}

static void delayms(dwDeviceTypes_t *dev, unsigned int delay)
{
    HAL_Delay(delay);
}

dwFunction_t dwFunc = {
    .spiRead = spiRead,
    .spiWrite = spiWrite,
    .spiSetSpeed = spiSetSpeed,
    .delayms = delayms,
    .reset = reset,
};
