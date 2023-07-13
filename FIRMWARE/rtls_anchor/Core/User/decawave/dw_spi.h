#ifndef _DW_SPI_H_
#define _DW_SPI_H_

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "dw_types.h"

void dwSpiRead(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address, void *data, size_t length);
uint16_t dwSpiRead16(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address);
uint32_t dwSpiRead32(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address);

void dwSpiWrite(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address, const void *data, size_t length);
void dwSpiWrite8(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address, uint8_t data);
void dwSpiWrite32(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address, uint32_t data);

#endif
