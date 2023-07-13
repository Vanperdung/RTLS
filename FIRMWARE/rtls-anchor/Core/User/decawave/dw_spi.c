#include "dw_spi.h"

void dwSpiRead(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address, void *data, size_t length)
{
    uint8_t header[3];
    size_t headerLength = 1;
    header[0] = regid & 0x3f;
    if (address != 0)
    {
        header[0] |= 0x40;
        header[1] = address & 0x7f;
        address >>= 7;
        headerLength = 2;
        if (address != 0)
        {
            header[1] |= 0x80;
            header[2] = address & 0xff;
            headerLength = 3;
        }
    }
    dev->func->spiRead(dev, header, headerLength, data, length);
}

uint16_t dwSpiRead16(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address)
{
    uint16_t data;
    dwSpiRead(dev, regid, address, &data, sizeof(data));
    return data;
}

uint32_t dwSpiRead32(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address)
{
    uint32_t data;
    dwSpiRead(dev, regid, address, &data, sizeof(data));
    return data;
}

void dwSpiWrite(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address, const void *data, size_t length)
{
    uint8_t header[3];
    size_t headerLength = 1;
    header[0] = regid & 0x3f;
    header[0] |= 0x80;
    if (address != 0)
    {
        header[0] |= 0x40;
        header[1] = address & 0x7f;
        address >>= 7;
        headerLength = 2;
        if (address != 0)
        {
            header[1] |= 0x80;
            header[2] = address & 0xff;
            headerLength = 3;
        }
    }
    dev->func->spiWrite(dev, header, headerLength, data, length);
}

void dwSpiWrite8(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address, uint8_t data)
{
    dwSpiWrite(dev, regid, address, &data, sizeof(data));
}

void dwSpiWrite32(dwDeviceTypes_t *dev, uint8_t regid, uint32_t address, uint32_t data)
{
    dwSpiWrite(dev, regid, address, &data, sizeof(data));
}
