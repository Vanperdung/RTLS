#include "flash_storage.h"
#include "main.h"

uint32_t getSector(uint32_t address)
{
    uint32_t sector = 0;
    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if (address >= ADDR_FLASH_SECTOR_7)
    {
        sector = FLASH_SECTOR_7;
    }
    return sector;
}

uint32_t flashWriteData(uint32_t startAddress, uint8_t *data, uint32_t num)
{
    static FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t cnt = 0;
    uint32_t sectorError;
    uint32_t sector = getSector(startAddress);
    HAL_FLASH_Unlock();
    FLASH_Erase_Sector(sector, FLASH_VOLTAGE_RANGE_3);
    HAL_FLASH_Lock();

    HAL_FLASH_Unlock();
    while (cnt < num)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, startAddress, data[cnt]) == HAL_OK)
        {
        	startAddress++;
            cnt++;
        }
        else
        {
            HAL_FLASH_GetError();
        }
    }
    HAL_FLASH_Lock();
    return 0;
}

void flashReadData(uint32_t startAddress, uint8_t *data, uint32_t num)
{
    while (num)
    {
        *data = *(__IO uint8_t *)startAddress;
        startAddress++;
        data++;
        num--;
    }
}
