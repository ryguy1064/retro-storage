#include "Arduino.h"
#include "sectorSkew.h"

void performSectorSkew(uint8_t *sector, uint8_t skew, uint8_t maxSectors)
{
    if (skew == 0 || skew == 1)
    {
        // No sector adjustment
        return;
    }
    else
    {
        uint8_t newSector = 1;
        bool isEvenMaxSectors = (maxSectors % 2 == 0);

        for (uint8_t i = 0; i < (*sector)-1; i++)
        {
            newSector += skew;
            if (newSector > maxSectors)
            {
                newSector -= maxSectors;
                if (isEvenMaxSectors)
                {
                    newSector++;
                }
            }
        }
        *sector = newSector;
    }
}
