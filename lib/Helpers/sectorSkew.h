#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

//  Function: performSectorSkew
//  Description: Performs the sector skew operation.
//
//  Inputs:
//      sector - The sector to be skewed.
//      skew - The amount of skew to apply.
//      maxSectors - The maximum number of sectors in the disk.
//
//  Outputs: New sector number in the sector variable.
//
//  Returns: None.
void performSectorSkew(uint8_t *sector, uint8_t skew, uint8_t maxSectors);

#ifdef __cplusplus
}
#endif