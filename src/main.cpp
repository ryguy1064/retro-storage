#include <Arduino.h>
#include "ArduinoFDC.h"
#include "ArduDOS.h"
#include "sectorSkew.h"

// -----------------------------------------------------------------
// 82C55 settings:
//   Base port address: 0x10
//   Control byte: 0xC3
//   Mode: 2
//   Port A: Bidirectional bus
//   Port B: General-purpose inputs
//   Port C: Control, Register select
//     0x80: Output buffer full (active-low) - PPI received byte from Z80
//     0x40: Output buffer ACK (active-low) - MCU-controlled - PPI sends byte on Port A to MCU
//     0x20: Input buffer full - HIGH: PPI has byte ready from MCU for Z80 to read: LOW: No byte ready
//     0x10: Strobe input (active-low) - MCU-controlled - PPI latches byte from MCU
//     0x01: Register select - HIGH: Change register pointer (low-byte first, then high-byte) - LOW: Read/write from register
// -----------------------------------------------------------------

/* Z80 test code

; Placed at $F000
LD A,$C2
OUT ($13),A
LD C,$01
LD A,C
OUT ($10),A
IN A,($12)
AND $80
JR Z,$F009
INC C
NOP
JR $F006

LD A,01
OUT (28),A
LD A,00
OUT (28),A
RET

*/

// Debug output
#define INFO_ENABLED        1
#define DEBUG_ENABLED       0
#define VERBOSE_ENABLED     0

// Drive simulation
#define DRIVE_SIM 0

// Buffer R/W performance measurement
#define PERF_MEAS 0

#if DEBUG_ENABLED
#define DBG_PRINT(...)      Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...)    Serial.println(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTLN(...)
#endif

#if VERBOSE_ENABLED
#define VRB_PRINT(...)      Serial.print(__VA_ARGS__)
#define VRB_PRINTLN(...)    Serial.println(__VA_ARGS__)
#else
#define VRB_PRINT(...)
#define VRB_PRINTLN(...)
#endif

#if INFO_ENABLED
#define INF_PRINT(...)      Serial.print(__VA_ARGS__)
#define INF_PRINTLN(...)    Serial.println(__VA_ARGS__)
#else
#define INF_PRINT(...)
#define INF_PRINTLN(...)
#endif

#define DISK_MOTOR_IDLE_TIMEOUT_MS  4000

#define DEFAULT_SECTOR_SKEW 1

// Pins/ports
#define BUS_INTERFACE_DATA_OUT      PORTA
#define BUS_INTERFACE_DATA_DDR      DDRA
#define BUS_INTERFACE_DATA_IN       PINA
#define BUS_INTERFACE_IN_STROBE     16
#define BUS_INTERFACE_IN_BUFFUL     19
#define BUS_INTERFACE_OUT_ACK       17
#define BUS_INTERFACE_OUT_BUFFUL    18
#define BUS_INTERFACE_REG_SEL       38
#define BUS_INTERFACE_REG_SEL_ACK   37
#define BUS_INTERFACE_BUSY          36

#define SET_PIN(port, pinmask) ((port) |= (pinmask))
#define CLEAR_PIN(port, pinmask) ((port) &= ~(pinmask))
#define GET_PIN(port, pinmask) ((port) & (pinmask))

// Z80 -> MCU data full notification (input, active-low)
#define BUS_INTERFACE_OUT_BUFFUL_port           PIND
#define BUS_INTERFACE_OUT_BUFFUL_mask           (1 << 3)

// Z80 -> MCU read data (output, active-low)
#define BUS_INTERFACE_OUT_ACK_port              PORTH
#define BUS_INTERFACE_OUT_ACK_mask              (1 << 0)

// MCU -> Z80 data empty notification (input, active-low)
#define BUS_INTERFACE_IN_BUFFUL_port            PIND
#define BUS_INTERFACE_IN_BUFFUL_mask            (1 << 2)

// MCU -> Z80 write data (output, active-low)
#define BUS_INTERFACE_IN_STROBE_port            PORTH
#define BUS_INTERFACE_IN_STROBE_mask            (1 << 1)

// Z80 -> MCU register select (active-high)
#define BUS_INTERFACE_REG_SEL_port              PIND
#define BUS_INTERFACE_REG_SEL_mask              (1 << 7)

// MCU -> Z80 register select ACK (active-high)
#define BUS_INTERFACE_REG_SEL_ACK_port          PORTC
#define BUS_INTERFACE_REG_SEL_ACK_mask          (1 << 0)

// MCU -> Z80 busy (active-high)
#define BUS_INTERFACE_BUSY_port                 PORTC
#define BUS_INTERFACE_BUSY_mask                 (1 << 1)

// Global variables
static uint16_t gRegisterPointer = 0;
static uint16_t gNewRegisterPointer = 0;
static uint8_t gNewRegisterPointerByteIndex = 0;

// Register map
//
// Per-drive parameters
// 0x00p: Drive A parameters
// 0x01p: Drive B parameters
// 0x02p: Drive C parameters
// 0x03p: Drive D parameters
// p =
//   0x0: Default drive properties (read from HW jumpers) (0 = None, 1 = 5.25" DD, 2 = 5.25" DD in HD drive, 3 = 5.25" HD, 4 = 3.5" DD, 5 = 3.5" HD)
//   0x1: Configured drive properties (overrides default)
//   0x2 - 0x5: Max LBA (512-byte sectors)
//   0x6: Sector skew (within a track) (0-1 = 1 sector, 2 = 2 sectors, 3 = 3 sectors, etc...)
//   0x7 - 0xf: Reserved (0x00)
//
// Common parameters
// 0x040: Drive select (0 = Drive A, 1 = Drive B, 2 = Drive C, 3 = Drive D)
// 0x041 - 0x044: Drive LBA (0x00000000 - 0xFFFFFFFF)
// 0x045: Drive operation (0 = Controller reset, 1 = Read sector from drive into buffer, 2 = Write sector to drive from buffer, 3 = Format (floppy: low-level))
// 0x046: Drive status (0 = No error/Operation complete, 1 = Invalid param, 2 = Write-protected, 3 = Read/write error, 4 = Not ready, 5 = Busy, 6 = Timeout)
// 0x047 - 0x0FF: Reserved (0x00)
//
// R/W Buffer
// 0x100 - 0x2FF: R/W buffer for drive's sector data (512 bytes)
//
// Reserved
// 0x300 - 0xFFFF: Reserved (0x00)

#define REG_ADDR__DRIVE_NUM_MASK 0x030
#define REG_ADDR__DRIVE_NUM_SHIFT 4
#define REG_ADDR__DRIVE_A 0x000
#define REG_ADDR__DRIVE_B 0x010
#define REG_ADDR__DRIVE_C 0x020
#define REG_ADDR__DRIVE_D 0x030

#define REG_ADDR__DRIVE_PROPERTY_MASK 0x00F
#define REG_ADDR__DRIVE_PROPERTY_SHIFT 0
#define REG_ADDR__DRIVE_PROPERTY_DEFAULT 0x000
#define REG_ADDR__DRIVE_PROPERTY_CONFIGURED 0x001
#define REG_ADDR__DRIVE_PROPERTY_MAX_LBA 0x002
#define REG_ADDR__DRIVE_PROPERTY_SECTOR_SKEW 0x006
#define REG_ADDR__DRIVE_PROPERTY_RESERVED 0x007

#define REG_ADDR__DRIVE_SELECT 0x040
#define REG_ADDR__DRIVE_LBA 0x041
#define REG_ADDR__DRIVE_OPERATION 0x045
#define REG_ADDR__DRIVE_STATUS 0x046

#define REG_ADDR__RW_BUFFER 0x100
#define REG_SIZE__RW_BUFFER 512

typedef enum
{
    DRIVE_A = 0,
    DRIVE_B = 1,
    DRIVE_C = 2,
    DRIVE_D = 3,
    DRIVE_MAX
} drive_num_t;

typedef enum
{
    _NOOP = -1,
    _RESET = 0,
    _READ = 1,
    _WRITE = 2,
    _FORMAT = 3
} drive_operation_t;

typedef enum
{
    _NO_ERROR = 0,
    _INVALID_PARAM = 1,
    _WRITE_PROTECTED = 2,
    _READ_WRITE_ERROR = 3,
    _NOT_READY = 4,
    _BUSY = 5,
    _TIMEOUT = 6
} drive_status_t;

typedef enum
{
    _NONE = 0,
    _5_25_DD = 1,
    _5_25_DD_in_HD_drive = 2,
    _5_25_HD = 3,
    _3_5_DD = 4,
    _3_5_HD = 5
} drive_type_t;

const ArduinoFDCClass::DriveType cRSToArduinoFDCDriveTypeMap[] =
{
    ArduinoFDCClass::DT_5_DD,
    ArduinoFDCClass::DT_5_DD,
    ArduinoFDCClass::DT_5_DDonHD,
    ArduinoFDCClass::DT_5_HD,
    ArduinoFDCClass::DT_3_DD,
    ArduinoFDCClass::DT_3_HD
};

typedef struct
{
    drive_type_t def;
    drive_type_t configured;
    uint32_t maxLBA;
    uint8_t sectorSkew;
} drive_properties_t;

drive_properties_t gDriveProperties[4] = 
{
    { _3_5_HD, _3_5_HD, (1440UL*1024/512), 2 },
    { _5_25_DD_in_HD_drive, _5_25_DD_in_HD_drive, (360UL*1024/512), 2 },
    { _NONE, _NONE, 0x00000000, 0 },
    { _NONE, _NONE, 0x00000000, 0 }
};

drive_num_t gDriveSelect = DRIVE_A;
uint32_t gDriveLBA = 0x00000000;
drive_operation_t gDriveOperation = _NOOP;
drive_status_t gDriveStatus = _NO_ERROR;
bool gIsRegisterSelectAsserted = false;
uint32_t gDiskMotorStartTime;

uint8_t gFdcBuffer[516] = { 0x00 };
uint8_t *gRwBuffer = &gFdcBuffer[1];

#if DRIVE_SIM
uint8_t gTestSector[516] = { 0x00 };    // Used for test/simulation
#endif

#if PERF_MEAS
uint32_t gTime;
#endif

inline bool isRegisterSelectPinAsserted()
{
    return GET_PIN(BUS_INTERFACE_REG_SEL_port, BUS_INTERFACE_REG_SEL_mask);
}

inline void setDataBusToInput()
{
    BUS_INTERFACE_DATA_DDR = 0x00;
}

// MCU -> Z80 write data sequence
// 1	Z80 waits for PPI input latch buffer full pin to be HIGH (data from MCU is available)
// 2	Z80 reads data from PPI's input latch
// 3	PPI clears input latch buffer full pin
// 4	MCU sees PPI input latch buffer full pin change to LOW
// 5	MCU increments register pointer
// 6	MCU reads data at register pointer and writes to the PPI's input latch using strobe
// 7	PPI sets input latch buffer pin

inline bool isZ80ReadCompleted()
{
    return !GET_PIN(BUS_INTERFACE_IN_BUFFUL_port, BUS_INTERFACE_IN_BUFFUL_mask);
}

inline void writeDataToZ80(uint8_t data)
{
    BUS_INTERFACE_DATA_DDR = 0xFF;  // Data bus is now output
    BUS_INTERFACE_DATA_OUT = data;

    // Strobe the data to the bus interface
    noInterrupts();
    CLEAR_PIN(BUS_INTERFACE_IN_STROBE_port, BUS_INTERFACE_IN_STROBE_mask);
    SET_PIN(BUS_INTERFACE_IN_STROBE_port, BUS_INTERFACE_IN_STROBE_mask);
    interrupts();

    BUS_INTERFACE_DATA_DDR = 0x00; // Data bus is now input
}

// Z80 -> MCU write data sequence
// 1	Z80 writes data to PPI output latch
// 2	Z80 waits for PPI output latch buffer full pin (active-low) to be HIGH (data was read by MCU)
// 3	PPI clears output latch buffer full pin (active-low)
// 4	MCU sees PPI output latch buffer full pin (active-low) change to LOW
// 5	MCU read data at (register pointer+1) and writes to the PPI's input latch using strobe
// 6	MCU reads data from PPI output latch and writes it to data at register pointer
// 7	PPI sets output latch buffer full pin (active-low)
// 8	MCU increments register pointer

inline bool isZ80WriteDataAvailable()
{
    return !GET_PIN(BUS_INTERFACE_OUT_BUFFUL_port, BUS_INTERFACE_OUT_BUFFUL_mask);
}

inline uint8_t getDataFromZ80()
{
    uint8_t b;

    // THIS SECTION IS TIMING CRITICAL! IT MUST COMPLETE AS FAST AS POSSIBLE!
    {
        noInterrupts();
        CLEAR_PIN(BUS_INTERFACE_OUT_ACK_port, BUS_INTERFACE_OUT_ACK_mask);
        // Allow PPI time to output data on Port A (MCU is too fast without this delay)
        __asm__ __volatile__ (
        "nop" "\n\t"
        "nop"); // 2 cycles
        b = (uint8_t)BUS_INTERFACE_DATA_IN;
        SET_PIN(BUS_INTERFACE_OUT_ACK_port, BUS_INTERFACE_OUT_ACK_mask);
        interrupts();
    }

    return b;
}

uint16_t getNextRegisterPointer(uint16_t p)
{
    // Jump to R/W buffer
    if (p == REG_ADDR__DRIVE_STATUS)
    {
        p = REG_ADDR__RW_BUFFER;
    }

    // Wrap around back to beginning of registers
    else if (p == (REG_ADDR__RW_BUFFER + REG_SIZE__RW_BUFFER))
    {
        p = REG_ADDR__DRIVE_A;
    }

    // Otherwise, increment register pointer
    else
    {
        p++;
    }

    return p;
}

uint8_t getRegisterData(uint16_t p)
{
    uint8_t data;

    // Per-drive parameters
    if ((p >= REG_ADDR__DRIVE_A) && (p < REG_ADDR__DRIVE_SELECT))
    {
        drive_num_t drive = (drive_num_t)((p & REG_ADDR__DRIVE_NUM_MASK) >> REG_ADDR__DRIVE_NUM_SHIFT);
        uint8_t property = (p & REG_ADDR__DRIVE_PROPERTY_MASK);
        drive_properties_t *propertyData = &gDriveProperties[drive];

        switch (property)
        {
        case REG_ADDR__DRIVE_PROPERTY_DEFAULT:
            data = (uint8_t)propertyData->def;
            break;

        case REG_ADDR__DRIVE_PROPERTY_CONFIGURED:
            data = (uint8_t)propertyData->configured;
            break;

        case REG_ADDR__DRIVE_PROPERTY_MAX_LBA+0:
            data = (uint8_t)((propertyData->maxLBA & 0x000000FF) >> 0);
            break;

        case REG_ADDR__DRIVE_PROPERTY_MAX_LBA+1:
            data = (uint8_t)((propertyData->maxLBA & 0x0000FF00) >> 8);
            break;

        case REG_ADDR__DRIVE_PROPERTY_MAX_LBA+2:
            data = (uint8_t)((propertyData->maxLBA & 0x00FF0000) >> 16);
            break;

        case REG_ADDR__DRIVE_PROPERTY_MAX_LBA+3:
            data = (uint8_t)((propertyData->maxLBA & 0xFF000000) >> 24);

        case REG_ADDR__DRIVE_PROPERTY_SECTOR_SKEW:
            data = propertyData->sectorSkew;
            break;

        default:
            data = 0;
            break;
        }
    }

    // Common parameters
    else if ((p >= REG_ADDR__DRIVE_SELECT) && (p < REG_ADDR__RW_BUFFER))
    {
        switch (p)
        {
        case REG_ADDR__DRIVE_SELECT:
            data = (uint8_t)gDriveSelect;
            break;

        case REG_ADDR__DRIVE_LBA+0:
            data = (uint8_t)(gDriveLBA & 0x000000FF);
            break;

        case REG_ADDR__DRIVE_LBA+1:
            data = (uint8_t)((gDriveLBA & 0x0000FF00) >> 8);
            break;

        case REG_ADDR__DRIVE_LBA+2:
            data = (uint8_t)((gDriveLBA & 0x00FF0000) >> 16);
            break;

        case REG_ADDR__DRIVE_LBA+3:
            data = (uint8_t)((gDriveLBA & 0xFF000000) >> 24);
            break;

        case REG_ADDR__DRIVE_OPERATION:
            data = (uint8_t)gDriveOperation;
            break;

        case REG_ADDR__DRIVE_STATUS:
            data = (uint8_t)gDriveStatus;
            break;

        default:
            data = 0;
            break;
        }
    }

    // Reading from buffer
    else if ((p >= REG_ADDR__RW_BUFFER) && (p < (REG_ADDR__RW_BUFFER + REG_SIZE__RW_BUFFER)))
    {
        data = gRwBuffer[p - REG_ADDR__RW_BUFFER];
    }

    // Unknown
    else
    {
        data = 0;
    }

    return data;
}

void setRegisterData(uint16_t p, uint8_t data)
{
    // Per-drive parameters
    if ((p >= REG_ADDR__DRIVE_A) && (p < REG_ADDR__DRIVE_SELECT))
    {
        drive_num_t drive = (drive_num_t)((p & REG_ADDR__DRIVE_NUM_MASK) >> REG_ADDR__DRIVE_NUM_SHIFT);
        uint8_t property = (p & REG_ADDR__DRIVE_PROPERTY_MASK);
        drive_properties_t *propertyData = &gDriveProperties[drive];

        switch (property)
        {
        case REG_ADDR__DRIVE_PROPERTY_CONFIGURED:
            propertyData->configured = (drive_type_t)data;
            break;

        case REG_ADDR__DRIVE_PROPERTY_SECTOR_SKEW:
            propertyData->sectorSkew = data;
            break;

        default:
            // Do nothing
            break;
        }
    }

    // Common parameters
    else if ((p >= REG_ADDR__DRIVE_SELECT) && (p < REG_ADDR__RW_BUFFER))
    {
        switch (p)
        {
        case REG_ADDR__DRIVE_SELECT:
            gDriveSelect = (drive_num_t)data;
            gDriveStatus = _BUSY;
            break;

        case REG_ADDR__DRIVE_LBA+0:
            gDriveLBA = data;
            // INF_PRINT(data, HEX);
            // INF_PRINT(' ');
            // gDriveStatus = _BUSY;
            break;

        case REG_ADDR__DRIVE_LBA+1:
            gDriveLBA |= ((uint32_t)data << 8);
            // INF_PRINT(data, HEX);
            // INF_PRINT(' ');
            // gDriveStatus = _BUSY;
            break;

        case REG_ADDR__DRIVE_LBA+2:
            gDriveLBA |= ((uint32_t)data << 16);
            // INF_PRINT(data, HEX);
            // INF_PRINT(' ');
            // gDriveStatus = _BUSY;
            break;

        case REG_ADDR__DRIVE_LBA+3:
            gDriveLBA |= ((uint32_t)data << 24);
            // INF_PRINT(data, HEX);
            // INF_PRINT(" -> ");
            // gDriveStatus = _BUSY;
            break;

        case REG_ADDR__DRIVE_OPERATION:
            gDriveOperation = (drive_operation_t)data;
            // gDriveStatus = _BUSY;
            break;

        case REG_ADDR__DRIVE_STATUS:
            // Do nothing (drive status is read-only)
            break;

        default:
            // Do nothing (reserved)
            break;
        }
    }

    // Writing to buffer
    else if ((p >= REG_ADDR__RW_BUFFER) && (p < (REG_ADDR__RW_BUFFER + REG_SIZE__RW_BUFFER)))
    {
        // Write data to buffer
        gRwBuffer[p - REG_ADDR__RW_BUFFER] = data;
    }
}

// RJW: This mapping matches standard Floppies formatted in Windows (when no skew is used)
void lbaToTrackHeadSector(uint16_t targetLba, uint8_t *diskTrack, uint8_t *diskHead, uint8_t *diskSector, uint8_t sectorSkew)
{
    uint8_t numSectors = ArduinoFDC.numSectors();
    uint8_t numSectorsX2 = numSectors * 2;

    *diskHead = 0;
    *diskTrack = (uint8_t)(targetLba / (uint16_t)numSectorsX2);
    *diskSector = (uint8_t)(targetLba % (uint16_t)numSectorsX2);
    if (*diskSector >= numSectors)
    {
        *diskHead = 1;
        *diskSector -= numSectors;
    }
    *diskSector += 1;

    #if 0
    // Provide sector skew for performance
    const uint8_t skewMap1[]  = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
    const uint8_t skewMap2[]  = {1, 3, 5, 7, 9, 11, 13, 15, 17, 2, 4, 6, 8, 10, 12, 14, 16, 18};
    const uint8_t skewMap3[]  = {1, 4, 7, 10, 13, 16, 2, 5, 8, 11, 14, 17, 3, 6, 9, 12, 15, 18};
    const uint8_t skewMap6[]  = {1, 7, 13, 2, 8, 14, 3, 9, 15, 4, 10, 16, 5, 11, 17, 6, 12, 18};
    const uint8_t skewMap12[] = {1, 13, 8, 3, 15, 10, 5, 17, 12, 7, 2, 14, 9, 4, 16, 11, 6, 18};
    const uint8_t skewMap13[] = {1, 14, 10, 6, 2, 15, 11, 7, 3, 16, 12, 8, 4, 17, 13, 9, 5, 18};
    const uint8_t *chosenSkewMap = skewMap2;
    *diskSector = chosenSkewMap[(*diskSector)-1];
    #else
    performSectorSkew(diskSector, sectorSkew, numSectors);
    #endif

    INF_PRINT(F(" -> CHS: "));
    INF_PRINT(*diskTrack);
    INF_PRINT(' ');
    INF_PRINT(*diskHead);
    INF_PRINT(' ');
    INF_PRINT(*diskSector);
}

/**
 * @brief Initializes the bus interface between the MCU and Z80 processor.
 * 
 * This function sets up the necessary pins and configurations for the communication
 * between the MCU and Z80 processor. It configures the data direction register for
 * the bus interface data pins, initializes the control pins for both the MCU to Z80
 * and Z80 to MCU communication, and starts the serial communication interface.
 */
void setup() {
    setDataBusToInput();

    // Initialize MCU -> Z80 control pins
    pinMode(BUS_INTERFACE_IN_STROBE, OUTPUT);
    digitalWrite(BUS_INTERFACE_IN_STROBE, 1);
    pinMode(BUS_INTERFACE_IN_BUFFUL, INPUT_PULLUP);

    // Initialize Z80 -> MCU control pins
    pinMode(BUS_INTERFACE_OUT_ACK, OUTPUT);
    digitalWrite(BUS_INTERFACE_OUT_ACK, 1);
    pinMode(BUS_INTERFACE_OUT_BUFFUL, INPUT_PULLUP);

    // Initialize Z80 -> MCU register select
    pinMode(BUS_INTERFACE_REG_SEL, INPUT_PULLUP);
    pinMode(BUS_INTERFACE_REG_SEL_ACK, OUTPUT);
    digitalWrite(BUS_INTERFACE_REG_SEL_ACK, 0);

    // Initialize busy
    pinMode(BUS_INTERFACE_BUSY, OUTPUT);
    digitalWrite(BUS_INTERFACE_BUSY, 0);

    Serial.begin(115200);

    delayMicroseconds(2);

    INF_PRINTLN("retro-storage v0.1");

    // Prime the first register data
    //   Read data at register pointer and write it onto the bus interface
    writeDataToZ80(getRegisterData(gRegisterPointer));

#if DRIVE_SIM
    // Populate test sector
    for (uint16_t i = 0; i < sizeof(gTestSector); i++)
    {
        gTestSector[i] = i;
    }
#else
    // Initialize ArduinoFDC
    ArduinoFDC.begin(
        cRSToArduinoFDCDriveTypeMap[gDriveProperties[0].def],
        cRSToArduinoFDCDriveTypeMap[gDriveProperties[1].def]);
#endif

    // // Print current contents of gDriveProperties
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     DBG_PRINT("Drive ");
    //     DBG_PRINT(i);
    //     DBG_PRINT(": ");
    //     DBG_PRINT(gDriveProperties[i].def);
    //     DBG_PRINT(" ");
    //     DBG_PRINT(gDriveProperties[i].configured);
    //     DBG_PRINT(" ");
    //     DBG_PRINTLN(gDriveProperties[i].maxLBA);
    // }
}

uint32_t gLastTime = 0;
String serialCmd;

void loop() {
    if (Serial.available())
    {
        // Serial.println("Char detected");
        serialCmd = Serial.readString();
        Serial.println(serialCmd);
        serialCmd.trim();
        if (serialCmd == "dos")
        {
        Serial.println(F("Entering ArduDOS"));
        arduDOS();
        }
    }

    // Check for register select assertion
    if (isRegisterSelectPinAsserted() && !gIsRegisterSelectAsserted)
    {
        gIsRegisterSelectAsserted = true;
        gNewRegisterPointerByteIndex = 0;

        // Signal register select ACK
        SET_PIN(BUS_INTERFACE_REG_SEL_ACK_port, BUS_INTERFACE_REG_SEL_ACK_mask);

        VRB_PRINTLN("Register select asserted");
    }
    else if (!isRegisterSelectPinAsserted() && gIsRegisterSelectAsserted)
    {
        gIsRegisterSelectAsserted = false;

        // Signal register select de-ACK
        CLEAR_PIN(BUS_INTERFACE_REG_SEL_ACK_port, BUS_INTERFACE_REG_SEL_ACK_mask);

        VRB_PRINTLN("Register select deasserted");
    }

    // Received data from Z80
    if (isZ80WriteDataAvailable())
    {
        // If register select is set, then set the register pointer at the specified byte index
        if (gIsRegisterSelectAsserted)
        {
            // Get byte from PPI
            uint8_t data = getDataFromZ80();

            // Set LSB
            if (gNewRegisterPointerByteIndex == 0)
            {
                VRB_PRINT("reg ptr LSB set to ");
                VRB_PRINTLN(data, HEX);
                gNewRegisterPointer = data;
                gNewRegisterPointerByteIndex++;
            }

            // Set MSB
            else if (gNewRegisterPointerByteIndex == 1)
            {
                gNewRegisterPointer |= (uint16_t)data << 8;
                VRB_PRINT("reg ptr MSB set to ");
                VRB_PRINTLN(data, HEX);

                VRB_PRINT("reg ptr = ");
                VRB_PRINTLN(gNewRegisterPointer, HEX);
                gRegisterPointer = gNewRegisterPointer;
                gNewRegisterPointerByteIndex++;

                // Prime the register data
                writeDataToZ80(getRegisterData(gRegisterPointer));
            }

            // All further bytes are ignored (can be used by Z80 to ensure register read data is valid for new register pointer)
            else
            {
                VRB_PRINTLN("reg ptr dummy");
            }
        }

        // Otherwise, get Z80's write data and write it to the register
        else
        {
            // Required sequence of operations:

            //   Determine next register pointer
            uint16_t nextRegisterPointer = getNextRegisterPointer(gRegisterPointer);

            //   Get data at NEXT register pointer address
            uint8_t data = getRegisterData(nextRegisterPointer);

            //   Write data at NEXT register back to Z80 (will overwrite what's in the bus interface if there's data already there)
            writeDataToZ80(data);

            //   Get Z80 data and write it to the current register pointer (this action will unblock the Z80)
            data = getDataFromZ80();
            setRegisterData(gRegisterPointer, data);

            VRB_PRINT("write to reg ");
            VRB_PRINT(gRegisterPointer, HEX);
            VRB_PRINT(": ");
            VRB_PRINTLN(data, HEX);

            //   Move to the next register pointer
            gRegisterPointer = nextRegisterPointer;
        }
    }

    // Perform drive operartion if it's set to something other than NOOP
    if (gDriveOperation != _NOOP)
    {
        gLastTime = micros();

        // Set busy
        SET_PIN(BUS_INTERFACE_BUSY_port, BUS_INTERFACE_BUSY_mask);
        gDriveStatus = _BUSY;

        do
        {
        #if !DRIVE_SIM
            byte fdcStatus = _NOT_READY;
            uint8_t track, sector, side;
        #endif

            // Select drive
            if (gDriveSelect == DRIVE_A)
            {
                ArduinoFDC.selectDrive(0);
                ArduinoFDC.setDriveType(cRSToArduinoFDCDriveTypeMap[gDriveProperties[0].configured]);
            }
            else if (gDriveSelect == DRIVE_B)
            {
                ArduinoFDC.selectDrive(1);
                ArduinoFDC.setDriveType(cRSToArduinoFDCDriveTypeMap[gDriveProperties[1].configured]);
            }
            else
            {
                gDriveStatus = _INVALID_PARAM;
                break;
            }

            // Check drive read/write action parameters
            if (gDriveOperation == _READ || gDriveOperation == _WRITE)
            {
                //   Drive select
                if (gDriveSelect >= DRIVE_MAX)
                {
                    INF_PRINTLN("Invalid drive select");
                    gDriveStatus = _INVALID_PARAM;
                    break;
                }

                //   LBA
                if (gDriveLBA >= gDriveProperties[gDriveSelect].maxLBA)
                {
                    INF_PRINT("Invalid LBA ");
                    INF_PRINTLN(gDriveLBA);
                    gDriveStatus = _INVALID_PARAM;
                    break;
                }
            }

            // Turn on motor early to avoid delay in ArduinoFDC code
            gDiskMotorStartTime = millis();
            ArduinoFDC.motorOn();

            // Check drive write protection
            if ((gDriveOperation == _WRITE) || (gDriveOperation == _FORMAT))
            {
                if (ArduinoFDC.isWriteProtected())
                {
                    INF_PRINTLN("Write protected");
                    gDriveStatus = _WRITE_PROTECTED;
                    break;
                }
            }

            // Perform drive action, saving drive operation status (NOTE: status was already set to busy earlier)
            INF_PRINT("DRIVE ");
            INF_PRINT(gDriveSelect);
            INF_PRINT(" OP: ");
            switch (gDriveOperation)
            {
            case _READ:
            #if DRIVE_SIM
                INF_PRINT("SIM: reading from drive...");
                memcpy(gRwBuffer, gTestSector, sizeof(gTestSector));
                gDriveStatus = _NO_ERROR;
            #else
                INF_PRINT("READ LBA ");
                INF_PRINT(gDriveLBA);
                lbaToTrackHeadSector(gDriveLBA, &track, &side, &sector, gDriveProperties[gDriveSelect].sectorSkew);
                fdcStatus = ArduinoFDC.readSector(track, side, sector, gFdcBuffer);
            #endif
                break;

            case _WRITE:
            #if DRIVE_SIM
                INF_PRINT("SIM: writing to drive...");
                memcpy(gTestSector, gRwBuffer, sizeof(gTestSector));
                gDriveStatus = _NO_ERROR;
            #else
                INF_PRINT("WRITE LBA ");
                INF_PRINT(gDriveLBA);
                lbaToTrackHeadSector(gDriveLBA, &track, &side, &sector, gDriveProperties[gDriveSelect].sectorSkew);
                fdcStatus = ArduinoFDC.writeSector(track, side, sector, gFdcBuffer, false);
            #endif
                break;

            case _FORMAT:
            #if DRIVE_SIM
                INF_PRINT("SIM: formatting drive...");
                memset(gTestSector, 0xEE, sizeof(gTestSector));
                gDriveStatus = _NO_ERROR;
            #else
                INF_PRINT("FORMAT ");
                fdcStatus = ArduinoFDC.formatDisk(gFdcBuffer);
            #endif
                break;

            case _RESET:
            #if DRIVE_SIM
                INF_PRINT("SIM: resetting drive...");
                gDriveStatus = _NO_ERROR;
            #else
                // Nothing to do
                INF_PRINT("RESET ");
                gDriveStatus = _NO_ERROR;
            #endif
                break;

            default:
            #if DRIVE_SIM
                INF_PRINTLN("SIM: invalid drive operation");
            #else
                INF_PRINT("invalid drive operation ");
            #endif
                break;
            }

        #if DRIVE_SIM
            #define SIM_DRIVE_OP_TIME_SEC 3
            for (uint8_t i = 0; i < SIM_DRIVE_OP_TIME_SEC; i++)
            {
                INF_PRINT((SIM_DRIVE_OP_TIME_SEC-i));
                INF_PRINT(" ");
                delay(1000);
            }
        #else
            if (fdcStatus != S_OK)
            {
                INF_PRINT(" FDC status: ");
                INF_PRINT(fdcStatus);
            }

            // Convert ArduinoFDC status to drive status
            switch (fdcStatus)
            {
            case S_OK:
                gDriveStatus = _NO_ERROR;
                break;

            case S_NOTINIT:
            case S_NOTREADY:
                gDriveStatus = _NOT_READY;
                break;

            case S_NOSYNC:
            case S_NOHEADER:
            case S_INVALIDID:
            case S_CRC:
            case S_NOTRACK0:
            case S_VERIFY:
                gDriveStatus = _READ_WRITE_ERROR;
                break;

            case S_READONLY:
                gDriveStatus = _WRITE_PROTECTED;
                break;

            default:
                gDriveStatus = _NOT_READY;
                break;
            }
        #endif

            INF_PRINT(" Done");
        } while (0);

        // Save drive operation status so Z80 will read it when un-busied
        writeDataToZ80(gDriveStatus);

        // Clear busy (Z80 should be waiting for this to clear)
        CLEAR_PIN(BUS_INTERFACE_BUSY_port, BUS_INTERFACE_BUSY_mask);

        INF_PRINT(" *");
        INF_PRINTLN(micros() - gLastTime);

        // Reset drive operation back to idle
        gDriveOperation = _NOOP;
    }

    // Request for more data from MCU
    if (isZ80ReadCompleted())
    {
        // Clear data from PPI if register select is set
        // (don't reload the bus interface with new data)
        if (gIsRegisterSelectAsserted)
        {
            VRB_PRINTLN("Clear data from PPI");

            // Reset the new register pointer byte index
            gNewRegisterPointerByteIndex = 0;
        }

        // Otherwise, get data at next register pointer address and write it onto the bus interface
        else
        {
            VRB_PRINT("Z80 read from reg ");
            VRB_PRINTLN(gRegisterPointer, HEX);

        #if PERF_MEAS
            // Performance measurement
            if (gRegisterPointer == (REG_ADDR__RW_BUFFER + REG_SIZE__RW_BUFFER - 1))
            {
                uint32_t totalTime = micros() - gTime;

                DBG_PRINT("Full buffer read time (us): ");
                DBG_PRINTLN(totalTime);
            }
        #endif

            // Move to the next register
            gRegisterPointer = getNextRegisterPointer(gRegisterPointer);

            // Read data at NEXT register pointer and write it onto the bus interface
            uint8_t data = getRegisterData(gRegisterPointer);
            writeDataToZ80(data);

        #if PERF_MEAS
            // Performance measurement
            if (gRegisterPointer-1 == REG_ADDR__RW_BUFFER)
            {
                gTime = micros();
            }
        #endif

            VRB_PRINT("read from next reg ");
            VRB_PRINT(gRegisterPointer, HEX);
            VRB_PRINT(": ");
            VRB_PRINTLN(data, HEX);
        }
    }

    // Turn off disk motor after idle timeout
    if (ArduinoFDC.motorRunning() && ((millis() - gDiskMotorStartTime) >= DISK_MOTOR_IDLE_TIMEOUT_MS))
    {
        DBG_PRINTLN(F("Disk motor off (Idle timeout)"));
        ArduinoFDC.motorOff();
    }
}