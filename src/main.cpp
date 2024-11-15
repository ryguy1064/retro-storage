#include <Arduino.h>

// -----------------------------------------------------------------
// Current 82C55 I/O port address base: 0x10
// -----------------------------------------------------------------

// Debug output
#define DEBUG_ENABLED 1

#if DEBUG_ENABLED
#define DBG_PRINT(...)      Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...)    Serial.println(__VA_ARGS__)
#else

#endif

// Pins/ports
#define BUS_INTERFACE_DATA_OUT      PORTA
#define BUS_INTERFACE_DATA_DDR      DDRA
#define BUS_INTERFACE_DATA_IN       PINA
#define BUS_INTERFACE_IN_STROBE     16
#define BUS_INTERFACE_IN_BUFFUL     19
#define BUS_INTERFACE_OUT_ACK       17
#define BUS_INTERFACE_OUT_BUFFUL    18
#define BUS_INTERFACE_REG_SEL       38

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

// Z80 -> MCU register select
#define BUS_INTERFACE_REG_SEL_port              PORTD
#define BUS_INTERFACE_REG_SEL_mask              (1 << 7)

// Global variables
static uint16_t registerPointer = 0;
static uint16_t newRegisterPointer = 0;
static uint8_t newRegisterPointerByteIndex = 0;

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
//   0x6 - 0xf: Reserved (0x00000000)
//
// Common parameters
// 0x040: Drive select (0 = Drive A, 1 = Drive B, 2 = Drive C, 3 = Drive D)
// 0x041 - 0x044: Drive LBA (0x00000000 - 0xFFFFFFFF)
// 0x045: Drive operation (0 = Controller reset, 1 = Read sector from drive into buffer, 2 = Write sector to drive from buffer, 3 = Format (floppy: low-level))
// 0x046: Drive status (0 = No error, 1 = Invalid param, 2 = Write-protected, 3 = Read/write error, 4 = Not ready, 5 = Timeout)
// 
// 0x100 - 0x2FF: R/W buffer for drive's sector data (512 bytes)

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
#define REG_ADDR__DRIVE_PROPERTY_RESERVED 0x006

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
    DRIVE_D = 3
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
    _TIMEOUT = 5
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

typedef struct
{
    drive_type_t def;
    drive_type_t configured;
    uint32_t maxLBA;
} drive_properties_t;

drive_properties_t driveProperties[4] = 
{
    { _3_5_HD, _3_5_HD, (1440UL*1024/512) },
    { _5_25_DD_in_HD_drive, _5_25_DD_in_HD_drive, (320UL*1024/512) },
    { _NONE, _NONE, 0x00000000 },
    { _NONE, _NONE, 0x00000000 }
};

bool readDataNeedsUpdate = false;
drive_num_t driveSelect = DRIVE_A;
uint32_t driveLBA = 0x00000000;
drive_operation_t driveOperation = _NOOP;
drive_status_t driveStatus = _NO_ERROR;

uint8_t rwBuffer[REG_SIZE__RW_BUFFER] = { 0x00 };

inline bool isRegisterSelectPointer()
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
    {
        p++;
    }

    return p;
}

uint8_t getRegisterData(uint16_t p)
{
    uint8_t data;

    if ((p >= REG_ADDR__DRIVE_A) && (p < REG_ADDR__DRIVE_SELECT))
    {
        drive_num_t drive = (drive_num_t)((p & REG_ADDR__DRIVE_NUM_MASK) >> REG_ADDR__DRIVE_NUM_SHIFT);
        uint8_t property = (p & REG_ADDR__DRIVE_PROPERTY_MASK);

        switch (property)
        {
        case REG_ADDR__DRIVE_PROPERTY_DEFAULT:
            data = (uint8_t)driveProperties[drive].def;
            break;

        case REG_ADDR__DRIVE_PROPERTY_CONFIGURED:
            data = (uint8_t)driveProperties[drive].configured;
            break;

        case REG_ADDR__DRIVE_PROPERTY_MAX_LBA+0:
            data = (uint8_t)(driveProperties[drive].maxLBA & 0x000000FF);
            break;

        case REG_ADDR__DRIVE_PROPERTY_MAX_LBA+1:
            data = (uint8_t)((driveProperties[drive].maxLBA & 0x0000FF00) >> 8);
            break;

        case REG_ADDR__DRIVE_PROPERTY_MAX_LBA+2:
            data = (uint8_t)((driveProperties[drive].maxLBA & 0x00FF0000) >> 16);
            break;

        case REG_ADDR__DRIVE_PROPERTY_MAX_LBA+3:
            data = (uint8_t)((driveProperties[drive].maxLBA & 0xFF000000) >> 24);

        default:
            data = 0;
            break;
        }
    }

    else if ((p >= REG_ADDR__DRIVE_SELECT) && (p <= REG_ADDR__DRIVE_STATUS))
    {
        switch (p)
        {
        case REG_ADDR__DRIVE_SELECT:
            data = (uint8_t)driveSelect;
            break;

        case REG_ADDR__DRIVE_LBA+0:
            data = (uint8_t)(driveLBA & 0x000000FF);
            break;

        case REG_ADDR__DRIVE_LBA+1:
            data = (uint8_t)((driveLBA & 0x0000FF00) >> 8);
            break;

        case REG_ADDR__DRIVE_LBA+2:
            data = (uint8_t)((driveLBA & 0x00FF0000) >> 16);
            break;

        case REG_ADDR__DRIVE_LBA+3:
            data = (uint8_t)((driveLBA & 0xFF000000) >> 24);
            break;

        case REG_ADDR__DRIVE_OPERATION:
            data = (uint8_t)driveOperation;
            break;

        case REG_ADDR__DRIVE_STATUS:
            data = (uint8_t)driveStatus;
            break;

        default:
            data = 0;
            break;
        }
    }

    // Reading from buffer
    else
    {
        data = rwBuffer[p - REG_ADDR__RW_BUFFER];
    }

    return data;
}

void setRegisterData(uint16_t p, uint8_t data)
{
    if ((p >= REG_ADDR__DRIVE_A) && (p < REG_ADDR__DRIVE_SELECT))
    {
        drive_num_t drive = (drive_num_t)((p & REG_ADDR__DRIVE_NUM_MASK) >> REG_ADDR__DRIVE_NUM_SHIFT);
        uint8_t property = (p & REG_ADDR__DRIVE_PROPERTY_MASK);

        switch (property)
        {
        case REG_ADDR__DRIVE_PROPERTY_CONFIGURED:
        {
            driveProperties[drive].configured = (drive_type_t)data;
            break;
        }

        default:
            // Do nothing
            break;
        }
    }

    else if ((p >= REG_ADDR__DRIVE_SELECT) && (p <= REG_ADDR__DRIVE_STATUS))
    {
        switch (p)
        {
        case REG_ADDR__DRIVE_SELECT:
            driveSelect = (drive_num_t)data;
            break;

        case REG_ADDR__DRIVE_LBA+0:
            driveLBA = data;
            break;

        case REG_ADDR__DRIVE_LBA+1:
            driveLBA |= ((uint32_t)data << 8);
            break;

        case REG_ADDR__DRIVE_LBA+2:
            driveLBA |= ((uint32_t)data << 16);
            break;

        case REG_ADDR__DRIVE_LBA+3:
            driveLBA |= ((uint32_t)data << 24);
            break;

        case REG_ADDR__DRIVE_OPERATION:
            driveOperation = (drive_operation_t)data;
            break;

        default:
            break;
        }
    }

    // Writing to buffer
    else
    {
        // Write data to buffer
        rwBuffer[p - REG_ADDR__RW_BUFFER] = data;
    }
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

#if DEBUG_ENABLED
    Serial.begin(115200);
#endif

    DBG_PRINTLN("retro-storage v0.1");

    // Prime the first register data
    //   Read data at register pointer and write it onto the bus interface
    writeDataToZ80(getRegisterData(registerPointer));

    //   Move to the next register
    registerPointer = getNextRegisterPointer(registerPointer);
}

void loop() {
    // Received data from Z80
    if (isZ80WriteDataAvailable())
    {
        // If register select is set, then set the register pointer at the specified byte index
        if (isRegisterSelectPointer())
        {
            uint16_t *np = &newRegisterPointer;

            // Reset the new register pointer byte index if receiving the LSB
            if (newRegisterPointerByteIndex == 0)
            {
                DBG_PRINTLN("reg ptr LSB set");
                newRegisterPointer = 0;
            }

            np[newRegisterPointerByteIndex] = getDataFromZ80();
            newRegisterPointerByteIndex++;

            // Apply new register pointer value on receiving the MSB
            if (newRegisterPointerByteIndex >= 2)
            {
                DBG_PRINT("reg ptr = ");
                DBG_PRINTLN(newRegisterPointer, HEX);
                registerPointer = newRegisterPointer;
                newRegisterPointerByteIndex = 0;
            }
        }

        // Otherwise, get Z80's write data and write it to the register
        else
        {
            // Required sequence of operations:
            uint16_t nextRegisterPointer = getNextRegisterPointer(registerPointer);

            // If next register is STATUS, then don't write it to the Z80. Wait until after the drive operation is complete.
            if (nextRegisterPointer != REG_ADDR__DRIVE_STATUS)
            {
                //   1. Get data at NEXT register pointer address
                uint8_t data = getRegisterData(nextRegisterPointer);

                //   2. Write data back to Z80 (will overwrite what's in the bus interface if there's data already there)
                writeDataToZ80(data);
            }

            //   3. Get Z80 data and write it to the current register pointer (this action will unblock the Z80)
            setRegisterData(registerPointer, getDataFromZ80());

            //   4. Move to the next register pointer
            registerPointer = nextRegisterPointer;
        }
    }

    // Check if driveOperation is set to something other than NOOP
    // (needs to be in between the write and read Z80 operations)
    if (driveOperation != _NOOP)
    {
        // Check drive action parameters

        // Perform drive action

        // Save drive operation status back into register (will be written back to the waiting Z80 in the next block)
    }

    // Request for more data from MCU
    if (isZ80ReadCompleted())
    {
        // If register select is set, then don't reload the bus interface with new data
        if (!isRegisterSelectPointer())
        {
            // Read data at register pointer and write it onto the bus interface
            writeDataToZ80(getRegisterData(registerPointer));

            // Move to the next register
            registerPointer = getNextRegisterPointer(registerPointer);
        }

        // Reset the new register pointer byte index
        newRegisterPointerByteIndex = 0;
    }
}


#if 0
        uint8_t b;

        // Serial.print("Count: "); Serial.print(count);
        // Serial.print(" Bus: "); Serial.print((uint8_t)BUS_INTERFACE_DATA_IN, HEX); Serial.print(" IN_BUFFUL "); Serial.print(digitalRead(BUS_INTERFACE_IN_BUFFUL));
        // Serial.print(" OUT_BUFFUL "); Serial.print(!digitalRead(BUS_INTERFACE_OUT_BUFFUL)); Serial.print(" REG_SEL "); Serial.print(digitalRead(BUS_INTERFACE_REG_SEL));
        // Serial.println();

        // delay(5000);

        // Serial.println("OUT_ACK 0");

        // THIS SECTION IS TIMING CRITICAL! IT MUST COMPLETE AS FAST AS POSSIBLE!
        {
            // Get byte from 82C55A
            noInterrupts();
            CLEAR_PIN(BUS_INTERFACE_OUT_ACK_port, BUS_INTERFACE_OUT_ACK_mask);
            // delayMicroseconds(10);
            // delay(5000);
            b = (uint8_t)BUS_INTERFACE_DATA_IN;
            // Serial.print("  Received data from Z80: "); Serial.println(b, HEX);
            // delay(5000);
            // Serial.println("OUT_ACK 1");
            SET_PIN(BUS_INTERFACE_OUT_ACK_port, BUS_INTERFACE_OUT_ACK_mask);
            interrupts();
        }

        if (prevByte != b)
        {
            errors++;
        }
        prevByte = (b+1) & 0xFF;

        // Save timestamp when data is 0x00
        if (b == 0x00) {
            if (count == 0)
            {
                uint32_t now = millis();
                if (timestamp != 0)
                {
                    // Print delta millis from last time
                    Serial.print("Time for 64KB: "); Serial.print(now - timestamp, DEC); Serial.print(" ms (");
                    Serial.print((uint32_t)(65536*1000)/(now-timestamp), DEC); Serial.print(" bytes/s)  Errors: "); Serial.println(errors, DEC);
                }
                timestamp = now;
            }
            count++;
        }

        // delay(2000);

        // Send a byte back to the Z80
        // b++;
        // // Serial.print("  Send data+1 back to the Z80: "); Serial.println(b, HEX);
        // BUS_INTERFACE_DATA_DDR = 0xFF;
        // BUS_INTERFACE_DATA_OUT = b;
        // digitalWrite(BUS_INTERFACE_IN_STROBE, 0);
        // digitalWrite(BUS_INTERFACE_IN_STROBE, 1);
        // BUS_INTERFACE_DATA_DDR = 0x00;

        // count++;
    }

    // delay(2000);
}

#endif