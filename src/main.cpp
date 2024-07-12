#include <Arduino.h>

// -----------------------------------------------------------------
// Current 82C55 I/O port address base: 0x10
// -----------------------------------------------------------------

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

inline void setDataBusToInput()
{
    BUS_INTERFACE_DATA_DDR = 0x00;
}

inline void writeDataBus(uint8_t data)
{
    BUS_INTERFACE_DATA_DDR = 0xFF;  // Data bus is now output
    BUS_INTERFACE_DATA_OUT = data;

    noInterrupts();
    CLEAR_PIN(BUS_INTERFACE_IN_STROBE_port, BUS_INTERFACE_IN_STROBE_mask);
    SET_PIN(BUS_INTERFACE_IN_STROBE_port, BUS_INTERFACE_IN_STROBE_mask);
    interrupts();

    BUS_INTERFACE_DATA_DDR = 0x00; // Data bus is now input
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
    // BUS_INTERFACE_OUT_BUFFUL_port = portInputRegister(digitalPinToPort(BUS_INTERFACE_OUT_BUFFUL));
    // BUS_INTERFACE_OUT_BUFFUL_mask = digitalPinToBitMask(BUS_INTERFACE_OUT_BUFFUL);
    // BUS_INTERFACE_OUT_ACK_port = (volatile uint8_t *)portOutputRegister(digitalPinToPort(BUS_INTERFACE_OUT_ACK));
    // BUS_INTERFACE_OUT_ACK_mask = digitalPinToBitMask(BUS_INTERFACE_OUT_ACK);

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

    Serial.begin(115200);

    Serial.println("retro-storage v0.1");
}

static uint8_t count = 0;
static uint32_t timestamp = 0;
uint32_t errors = 0;
uint8_t prevByte = 0;
uint8_t testData = 1;

// Z80 -> MCU WRITE sequence
// 1. Z80 writes to PORTA (PC7 (BUS_INTERFACE_OUT_BUFFUL) will go low). Z80 waits while PC7 is low.
// 2. MCU detects this and reads PORTA to get the byte using PC6 (BUS_INTERFACE_OUT_ACK) by setting it low. This sets PC7 back to high.
//    NOTE: There's a bug (feature?) in the 82C55A that occurs while PC6 is low by the MCU: any further writes by the Z80 to PORTA are immediately
//          placed on PORTA, even while the MCU is busy trying to read from PORTA. This causes data miscompare errors. To avoid this, i made a rule
//          that requires that the MCU always write the contents pointed to by the register pointer back to the 82C55A.
// 3. 

void loop() {
    // Request for more data from MCU
    if (!GET_PIN(BUS_INTERFACE_IN_BUFFUL_port, BUS_INTERFACE_IN_BUFFUL_mask))
    {
        // Write byte to 82C55A
        Serial.print("Send data to Z80 (next access): "); Serial.println(testData, HEX);

        writeDataBus(testData);
       
        testData++;
    }

    // Received data from Z80
    if (!GET_PIN(BUS_INTERFACE_OUT_BUFFUL_port, BUS_INTERFACE_OUT_BUFFUL_mask))
    {
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
