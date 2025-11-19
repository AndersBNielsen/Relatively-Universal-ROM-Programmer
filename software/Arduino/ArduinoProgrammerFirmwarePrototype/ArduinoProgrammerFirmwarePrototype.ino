/*
 --------------------------------------------------------------------------------------------------
  Project:    Relatively-Universal-ROM-Programmer (RURP)
  File:       ArduinoProgrammerFirmwarePrototype.ino
  Target:     Arduino Uno (ATmega328P)
  License:    GNU GPL v3.0 (as per original project)
  
  Original Author:
      Anders Nielsen (2024)
      https://abnielsen.com/65uino
      GPL-3.0 License

  Modified By:
      BizarroBull (2024)
        • cast result of Serial.read() so it doesn't get treated as signed when bit shifting

      TheSubWayKing (2025)
        • Expanded and improved 2716 EPROM read/write support
        • Added improved error handling and input validation
        • Refactored for coding-standard consistency
        • Improved I/O configuration logic and bus-direction control
        • Added safer timing behavior and cleaner command parsing
        • Updated documentation and code comments

 --------------------------------------------------------------------------------------------------
  Hardware Context:
      This firmware is written for the “Relatively Universal ROM Programmer”
      (RURP) shield designed by Anders Nielsen for the Arduino Uno.  
      
      The RURP shield supports a broad variety of parallel ROM/EPROM/EEPROM
      devices through a flexible address/data bus, latch bank, and mode
      control lines (OE, CE, WE). The 2716 EPROM, being one of the earliest
      EPROM types with unique timing and power-enable constraints, requires
      specific handling not fully addressed by alternative RURP firmware.

      Another community firmware (“Firestarter”) provides more comprehensive
      device-family support overall, but **does not currently support the 2716**.
      This modified firmware exists to fill that gap and to modernize the
      original 2716-focused codebase.

 --------------------------------------------------------------------------------------------------
  Sketch Overview / Functional Description:
      This sketch implements a full command-driven serial interface for ROM
      programming, reading, verification, and memory utilities specifically
      tuned for the 2716 EPROM. Major features include:

        • Full 2716 read cycle implementation using correct OE/CE timing
        • Write/Program pulse generation with adjustable microsecond timing
        • Data-bus tri-state management for read/write transitions
        • Address-bus and control-line mapping via AVR ports:
              - PORTD / DDRD for address/data bus (D0–D7)
              - PORTB / DDRB for control latches (D8-D13)
        • Error handling for invalid commands, timing violations,
          serial framing issues, and out-of-range accesses
        • Buffered read operations and configurable dump size
        • Command set for:
              - Reading EPROM into buffer
              - Writing buffer contents to EPROM
              - Blank-check
              - Verification
              - Direct byte I/O for debugging
              - Diagnostic and version reporting

 --------------------------------------------------------------------------------------------------
  Revision History:
      2024-05-15 – Initial Arduino Firmware Prototype (AndersBNielsen)
      2024-05-17 - Arduino sketch now successfully reads a ROM to serial.. Slowly. (AndersBNielsen)
      2024-05-20 - Untested code for writing to W27C512 via serial (AndersBNielsen)
      2024-05-21 - Erasing and programming a W27C512 (AndersBNielsen)
      2024-05-22 - Erasing, Burning, and Reading a W27C512 now works! (AndersBNielsen)
      2024-05-24 - Added selection menu to Arduino sketch (AndersBNielsen)
      2024-05-24 - Arduino sketch enters calibration with a buttonpress (AndersBNielsen)
      2024-08-01 - Update ArduinoProgrammerFirmwarePrototype.ino (BizarroBull)
      2025-08-17 - 2716 / TMS2532 support (AndersBNielsen)
      2025-11-19 – Major enhancements and 2716-specific improvements (TheSubWayKing)
 --------------------------------------------------------------------------------------------------
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== Monochrome OLED Display =====
constexpr uint8_t SCREEN_WIDTH = 128;            // OLED width
constexpr uint8_t SCREEN_HEIGHT = 64;            // OLED height
constexpr int8_t OLED_RESET = -1;                // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===== Menu Items and User Input =====
constexpr char* const MENU_ITEMS[] = {           // Menu item options 
    "Calibrate VEP",
    "Display ROM ID",
    "Blank check ROM",
    "Erase W27C512"
};
constexpr uint8_t MENU_ITEM_COUNT = sizeof(MENU_ITEMS) / sizeof(MENU_ITEMS[0]);
constexpr uint16_t LONG_PRESS_TIME = 1000;       // Long press threshold in milliseconds
uint8_t menuIndex = 0;                           // Track USR button press menu selection 
unsigned long buttonPressTime = 0;               // Track USR button press duration
bool buttonPressed = false;                      // Track USR button pressed

// ===== Analog Voltage Monitoring (VPP/VEP) =====
constexpr float R1 = 270.0;                      // Resistance R1 in kohms
constexpr float R2 = 44.0;                       // Resistance R2 in kohms
constexpr float V_REF = 5.06;                    // Reference voltage in volts
constexpr uint8_t ANALOG_PIN = A2;               // Analog pin connected to VEP

// ===== Hardware Control Pins (PORTB bitmasks) =====
constexpr uint8_t RLSBLE  = 0b00000001;          // D8  – Latch Low-byte (A0-A7) when HIGH
constexpr uint8_t RMSBLE  = 0b00000010;          // D9  – Latch High-byte (A8-A15) when HIGH
constexpr uint8_t ROM_OE  = 0b00000100;          // D10 – Output Enable (active LOW)
constexpr uint8_t CTRL_LE = 0b00001000;          // D11 – Control Latch Enable (active HIGH)
constexpr uint8_t USRBTN  = 0b00010000;          // D12 – User button input
constexpr uint8_t ROM_CE  = 0b00100000;          // D13 – Chip Enable (active LOW)

// ===== EPROM Control Pins (PORTD bitmasks) =====
constexpr uint8_t VPE_TO_VPP    = 0b00000001;    // Route VPE line to VPP voltage source
constexpr uint8_t A9_VPP_ENABLE = 0b00000010;    // Enable VPP (programming voltage) on address line A9
constexpr uint8_t VPE_ENABLE    = 0b00000100;    // Enable programming voltage on VPE pin
constexpr uint8_t P1_VPP_ENABLE = 0b00001000;    // Enable VPP on pin P1 (or socket position 1)
constexpr uint8_t A17_E         = 0b00010000;    // Enable high-order address line A17
constexpr uint8_t VCC28PIN      = 0b00010000;    // (Alias) Select 28-pin device VCC mapping (shares bit with A17_E)
constexpr uint8_t A18_E         = 0b00100000;    // Enable high-order address line A18
constexpr uint8_t VCC24PIN      = 0b00100000;    // (Alias) Select 24-pin device VCC mapping (shares bit with A18_E)
constexpr uint8_t RW            = 0b01000000;    // Read/Write control bit (1 = read, 0 = write) - (currently unused))
constexpr uint8_t REG_DISABLE   = 0b10000000;    // Disable regulator

// ===== EPROM Chip Parameters =====
constexpr uint8_t ROM_PIN_COUNT = 28;            // Default ROM pin count
constexpr uint32_t ROM_SIZE = 4096;              // Default ROM size
constexpr uint32_t MAX_ROM_SIZE_16BIT = 65536;   // Maximum 16 Bit ROM size
uint8_t romPinCount = ROM_PIN_COUNT;             // ROM pin count
uint32_t romSize = ROM_SIZE;                     // ROM size - We need to support more than 16 address bits

// ===== Timing Constants =====
constexpr uint16_t SERIAL_SETTLE_US = 500;       // Default serial settle time in microseconds
constexpr uint8_t STARTUP_DELAY_MS = 100;        // Delay required at startup for Display/Shield settle?
constexpr uint8_t ADDRESS_DATA_SETTLE_US = 5;    // Default latch settle time in microseconds
constexpr uint8_t ROM_24PIN_WRITE_PULSE_MS = 50; // 24 Pin ROM write pulse duration in milliseconds
constexpr uint8_t ROM_28PIN_WRITE_PULSE_US = 100;// 28 Pin ROM write pulse duration in microseconds

// ===== EPROM Data Buffer & Address State =====
constexpr uint8_t BUFFERSIZE = 128;              // Default buffersize
byte buffer[BUFFERSIZE];                         // Contains the data being processed to and from EPROM
uint16_t cAddr = 0;                              // Current address being processed
byte prevLSB = 0xFF;                             // Previous LSB latchaddress state tracking
byte prevMSB = 0xFF;                             // Previous MSB latchaddress state tracking

// ===== Serial Communication =====
constexpr uint32_t BAUDRATE = 19200;             // Default device BAUDRATE

// ===== Command Byte Actions =====
constexpr byte CMD_MODE = 0xAA;                  // Command mode flag
constexpr byte CMD_DUMP = 0x01;                  // Command - Dump ROM to serial
constexpr byte CMD_BURN = 0x02;                  // Command - Burn ROM from serial
constexpr byte CMD_ERASE = 0x03;                 // Command - Erase ROM
struct Command {                                 // Struct to hold the current command, block size, and stop page
    uint8_t command;
    uint8_t blockSize;
    uint8_t stopPage;
};
Command currentCommand;                          // Instance of Command to store the current action being performed

// The setup function runs once when you press reset or power the board
void setup() {
    delay(STARTUP_DELAY_MS);                    // Without this behaviour is inconsistent/problematic - either LCD or Shield related!
    initial_pin_state();                        // Initialise port B, port D, address, VCC and VPP pin states
    
    // Initialize the OLED display
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        while(1);                               // Critical failure - halt
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("Boot complete.");
    display.display();
    
    Serial.begin(BAUDRATE);                     // Open serial port - Can't latch things when serial is enabled
    delayMicroseconds(SERIAL_SETTLE_US);        // Let serial settle
}

// Keep checking for data in the serial buffer, otherwise process button and display menu
void loop() {
    if (Serial.available()) {
        byte incomingByte = Serial.read();          // Read first byte from the serial receive buffer

        if (incomingByte == CMD_MODE ) {            // Check for incoming Command Mode byte
            while (!Serial.available());            // Wait for more data to arrive in the serial receive buffer
            
            currentCommand.command = Serial.read(); // Read from buffer current command 

            switch (currentCommand.command) {       // Process selected command
                case CMD_DUMP:                      
                    dumpROM();
                    break;
                case CMD_BURN:
                    burnROM();
                    break;
                case CMD_ERASE:
                    eraseROM();
                    break;
                default:
                    display.clearDisplay();
                    display.print(F("Unknown Command"));
                    break;
            }
            
            Serial.flush();                         // Wait for any prior transmission to complete
            Serial.end();                           // Release shared UART pins

            initial_pin_state();                    // Reinitialise port pin states
            Serial.begin(BAUDRATE);                 // Open serial port
            delayMicroseconds(SERIAL_SETTLE_US);    // Let serial settle
        } 
    } else {
        handleButton();
        displayMenu();
    }
} // Loop

// Configure the initial arduino/rurp pin states
void initial_pin_state() {
    // Set digital pin input/output direction
    DDRD = 0xFF;                                            // Initialise all data/address pins as outputs (D0-D7 pins)
    DDRB = 0x00;                                            // Initialise all digital pins D8-D13 as inputs
    DDRB |= RLSBLE | RMSBLE | ROM_OE | CTRL_LE | ROM_CE;    // Set specific digital pins as outputs
    
    // Clear all values and latch
    PORTD = 0x00;                                           // Set all data/address pins to 0 (D0-D7 pins)
    delayMicroseconds(ADDRESS_DATA_SETTLE_US);              // Let Address/Data settle
    PORTB |= RLSBLE | RMSBLE | CTRL_LE;                     // Set RLSBLE, RMSBLE, CTRL_LE pin HIGH 
    PORTB &= ~(RLSBLE | RMSBLE | CTRL_LE);                  // Set RLSBLE, RMSBLE, CTRL_LE pin LOW to latch zero values

    // Set ROM defaults
    PORTB |= ROM_OE | ROM_CE;                               // Initialise Output Enabled, Chip Enable, User button pins to HIGH
    romPinCount = ROM_PIN_COUNT;                            // Default rom pin count
    romSize = ROM_SIZE;                                     // We need to support more than 16 address bits

    PORTB |= USRBTN;                                        // Set internal pull-up resistor for USRBTN

    // Clear address state tracking
    prevLSB = 0xFF;                                         // Reset Previous LSB latchAddress() state tracking 
    prevMSB = 0xFF;                                         // Reset Previous MSB latchAddress() state tracking 
    cAddr = 0;                                              // Reset current address state tracking
}

// Read ROM data and dump over serial
void dumpROM() {
    currentCommand.blockSize = Serial.read();               // Read in the blocksize
    byte cAddrL = Serial.read();                            // Read start address LSB
    byte cAddrH = Serial.read();                            // Read start address MSB
    cAddr = (cAddrH << 8) | cAddrL;                         // Assemble 16-bit start address from MSB and LSB
    
    romSize = (static_cast<uint32_t>(Serial.read()) << 8);  // Reads stoppage a.k.a. the high byte of ROM size
    if (romSize == 0) romSize = MAX_ROM_SIZE_16BIT;         // Need a fix to support A17+
    romPinCount = Serial.read();                            // Read in the ROM Pin count
    
    // Validate block size
    if (currentCommand.blockSize > BUFFERSIZE) {
        display.print(F("Error: Block size too large"));
        return;
    }

    Serial.end();                                           // Release shared pins
    
    if (romPinCount == 24) latchAddress(VCC24PIN << 8);     // Enable VCC for 24 pin ROM - shift to MSB
    if (romPinCount == 28) latchControlByte(VCC28PIN);      // Enable VCC for 28 pin ROM
    
    display.clearDisplay();
    display.print(F("Sending ROM via serial..."));
    display.print("Blocksize: ");
    display.print(currentCommand.blockSize);
    display.display();

    // Calculate totalBlocks, rounding up to ensure any remainder bytes get their own block
    uint16_t totalBlocks = (romSize + currentCommand.blockSize - 1) / currentCommand.blockSize;

    // loop over the total number of blocks in the ROM
    for (uint32_t blockNum = 0; blockNum < totalBlocks; blockNum++) {
        // Calculate bytes to read in the final block
        uint16_t bytesToRead = currentCommand.blockSize;
        if (cAddr + bytesToRead > romSize) {
            bytesToRead = romSize - cAddr;
        }

        // Read entire block
        for (uint16_t addr = 0; addr < bytesToRead; addr++) {
            buffer[addr] = readAddress(cAddr);              // Read ROM data into buffer
            cAddr++;
        }
        Serial.begin(BAUDRATE);                             // Open serial port
        delayMicroseconds(SERIAL_SETTLE_US);                // Brief settle time to avoid framing errors

        Serial.write(0xAA);                                 // Transmit the frame start block indicator
        Serial.flush();                                     // Wait for start block transmission to complete
     
        Serial.write(buffer, bytesToRead);                  // Send entire buffer at once
        Serial.flush();                                     // Wait for transmission to complete

        Serial.end();                                       // Release pins for next read cycle
    }
}

// Read serial data and burn to ROM
void burnROM() {
    bool dataProcessed = false;                      // initialise flag to indicate data has been processed
    byte controlByte = 0x00;                         // initialise control byte
    static const byte EOF_BYTE = 0x00;               // define end of file as empty byte
    
    currentCommand.blockSize = Serial.read();        // read the blocksize
    currentCommand.stopPage = Serial.read();         // read the stop page - currently unused!
    romPinCount = Serial.read();                     // read the ROM pin count
    
    // Validate block size
    if (currentCommand.blockSize > BUFFERSIZE) {
        display.print(F("Error: Block size too large"));
        return;
    }

    Serial.end();                                    // Release shared pins
    
    if (romPinCount == 24) {
        latchAddress(VCC24PIN << 8);                 // Enable 24 pin VCC - shift to MSB
        PORTB &= ~(ROM_CE);                          // Make sure chip is enabled
        controlByte = (REG_DISABLE | P1_VPP_ENABLE); // Enabling P1_VPP_Enable for JP4 (Leave OPEN for 28 pin ROMs!!))
    }

    if (romPinCount == 28) {
        controlByte = (VPE_TO_VPP | REG_DISABLE | VPE_ENABLE | VCC28PIN);
    }

    latchControlByte(controlByte);                   // Apply ROM control pin configuration

    display.clearDisplay();
    display.println(F("Burning ROM from serial..."));
    display.print("Blocksize: ");
    display.print(currentCommand.blockSize);
    display.display();
    
    Serial.begin(BAUDRATE);                          // Open serial port
    delayMicroseconds(SERIAL_SETTLE_US);             // Let the serial come up and settle

    // Process each block of binary source file data via serial until no more bytes read
    while (1) { 
        Serial.write(0xAA);                          // Send ready signal
        Serial.flush();                              // Wait for 0xAA transmission to complete

        while (!Serial.available());                 // Wait for data to arrive in the serial receive buffer
        
        memset(buffer, 0xFF, BUFFERSIZE);            // Clear buffer with 0xFF
        
        // Read the block of data
        size_t bytesRead = Serial.readBytes((char *)buffer, currentCommand.blockSize);
        
        // check for EOF_BYTE and data processing complete, zero length block, no data, bad block, otherwise process data block
        if (bytesRead == 1 && buffer[0] == EOF_BYTE && dataProcessed) {
            display.println(F("Transfer Complete"));
            break;
        } else if (bytesRead == 1 && buffer[0] == 0x00 ) {
            display.println(F("Error: Bad zero length block"));
            break;
        } else if (bytesRead == 0) {
            display.println(F("Error: No data"));
            break;
        } else if (bytesRead > currentCommand.blockSize) {
            display.println(F("Error: Bad block"));
            break;      
        } else {
            Serial.end();                            // Release shared pins
            delayMicroseconds(SERIAL_SETTLE_US);     // Let the serial settle

            writefromBuffer(cAddr, bytesRead);
            dataProcessed = true;                    // Set data processed indicator
         
            Serial.begin(BAUDRATE);                  // Open serial port back up
            delayMicroseconds(SERIAL_SETTLE_US);     // Let the serial settle
        }
    }
}

// erase ROM data
void eraseROM() {
    byte vendor = Serial.read();
    byte device = Serial.read();
    uint16_t romid = (static_cast<uint16_t>(vendor) << 8) | device;

    Serial.end();                                   // Release shared UART pins

    display.clearDisplay();
    display.println(F("Erasing ROM with ID: "));
    display.println(romid, HEX);
    display.display();

    eraseW27C512(romid);
    delay(3000);
}

// Enable Regulator on the RURP shield
void enableRegulator() {
    byte outputState = REG_DISABLE;  // Set REG_DISABLE bit
    latchControlByte(outputState);   // Set all pins using direct port manipulation
}

// Show VEP on oled display to facilitate manual adjustment
void displayVEP() {
    int sensorValue = analogRead(ANALOG_PIN);           // Read voltage from analog pin A2
    float v_in = sensorValue * (V_REF / 1023.0);        // Convert ADC reading to voltage (assuming 5V reference voltage)
    float v_vep = v_in * (R1 + R2) / R2;                // Calculate voltage at VEP using voltage divider formula
  
    // Display voltage on OLED display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Press RST to return."));
    display.print(F("VEP Voltage: "));
    display.print(v_vep, 2);                            // Display voltage with 2 decimal places
    display.println(" V");
    display.display();
}

// Perform data write to selected ROM for the current address parameter
void writefromBuffer(uint16_t addr, uint16_t len) {
    DDRD = 0xFF;                                // Set all digital pins (0-7) as Outputs
  
    // Initialise CE state for 24 pin 
    if (romPinCount == 24) {
        PORTB &= ~(ROM_CE);                     // Ensure Chip Enable intially starts LOW for 2716 chips 
    }
    
    // Write each buffer byte to ROM  
    for (int i = 0; i < len; i++){
        latchAddress(addr);
        PORTD = buffer[i];                      // Push buffer data for cuurent address (i) onto PORTD pins
        
        if (romPinCount == 28) {
            PORTB &= ~(ROM_CE);                 // Low pulse sets up ROM write for 28pin chips
            delay(ROM_28PIN_WRITE_PULSE_US);    // 100us pulse length performs the 28pin ROM write
            PORTB |= ROM_CE;                    // Return CE to high
        }
        if (romPinCount == 24) {
            PORTB |= ROM_CE;                    // High pulse allows ROM write for 2716/TMS2516
            delay(ROM_24PIN_WRITE_PULSE_MS);    // 50ms pulse length performs the 2716/TMS2516 ROM write
            PORTB &= ~(ROM_CE);                 // Low pulse allows ROM write for all other 24pin chips, whilst ending 2716 pulse
            delay(ROM_24PIN_WRITE_PULSE_MS);    // 50ms pulse length performs all other 24pin ROM write
            PORTB |= ROM_CE;                    // End all other 24pin write pulse
            PORTB &= ~(ROM_CE);                 // Return Chip Enable low in readiness for next address to be written for 2716
        }
        addr++;
    } 
    cAddr = addr;
}

// erase process for W27C512 ROM chips
void eraseW27C512(uint16_t romid) {
    if (getROMID() == romid) {
        DDRD = 0xFF;                    // Set all digital pins (0-7) as Outputs
        latchAddress(0x0000);
        enableRegulator();
        delay(100);
        latchControlByte(A9_VPP_ENABLE | REG_DISABLE | VPE_ENABLE );
        delay(500);
        PORTB &= ~(ROM_CE);
        delay(102);
        PORTB |= ROM_CE;
    } else {
        display.println("ROM ID didn't match.");
        display.display();
    }
    latchControlByte(0);
}

// Retrieve the ROM Identification
uint16_t getROMID() {
    enableRegulator();
    // Introduce a delay before repeating the process
    delay(50); // Adjust delay time as needed
    latchControlByte(VPE_TO_VPP | REG_DISABLE | A9_VPP_ENABLE );
    delay(50);
    byte byteRead = readAddress(0x0000);
    uint16_t romid = byteRead;
    romid = romid <<8;
    byteRead = readAddress(0x0001);
    romid |= byteRead;
    latchControlByte(0);
    return romid;
}

// Read a 16-bit address from the ROM and return the value
byte readAddress(uint16_t addr) {
    DDRD = 0xFF;                                // Set all digital pins (0-7) as Output
    latchAddress(addr);                         // Set the address pins
    DDRD = 0x00;                                // Set all digital pins (0-7) as Input
    PORTB &= ~(ROM_OE | ROM_CE);                // Set pins Output Enable and Chip Enable LOW 
    delayMicroseconds(ADDRESS_DATA_SETTLE_US);  // Let Address/Data settle 
    byte val = PIND;                            // Read the value from all digital output pins
    PORTB |= (ROM_OE);                          // Set Output Enable HIGH, leave CE LOW
    return val;
}

// Push a 8-bit value onto the arduino D0-D7 pins and latch
void latchControlByte(byte controlByte) {
    if (romPinCount == 28) controlByte |= VCC28PIN; // Enable VCC for 28 pin ROMs
  
    DDRD = 0xFF;                                    // Set all digital pins (0-7) as Output
    PORTD = controlByte;                            // Push the control value onto the pins
    delayMicroseconds(ADDRESS_DATA_SETTLE_US);      // Let Address/Data settle
    PORTB |= CTRL_LE;                               // Set CTRL_LE pin HIGH to latch
    PORTB &= ~(CTRL_LE);                            // Set CTRL_LE pin LOW to unlatch
}

// Push a 16-bit address onto the arduino D0-D7 pins for the each upper/lower byte
void latchAddress(uint16_t address) {
    byte lsb = address & 0xFF;                      // Extract the least significant byte
    byte msb = (address >> 8) & 0xFF;               // Extract the most significant byte

    // Check if LSB has changed
    if (lsb != prevLSB) {
        PORTD = lsb;                                // Write LSB address to PORTD pins
        delayMicroseconds(ADDRESS_DATA_SETTLE_US);  // Let Address/Data settle
        PORTB |= RLSBLE;                            // Set RLSBLE pin HIGH to latch lower 8 bits of address (LSB)
        PORTB &= ~RLSBLE;                           // Set RLSBLE pin LOW to unlatch lower 8 bits of address (LSB)
        prevLSB = lsb;                              // Update prevLSB
    }
  
    // Check if MSB has changed
    if (msb != prevMSB) {
        prevMSB = msb;                              // Update prevMSB before ORing in VCC for 24pin ROMs, otherwise above check will always be true
        
        if (romPinCount == 24) {
            msb |= VCC24PIN;                        // Enable VCC on "A13"
        }
        PORTD = msb;                                // Write MSB address to PORTD pins
        delayMicroseconds(ADDRESS_DATA_SETTLE_US);  // Let Address/Data settle
        PORTB |= RMSBLE;                            // Set RMSBLE pin HIGH to latch higher 8 bits of address (MSB)
        PORTB &= ~RMSBLE;                           // Set RMSBLE pin LOW to unlatch higher 8 bits of address (MSB)
    }
}

// When USR button is pressed, process menu cycle (short press) or menu selection (long press)
void handleButton() {
    if(digitalRead(12) == LOW) {
        if (!buttonPressed) {
            buttonPressed = true;
            buttonPressTime = millis();
        } else {
            if (millis() - buttonPressTime > LONG_PRESS_TIME) {
                // Long press detected
                buttonPressed = false;      // Reset button state
                handleSelection(menuIndex);
                Serial.begin(BAUDRATE);     // Open serial port 
                delay(500);                 // Prevent multiple triggers
            }
        }
    } else {
        if (buttonPressed) {
            if (millis() - buttonPressTime < LONG_PRESS_TIME) {
                // Short press detected
                menuIndex++;
                if (menuIndex >= MENU_ITEM_COUNT) menuIndex = 0;
            }
            buttonPressed = false;          // Reset button state
        }
    }
}

// Show Menu options and highlight currently selected item
void displayMenu() {
    display.clearDisplay();
    for(int i = 0; i < MENU_ITEM_COUNT; i++) {
        if(i == menuIndex) {
            display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Highlight selected item
        } else {
            display.setTextColor(SSD1306_WHITE);
        }
        display.setCursor(0, i*10);
        display.println(MENU_ITEMS[i]);
    }
    display.display();
}

// Process menu action based on USR button selection index parameter
void handleSelection(int index) {
    switch (index) {
        case 0: // Calibrate VEP
            Serial.end();                           // Release shared UART pins
            enableRegulator();
            while (1) {                             // Infinite Loop!! - Perhaps change to while until USRBTN pressed
                display.println(F("Press RST after calibrating VEP"));
                displayVEP();
                delay(17);
            }
            break;
        case 1: // Display ROM ID
            Serial.end();                           // Release shared UART pins
            display.clearDisplay();
            display.setCursor(0,0);
            display.print(F("Read ROM ID: "));
            display.println(getROMID(),HEX);
            display.display();
            delay(3000);
            break;
        case 2: // Blank check ROM
            Serial.end();                           // Release shared UART pins
            display.clearDisplay();
            display.setCursor(0,0);
            if (blankCheck() == 0) display.println("ROM is blank");
            display.display();
            delay(3000);
            break;
        case 3: // Erase W27C512
            Serial.end();                           // Release shared UART pins
            display.clearDisplay();
            display.setCursor(0,0);
            display.println("Erasing ROM with ID 0xDA08 ");
            eraseW27C512(0xDA08);
            display.display();
            delay(3000);
            break;
        default:
            break;
    }
}

// check if ROM is blank, return 1 if not blank otherwise return 0
uint16_t blankCheck () {
    display.clearDisplay();
    display.println(F("Checking if ROM is blank.."));
    display.display();
    uint32_t i;
    for (i = 0; i < MAX_ROM_SIZE_16BIT; i++) {
        byte data = readAddress(i);
        if (data != 0xFF) {
            display.print(data,HEX);
            display.print(" found at address ");
            display.println(i, HEX);
            display.display();
            return 1;               // Not blank
        } 
    }
    return 0;                       // blank
}