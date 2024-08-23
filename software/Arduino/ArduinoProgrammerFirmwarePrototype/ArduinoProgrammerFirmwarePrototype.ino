/*
 Written by Anders Nielsen, 2024
 https://abnielsen.com/65uino
 Some GPL license
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LONG_PRESS_TIME 1000  // Long press threshold in milliseconds

int menuIndex = 0;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;

const char* menuItems[] = {"Calibrate VEP", "Display ROM ID", "Blank check ROM", "Erase W27C512"};
const int menuItemsCount = sizeof(menuItems) / sizeof(menuItems[0]);

const float R1 = 270.0;   // Resistance R1 in kohms
const float R2 = 44.0;  // Resistance R2 in kohms
const float V_REF = 5.06; // Reference voltage in volts
const int ANALOG_PIN = A2; // Analog pin connected to VEP

#define RLSBLE  0x01
#define RMSBLE  0x02
#define ROM_OE  0x04
#define CTRL_LE 0x08
#define USRBTN  0x10
#define ROM_CE  0x20

#define VPE_TO_VPP    0b00000001
#define A9_VPP_ENABLE 0b00000010
#define VPE_ENABLE    0b00000100
#define P1_VPP_ENABLE 0b00001000
#define A17_E         0b00010000
#define A18_E         0b00100000
#define RW            0b01000000
#define REG_DISABLE   0b10000000

uint32_t romsize = 4096; //We need to support more than 16 address bits
byte pattern = 0xAA;
uint16_t cAddr = 0;
byte buffer[128]; 
#define BUFFERSIZE 128

uint32_t baudrate = 19200;

// Define a struct to hold the command, block size, and stop page
struct Command {
  uint8_t command;
  uint8_t blockSize;
  uint8_t stopPage;
};

Command currentCommand;

// the setup function runs once when you press reset or power the board
void setup() {
  // Set pins D0-D7 as outputs
  DDRD = 0xFF; // Set all D0-D7 pins as outputs
  // Set control signals as outputs
  DDRB |= RLSBLE | RMSBLE | ROM_OE | CTRL_LE | ROM_CE;

  // Set all pins using direct port manipulation
  PORTD = 0;
  
  // Set all latch pins HIGH using direct port manipulation and disable ROM 
  PORTB |= RLSBLE | RMSBLE | ROM_OE | CTRL_LE | ROM_CE | USRBTN;
delayMicroseconds(1);
  // Set all latch pins LOW using direct port manipulation
  PORTB &= ~(RLSBLE | RMSBLE | CTRL_LE); // Set pins D8, D9, D11 low

    // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    latchControlByte(0x40);
    for(;;);
  }
  
  delay(100); // Pause for 2 seconds

  // Clear the display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("Boot complete.");
  display.display();

  latchControlByte(0x10); //Can't latch things when serial is enabled
  Serial.begin(baudrate);

}

void loop() {
  if (Serial.available()) {
    byte incomingByte = Serial.read();
    if (incomingByte == 0xAA ) {
      while (!Serial.available()) { 
        ;; //Maybe we don't need it
      }
      currentCommand.command = Serial.read(); 

      if (currentCommand.command == 0x01) {
        //Dump ROM via serial
        currentCommand.blockSize = Serial.read();
        byte cAddrL = Serial.read();
        byte cAddrH = Serial.read();
        cAddr = cAddrH << 8;
        cAddr |= cAddrL;
        romsize = (static_cast<uint32_t>(Serial.read()) << 8); //Reads stoppage a.k.a. the high byte of ROM size
        if (romsize == 0) romsize = 65536; //Need a fix to support A17+
        Serial.end();
        display.clearDisplay();
        display.print(F("Sending ROM via serial..."));
        display.print("Blocksize: ");
        display.print(currentCommand.blockSize);
        display.display();
        for (int i = 0; i < romsize/currentCommand.blockSize; i++) {
        // Read data into buffer
        for (uint16_t addr = 0; addr < currentCommand.blockSize; addr++) {
          buffer[addr] = readAddress(cAddr);
          cAddr++;
        }
        Serial.begin(baudrate);
        Serial.write(0xAA);
        for (uint16_t addr = 0; addr < currentCommand.blockSize; addr++) {
          Serial.write(buffer[addr]);  
        }
        Serial.end();
        }
      }

      if (currentCommand.command == 0x03) {
        byte vendor = Serial.read();
        byte device = Serial.read();
        uint16_t romid = (static_cast<uint16_t>(vendor) << 8) | device;
        Serial.end();
        display.clearDisplay();
        display.println(F("Erasing ROM with ID: "));
        display.println(romid, HEX);
        display.display();
        eraseW27C512(romid);
        delay(3000);
      }

      if (currentCommand.command == 0x02) { 
        //Burn ROM from serial
        currentCommand.blockSize = Serial.read();
        currentCommand.stopPage = Serial.read();
        Serial.end();
        display.clearDisplay();
        display.println(F("Burning ROM from serial..."));
        display.print("Blocksize: ");
        display.print(currentCommand.blockSize);
        display.display();
        latchControlByte(VPE_TO_VPP | REG_DISABLE);
        delay(50); //Settle before enabling
        latchControlByte(VPE_TO_VPP | REG_DISABLE | VPE_ENABLE );
        delay(200);
        Serial.begin(baudrate);
        while (1) { 
          Serial.write(0xAA);
          while (!Serial.available()) { 
          ;; //Maybe we don't need it
           }
          // Read the block of data
          size_t bytesRead = Serial.readBytes((char *)buffer, currentCommand.blockSize);
            // Check if we've read the entire block
          if (bytesRead == currentCommand.blockSize) {
            // Process the buffer data
           Serial.end();
           delay(1);
           writefromBuffer(cAddr, currentCommand.blockSize);
          } else {
            display.println("Bad block");
            break;
          }
          Serial.begin(baudrate);
          delay(1);
          }
      }
    } 
  } else {
    handleButton();
    displayMenu();
  /*
  if (digitalRead(12) == 0) {
    Serial.end();
    enableRegulator();
    display.println("Press RST after calibrating VEP");
    while (1) {
          displayVEP();
          delay(17);
      }
    } else {
      display.print(".");
      display.display();
      delay(500);
    }
  */
  } //Serial 0xAA

} //Loop

void enableRegulator() {
  byte outputState = REG_DISABLE;  // Set REG_DISABLE bit
  // Set all pins using direct port manipulation
  latchControlByte(outputState);
}

void displayVEP() {
    // Read voltage from analog pin A2
  int sensorValue = analogRead(A2);
 // Convert ADC reading to voltage (assuming 5V reference voltage)
  float v_in = sensorValue * (V_REF / 1023.0);
  // Calculate voltage at VEP using voltage divider formula
  float v_vep = v_in * (R1 + R2) / R2;

    // Display voltage on OLED display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Press RST to return."));
  display.print(F("VEP Voltage: "));
  display.print(v_vep, 2); // Display voltage with 2 decimal places
  display.println(" V");
  display.display();
}

//For single byte writes 
void writeByte(uint16_t address, byte data) {
latchAddress(address);
latchControlByte(VPE_TO_VPP | REG_DISABLE);
delay(50); //Settle before enabling
latchControlByte(VPE_TO_VPP | REG_DISABLE | VPE_ENABLE );
//delay(255); //What works on 65uino
DDRD = 0xFF; //Output
PORTD = data; 
PORTB &= ~(ROM_CE);
delayMicroseconds(101);
PORTB |= ROM_CE;
latchControlByte(0x00); 
}

void writefromBuffer(uint16_t addr, uint16_t len) {
  DDRD = 0xFF; //Output
  for (int i = 0; i < len; i++){
    latchAddress(addr);
    PORTD = buffer[i];
    PORTB &= ~(ROM_CE);
    delayMicroseconds(101);
    PORTB |= ROM_CE;
    addr++;
  } 
  cAddr = addr;
}

void eraseW27C512(uint16_t romid) {
if (getROMID() == romid) {
  DDRD = 0xFF;
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

byte readAddress(uint16_t addr) {
  DDRD = 0xFF; 
  latchAddress(addr);
  DDRD = 0; //PORTD as input
  PORTB &= ~(ROM_OE | ROM_CE);
  delayMicroseconds(5); //Let it settle a bit. Maybe a NOP would do. 
  byte val = PIND;
  PORTB |= (ROM_OE | ROM_CE);
  return val;
}

void latchControlByte(byte controlByte) {
  // Set control byte using direct port manipulation
  DDRD  = 0xFF ; // All output
  PORTD = controlByte;

// Set CTRL_LE pin HIGH to latch
  PORTB |= CTRL_LE;
  // Set CTRL_LE pin LOW to unlatch
  PORTB &= ~(CTRL_LE);
}

// Global variables to store the previous LSB and MSB values
byte prevLSB = 0;
byte prevMSB = 0;

void latchAddress(uint16_t address) {
  // Extract the least significant byte
  byte lsb = address & 0xFF;
  // Extract the most significant byte
  byte msb = (address >> 8) & 0xFF;
  //Assuming PORTD is output
  // Check if LSB has changed
  if (lsb != prevLSB) {
    // Write LSB to PORTD
    PORTD = lsb;
    // Set CTRL_LE pin HIGH to latch lower 8 bits of address (LSB)
    PORTB |= RLSBLE;
    // Set RLSBLE pin LOW to unlatch lower 8 bits of address (LSB)
    PORTB &= ~RLSBLE;
    // Update prevLSB
    prevLSB = lsb;
  }

  // Check if MSB has changed
  if (msb != prevMSB) {
    // Write MSB to PORTD
    PORTD = msb;
    // Set CTRL_LE pin HIGH to latch higher 8 bits of address (MSB)
    PORTB |= RMSBLE;
    // Set RMSBLE pin LOW to unlatch higher 8 bits of address (MSB)
    PORTB &= ~RMSBLE;
    // Update prevMSB
    prevMSB = msb;
  }
}

void handleButton() {
  if(digitalRead(12) == LOW) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonPressTime = millis();
    } else {
      if (millis() - buttonPressTime > LONG_PRESS_TIME) {
        // Long press detected
        buttonPressed = false;  // Reset button state
        handleSelection(menuIndex);
        Serial.begin(baudrate);
        delay(500);  // Prevent multiple triggers
      }
    }
  } else {
    if (buttonPressed) {
      if (millis() - buttonPressTime < LONG_PRESS_TIME) {
        // Short press detected
        menuIndex++;
        if (menuIndex >= menuItemsCount) menuIndex = 0;
      }
      buttonPressed = false;  // Reset button state
    }
  }
}

void displayMenu() {
  display.clearDisplay();
  for(int i = 0; i < menuItemsCount; i++) {
    if(i == menuIndex) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Highlight selected item
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(0, i*10);
    display.println(menuItems[i]);
  }
  display.display();
}


void handleSelection(int index) {
  //Calibrate VEP", "Display ROM ID", "Blank check ROM", "Erase W27C512"
  switch (index) {
    case 0:
          Serial.end();
          enableRegulator();
          while (1) {
                display.println(F("Press RST after calibrating VEP"));
                displayVEP();
                delay(17);
            }
      break;
    case 1:
          Serial.end();
      display.clearDisplay();
      display.setCursor(0,0);
      display.print(F("Read ROM ID: "));
      display.println(getROMID(),HEX);
      display.display();
           delay(3000);
      break;
    case 2:
     Serial.end();
     display.clearDisplay();
     display.setCursor(0,0);
     if (blankCheck() == 0) display.println("ROM is blank");
     display.display();
     delay(3000);
      break;
    case 3:
     Serial.end();
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

uint16_t blankCheck () {
  display.clearDisplay();
  display.println(F("Checking if ROM is blank.."));
  display.display();
  uint32_t i;
  for (i = 0; i < 65537; i++) {
    byte data = readAddress(i);
    if (data != 0xFF) {
      display.print(data,HEX);
      display.print(" found at address ");
      display.println(i, HEX);
      display.display();
      return 1;
    } 
  }
  if (i == 65537) i = 0;
  return i;
}
