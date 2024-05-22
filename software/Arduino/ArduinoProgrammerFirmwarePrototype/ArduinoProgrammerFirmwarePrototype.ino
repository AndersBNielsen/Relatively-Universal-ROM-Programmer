/*
 Written by Anders Nielsen, 2024
 https://abnielsen.com/65uino
 Some GPL license
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define BAUDRATE 57600

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
  PORTB |= RLSBLE | RMSBLE | ROM_OE | CTRL_LE | ROM_CE;
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
  display.println("Boot complete..");
  display.display();

  latchControlByte(0x10); //Can't latch things when serial is enabled
  Serial.begin(BAUDRATE);

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
        display.print(currentCommand.blockSize);
        byte cAddrL = Serial.read();
        byte cAddrH = Serial.read();
        cAddr = cAddrH << 8;
        cAddr |= cAddrL;
        /*display.print(" ");
        display.print(cAddr);
        display.print(" ");*/
        romsize = (Serial.read() << 8); //Reads stoppage a.k.a. the high byte of ROM size
        if (romsize == 0) romsize = 65536; //Need a fix to support A17+
        //display.print(romsize);
        //display.display();
        Serial.end();

        for (int i = 0; i < romsize/currentCommand.blockSize; i++) {
        // Read data into buffer
        for (uint16_t addr = 0; addr < currentCommand.blockSize; addr++) {
          buffer[addr] = readAddress(cAddr);
          cAddr++;
        }

        Serial.begin(BAUDRATE);
        Serial.write(0xAA);
        for (uint16_t addr = 0; addr < currentCommand.blockSize; addr++) {
          Serial.write(buffer[addr]);  
        }
        Serial.end();
        }

        }

      if (currentCommand.command == 0x03) {
        Serial.end();
        display.println("Erasing ROM with ID: ");
        eraseW27C512();
      }

      if (currentCommand.command == 0x02) { 
        //Burn ROM from serial
        currentCommand.blockSize = Serial.read();
        currentCommand.stopPage = Serial.read();
        Serial.end();
        latchControlByte(VPE_TO_VPP | REG_DISABLE);
        delay(50); //Settle before enabling
        latchControlByte(VPE_TO_VPP | REG_DISABLE | VPE_ENABLE );
        delay(200);
        Serial.begin(BAUDRATE);
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
          Serial.begin(BAUDRATE);
          delay(1);
          }
      }
    } 

  } else {
  //display.setCursor(0, 0);
  display.print(".");
  display.display();
  delay(500);
  }

  //latchControlByte(RW); //Assuming we might want to read a 32pin ROM with RW on pin 31. 

  /*byte byteRead = readAddress(cAddr);
  cAddr++;
  delay(100); //To hopefully make the serial receiver forget the junk it just got, before we send real data. 
  Serial.begin(BAUDRATE);
  Serial.print(byteRead,HEX);
  Serial.end();
*/

/*
for (int q = 0; q < 4; q++) {
writeByte(cAddr+q, 0xAA);
}
delay(1000);

  byte gotbyte = readAddress(cAddr);
display.print(gotbyte,HEX); //Debugging
display.display();
delay(1000);
cAddr+=4;
*/

//eraseW27C512();
/*
for (int i = 0; i < 4; i++) {
  writeByte(i, 0xAA);
}

for (int q = 0; q < 5; q++) {
byte gotbyte = readAddress(q);
display.print(gotbyte,HEX); //Debugging
display.display();
}

while (1) {
  ;;
}
*/
/*
for (int i = 0; i < ROMSIZE/512; i++) {
  // Read data into buffer
  for (uint16_t addr = 0; addr < 512; addr++) {
    buffer[addr] = readAddress(cAddr);
    cAddr++;
  }

  delay(1);
  Serial.begin(19200);
  for (uint16_t addr = 0; addr < 512; addr++) {
    Serial.print(buffer[addr],HEX);  
  }
  Serial.end();
}
*/

/*
   for (uint32_t address = 0; address <= 65535; ++address) {
    latchAddress(address);
    // Delay between each address (adjust as needed)
    delayMicroseconds(30); // 10 milliseconds delay, for example
  }
*/
  //pattern = (pattern >> 1) | (pattern << 7) ;
}

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
  display.print("Voltage at VEP:");
  display.setCursor(0, 10);
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

void eraseW27C512() {
if (getROMID() == 0xDA08) {
  DDRD = 0xFF;
  latchAddress(0x0000);
  enableRegulator();
  delay(100);
  latchControlByte(A9_VPP_ENABLE | REG_DISABLE | VPE_ENABLE );
  delay(500);
  PORTB &= ~(ROM_CE);
  delay(102);
  PORTB |= ROM_CE;
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
  display.println(romid,HEX); //Debugging
  latchControlByte(0);
  display.display();
return romid;
}

byte readAddress(uint16_t addr) {
  DDRD = 0xFF; 
  latchAddress(addr);
  DDRD = 0; //PORTD as input
  PORTB &= ~(ROM_OE | ROM_CE);
  delayMicroseconds(5); //Let it settle a bit. Maybe a NOP would do. 
  byte val = PIND;
  PORTB = (ROM_OE | ROM_CE);
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

