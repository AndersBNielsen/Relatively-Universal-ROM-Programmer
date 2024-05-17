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

byte pattern = 0xAA;
uint16_t cAddr = 0;

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
  //  Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(100); // Pause for 2 seconds

  // Clear the display
  display.clearDisplay();
  display.display();

}

void loop() {

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

  latchControlByte(RW); //Assuming we might want to read a 32pin ROM with RW on pin 31. 

  byte byteRead = readAddress(cAddr);
  cAddr++;

  display.println(byteRead,HEX); //Debugging
  display.display();

  delay(100); //To hopefully make the serial receiver forget the junk it just got, before we send real data. 
  Serial.begin(9600);
  Serial.print(byteRead,HEX);
  Serial.end();

  // Introduce a delay before repeating the process
  delay(100); // Adjust delay time as needed

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

byte readAddress(uint16_t addr) {
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
  // Delay to ensure proper latching
  delayMicroseconds(1);

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

  // Set PORTD as output
  DDRD = 0xFF; // Set PORTD as output

  // Check if LSB has changed
  if (lsb != prevLSB) {
    // Set CTRL_LE pin HIGH to latch lower 8 bits of address (LSB)
    PORTB |= RLSBLE;

    // Write LSB to PORTD
    PORTD = lsb;

    // Set RLSBLE pin LOW to unlatch lower 8 bits of address (LSB)
    PORTB &= ~RLSBLE;

    // Update prevLSB
    prevLSB = lsb;
  }

  // Check if MSB has changed
  if (msb != prevMSB) {
    // Set CTRL_LE pin HIGH to latch higher 8 bits of address (MSB)
    PORTB |= RMSBLE;

    // Write MSB to PORTD
    PORTD = msb;

    // Set RMSBLE pin LOW to unlatch higher 8 bits of address (MSB)
    PORTB &= ~RMSBLE;

    // Update prevMSB
    prevMSB = msb;
  }
}

