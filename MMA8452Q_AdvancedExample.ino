/* MMA8452Q Example Code
 by: Jim Lindblom
 SparkFun Electronics
 date: November 17, 2011
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Modified by: Kris Winer, February 20, 2014 to include reset,  accelerometer trimming, sleep mode functionality as well as
 parameterizing the register addresses. Added LCD functions to allow display to on breadboard monitor and will add motion detection
 as well as calculation of instantaneous velocity, etc.
 
 This code should provide example usage for most features of
 the MMA8452Q 3-axis, I2C accelerometer. In the loop function
 the accelerometer interrupt outputs will be polled, and either
 the x/y/z accel data will be output, or single/double-taps,
 portrait/landscape changes will be announced to the serial port.
 Feel free to comment/uncomment out some of the Serial.print 
 lines so you can see the information you're most intereseted in.
 
 The skeleton is here, feel free to cut/paste what code you need.
 Play around with the settings in initMMA8452Q. Try running the
 code without printing the accel values, to really appreciate
 the single/double-tap and portrait landscape functions. The
 P/L stuff is really neat, something not many accelerometers have.
 
 Hardware setup:
 MMA8452 Breakout ------------ Arduino
 3.3V --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 INT2 ---------------------- D5
 INT1 ---------------------- D4
 GND ---------------------- GND
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors worked for me. They should be on the breakout
 board.
 
 Note: The MMA8452 is an I2C sensor, however this code does
 not make use of the Arduino Wire library. Because the sensor
 is not 5V tolerant, we can't use the internal pull-ups used
 by the Wire library. Instead use the included i2c.h, defs.h and types.h files.
 */
#include "i2c.h"  // not the wire library, can't use pull-ups
#include <Adafruit_CharacterOLED.h> //include the OLED library to control LCD

Adafruit_CharacterOLED lcd(OLED_V2, 6, 7, 8, 10, 11, 12, 13); // initialize the LCD library with the numbers of the interface pins

// Define registers per MMA8452Q, Rev 4.1, 08/2011 3-Axis, 12-bit/8-bit Digital Accelerometer
// Freescale Semiconductor Data Sheet
#define STATUS           0x00
#define OUT_X_MSB        0x01    
#define OUT_X_LSB        0x02
#define OUT_Y_MSB        0x03
#define OUT_Y_LSB        0x04
#define OUT_Z_MSB        0x05
#define OUT_Z_LSB        0x06
#define SYSMOD           0x0B
#define INT_SOURCE       0x0C
#define WHO_AM_I         0x0D   
#define XYZ_DATA_CFG     0x0E
#define HP_FILTER_CUTOFF 0x0F
#define PL_STATUS        0x10
#define PL_CFG           0x11
#define PL_COUNT         0x12
#define PL_BF_ZCOMP      0x13
#define P_L_THS_REG      0x14
#define FF_MT_CFG        0x15
#define FF_MT_SRC        0x16
#define FF_MT_THS        0x17
#define FF_MT_COUNT      0x18
#define TRANSIENT_CFG    0x1D
#define TRANSIENT_SRC    0x1E
#define TRANSIENT_THS    0x1F
#define TRANSIENT_COUNT  0x20
#define PULSE_CFG        0x21
#define PULSE_SRC        0x22
#define PULSE_THSX       0x23
#define PULSE_THSY       0x24
#define PULSE_THSZ       0x25
#define PULSE_TMLT       0x26
#define PULSE_LTCY       0x27
#define PULSE_WIND       0x28
#define ASLP_COUNT       0x29
#define CTRL_REG1        0x2A
#define CTRL_REG2        0x2B
#define CTRL_REG3        0x2C
#define CTRL_REG4        0x2D
#define CTRL_REG5        0x2E
#define OFF_X            0x2F
#define OFF_Y            0x30
#define OFF_Z            0x31

// The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define SA0 1
#if SA0
#define MMA8452_ADDRESS 0x1D  // SA0 is high, 0x1C if low
#else
#define MMA8452_ADDRESS 0x1C
#endif

// Set the scale below either 2, 4 or 8
const byte SCALE = 2;  // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
// Set the output data rate below. Value should be between 0 and 7
const byte dataRate = 3;  // 0=800Hz, 1=400, 2=200, 3=100, 4=50, 5=12.5, 6=6.25, 7=1.56

// Pin definitions
int int1Pin = 4;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int int2Pin = 5;
char contrastPin = A0;
int Contrast = 20;

int accelCount[3];  // Stores the 12-bit signed value
float accelG[3];  // Stores the real accel value in g's
int count = 0;
bool sleepMode = false;
int xdot;
int ydot;
int zdot;

void setup()
{
  byte c;

  Serial.begin(9600);

  lcd.begin(16, 2);// Initialize the LCD with 16 characters and 2 lines
 
  // Set up the interrupt pins, they're set as active high, push-pull
  pinMode(int1Pin, INPUT);
  digitalWrite(int1Pin, LOW);
  pinMode(int2Pin, INPUT);
  digitalWrite(int2Pin, LOW);
  analogWrite(contrastPin, Contrast);

  // Read the WHO_AM_I register, this is a good test of communication
  c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {  
    MMA8452Reset(); // Start by resetting sensor device to default settings
    initMMA8452(SCALE, dataRate);  // init the accelerometer if communication is OK
    Serial.println("MMA8452Q is online...");
    MMA8452Offsets(); // User defined compensation for x/y/z acceleration errors
  }
  else
  {
    Serial.print("Could not connect to MMA8452Q: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  

  static byte source;
  
  // If int1 goes high, all data registers have new data
  if (digitalRead(int1Pin)==1)  // Interrupt pin, should probably attach to interrupt function
  {
    readAccelData(accelCount);  // Read the x/y/z adc values

    /* 
     //Below we'll print out the ADC values for acceleration
     for (int i=0; i<3; i++)
     {
     Serial.print(accelCount[i]);
     Serial.print("\t\t");
     }
     Serial.println();
     */

    // Now we'll calculate the accleration value into actual g's
    for (int i=0; i<3; i++)
      accelG[i] = (float) accelCount[i]/((1<<12)/(2*SCALE));  // get actual g value, this depends on scale being set
    // Print out values
    for (int i=0; i<3; i++)
    {
      Serial.print(accelG[i], 3);  // Print g values to thousandths of a g
      Serial.print("\t\t");  // tabs in between axes
   
    }
    Serial.println();
    
    int deltat = millis() - count;
    if (deltat > 500) { // update LCD once per half-second independent of read rate
    lcd.clear();
    lcd.setCursor(0,0); 
    lcd.print(" x ");
    lcd.setCursor(5,0); 
    lcd.print(" y ");
    lcd.setCursor(10,0); 
    lcd.print("z (mg)");
    lcd.setCursor(0,1); 
    lcd.print((int)(1000*accelG[0])); lcd.print(" ");
    lcd.setCursor(7,1); 
    lcd.print((int)(1000*accelG[1])); lcd.print(" ");
    lcd.setCursor(12,1); 
    lcd.print((int)(1000*accelG[2])); lcd.print(" ");
    count = millis();
    }
  }

  // If int2 goes high, either p/l has changed or there's been a single/double tap
  if (digitalRead(int2Pin)==1)
  {
    source = readRegister(INT_SOURCE);  // Read the interrupt source reg.
    
  // Manage sleep/wake interrupts
  if((readRegister(INT_SOURCE) & 0x80) == 0x80) { // Check if interrupt source is sleep/wake interrupt

  if(!sleepMode) {
  Serial.println("entering sleep mode");
    sleepMode = TRUE;
  }
  else {
  Serial.println("exiting sleep mode");
    sleepMode = FALSE;
  }
  readRegister(SYSMOD); // clear sleep interrupt
  }

if ((source & 0x10)==0x10)  // If the p/l bit is set, go check those registers
      portraitLandscapeHandler();
    else if ((source & 0x08)==0x08)  // If tap register is set go check that
      tapHandler();
          else if ((source & 0x04)==0x04)  // Otherwise, if motion detection is set go check that
      motionDetect();
  }
//  delay(100);  // Delay here for visibility
}

void readAccelData(int * destination)
{
  byte rawData[6];  // x/y/z accel register data stored here

  readRegisters(0x01, 6, &rawData[0]);  // Read the six raw data registers into data array

  // Loop to calculate 12-bit ADC and g value for each axis
  for (int i=0; i<6; i+=2)
  {
    destination[i/2] = ((rawData[i] << 8) | rawData[i+1]) >> 4;  // Turn the MSB and LSB into a 12-bit value
    if (rawData[i] > 0x7F)
    {  
      // If the number is negative, we have to make it so manually (no 12-bit data type)
      destination[i/2] = ~destination[i/2] + 1;
      destination[i/2] *= -1;  // Transform into negative 2's complement #
    }
  }
}


  

// This function will read the status of the tap source register.
// Print if there's been a single or double tap, and on what axis.
void tapHandler()
{
  byte source = readRegister(PULSE_SRC);  // Reads the PULSE_SRC register

  if ((source & 0x10)==0x10)  // If AxX bit is set
  {
    if ((source & 0x08)==0x08)  // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on X");  // tabbing here for visibility
    else
      Serial.print("Single (1) tap on X");

    if ((source & 0x01)==0x01)  { // If PoIX is set
      Serial.println(" -");
      lcd.setCursor(0,0); 
      lcd.print("-x  "); }
    else {
      Serial.println(" +");
      lcd.setCursor(0,0); 
      lcd.print("+x  "); }
  }
  if ((source & 0x20)==0x20)  // If AxY bit is set
  {
    if ((source & 0x08)==0x08)  // If DPE (double pulse) bit is set
      Serial.print("    Double Tap (2) on Y");
    else
      Serial.print("Single (1) tap on Y");

    if ((source & 0x02)==0x02) { // If PoIY is set
      Serial.println(" -");
      lcd.setCursor(5,0); 
      lcd.print("-y  "); }
    else {
      Serial.println(" +");
      lcd.setCursor(5,0); 
      lcd.print("+y  "); }
  }
  if ((source & 0x40)==0x40)  // If AxZ bit is set
  {
    if ((source & 0x08)==0x08)  // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on Z");
    else
      Serial.print("Single (1) tap on Z");
    if ((source & 0x04)==0x04) { // If PoIZ is set
      Serial.println(" -"); 
      lcd.setCursor(9,0); 
      lcd.print("-z (mg)"); }
    else {
      Serial.println(" +");
      lcd.setCursor(9,0); 
      lcd.print("+z (mg)"); }
  }
}

// This function will read the p/l source register and
// print what direction the sensor is now facing
void portraitLandscapeHandler()
{
  byte pl = readRegister(0x10);  // Reads the PL_STATUS register
  switch((pl&0x06)>>1)  // Check on the LAPO[1:0] bits
  {
  case 0:
    Serial.print("Portrait up, ");
    break;
  case 1:
    Serial.print("Portrait Down, ");
    break;
  case 2:
    Serial.print("Landscape Right, ");
    break;
  case 3:
    Serial.print("Landscape Left, ");
    break;
  }
  if (pl&0x01)  // Check the BAFRO bit
    Serial.print("Back");
  else
    Serial.print("Front");
  if (pl&0x40)  // Check the LO bit
    Serial.print(", Z-tilt!");
  Serial.println();
}

// This function will read the motion detection source register and
// print motion direction
void motionDetect()
{
    byte source = readRegister(FF_MT_SRC);
  if((source >> 7) == 1) {  // If Event Active flag set in the FF_MT_SRC register

    int deltat = millis() - count;
  if ((source & 0x02)==0x02)  // If XHE bit is set, x-motion detected
  {
    if ((source & 0x01)==0x01)  { // If XHP is 1, x event was negative g
      Serial.println(" -");
      lcd.setCursor(0,0); 
      lcd.print(" x- "); 
      count = millis();
    }
    else {
      Serial.println(" +");
      lcd.setCursor(0,0); 
      lcd.print(" x+ "); 
    }
  }
  if ((source & 0x08)==0x08)  // If YHE bit is set, y-motion detected
  {
    if ((source & 0x04)==0x04) { // If YHP is set, y event was negative g
      Serial.println(" -");
      lcd.setCursor(5,0); 
      lcd.print(" y- "); 
      count = millis();
        }
    else {
      Serial.println(" +");
      lcd.setCursor(5,0); 
      lcd.print(" y+ "); 
        }
  }
  if ((source & 0x20)==0x20)  // If ZHE bit is set, z-motion detected
  {
    if ((source & 0x10)==0x10) { // If ZHP is set
      Serial.println(" -"); 
      lcd.setCursor(9,0); 
      lcd.print(" z-(mg)"); 
      count = millis();
        }
    else {
      Serial.println(" +");
      lcd.setCursor(9,0); 
      lcd.print(" z+(mg)"); 
        }
  }
}
}

// Set up sensor software reset
void MMA8452Reset() 
{
writeRegister(CTRL_REG2, 0x40); // set reset bit to 1 to assert software reset to zero at end of boot process
}

// Allow user compensation of acceleration errors
void MMA8452Offsets()
{
   MMA8452Standby();  // Must be in standby to change registers
   
   // Factory settings are pretty good; the settings below produce 1 mg error or less at 2 g full scale! For the device at rest on my table 
   // these values partially compensate for the slope of the table and the slope of the sensor in my breadboard. It is a pretty stable setup!
   // For negative values use 2's complement, i.e., -2 mg = 0xFF, etc.
   writeRegister(OFF_X, 0xF9); // X-axis compensation; this is -14 mg
   writeRegister(OFF_Y, 0x01); // Y-axis compensation; this is +2 mg
   writeRegister(OFF_Z, 0x00); // z-axis compensation; this is  0 mg adjustment
   
   MMA8452Active();  // Set to active to start reading
}

// Initialize the MMA8452 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
// Feel free to modify any values, these are settings that work well for me.
void initMMA8452(byte fsr, byte dataRate)
{
  MMA8452Standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
  if ((fsr==2)||(fsr==4)||(fsr==8))
    writeRegister(XYZ_DATA_CFG, fsr >> 2);  
  else
    writeRegister(XYZ_DATA_CFG, 0);

  // Setup the 3 data rate bits, from 0 to 7
  writeRegister(CTRL_REG1, readRegister(CTRL_REG1) & ~(0x38));
  if (dataRate <= 7)
    writeRegister(CTRL_REG1, readRegister(CTRL_REG1) | (dataRate << 3));  
    
// These settings have to do with setting up the sleep mode and should probably be broken up into a separate function
// set Auto-WAKE sample frequency when the device is in sleep mode

     writeRegister(0x29, 0x40 ); // sleep after ~36 seconds of inactivity at 6.25 Hz ODR

     writeRegister(CTRL_REG1, readRegister(CTRL_REG1) & ~(0xC0)); // clear bits 7 and 8
     writeRegister(CTRL_REG1, readRegister(CTRL_REG1) |  (0xC0)); // select 1.56 Hz sleep mode sample frequency for low power

  // set sleep power mode scheme
     writeRegister(CTRL_REG2, readRegister(CTRL_REG2) & ~(0x18)); // clear bits 3 and 4
     writeRegister(CTRL_REG2, readRegister(CTRL_REG2) |  (0x18)); // select low power mode
     
  // Enable auto SLEEP
     writeRegister(CTRL_REG2, readRegister(CTRL_REG2) & ~(0x04)); // clear bit 2
     writeRegister(CTRL_REG2, readRegister(CTRL_REG2) |  (0x04)); // enable auto sleep mode

  // set sleep mode interrupt scheme
     writeRegister(CTRL_REG3, readRegister(CTRL_REG3) & ~(0x3C)); // clear bits 3, 4, 5, and 6
     writeRegister(CTRL_REG3, readRegister(CTRL_REG3) |  (0x3C)); // select wake on transient, orientation change, pulse, or freefall/motion detect
     
   // Enable Auto-SLEEP/WAKE interrupt
     writeRegister(CTRL_REG4, readRegister(CTRL_REG4) & ~(0x80)); // clear bit 7
     writeRegister(CTRL_REG4, readRegister(CTRL_REG4) |  (0x80)); // select  Auto-SLEEP/WAKE interrupt enable
   
  // Set up portrait/landscape registers - 4 steps:
  // 1. Enable P/L
  // 2. Set the back/front angle trigger points (z-lock)
  // 3. Set the threshold/hysteresis angle
  // 4. Set the debouce rate
  // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
  writeRegister(PL_CFG, 0x40);        // 1. Enable P/L
 // writeRegister(PL_BF_ZCOMP, 0x44); // 2. 29deg z-lock (don't think this register is actually writable)
 // writeRegister(P_L_THS_REG, 0x84); // 3. 45deg thresh, 14deg hyst (don't think this register is writable either)
  writeRegister(PL_COUNT, 0x50);      // 4. debounce counter at 100ms (at 800 hz)

  /* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
  writeRegister(PULSE_CFG, 0x7F);  // 1. enable single/double taps on all axes
  // writeRegister(PULSE_CFS, 0x55);  // 1. single taps only on all axes
  // writeRegister(PULSE_CFS, 0x6A);  // 1. double taps only on all axes
  writeRegister(PULSE_THSX, 0x04);  // 2. x thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(PULSE_THSY, 0x04);  // 2. y thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(PULSE_THSZ, 0x04);  // 2. z thresh at 0.25g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(PULSE_TMLT, 0x30);  // 3. 2.55s time limit at 100Hz odr, this is very dependent on data rate, see the app note
  writeRegister(PULSE_LTCY, 0xA0);  // 4. 5.1s 100Hz odr between taps min, this also depends on the data rate
  writeRegister(PULSE_WIND, 0xFF);  // 5. 10.2s (max value)  at 100 Hz between taps max

  // Set up motion detection
  writeRegister(FF_MT_CFG, 0x58); // Set motion flag on x and y axes
  writeRegister(FF_MT_THS, 0x84); // Clear debounce counter when condition no longer obtains, set threshold to 0.25 g
  writeRegister(FF_MT_COUNT, 0x8); // Set debounce to 0.08 s at 100 Hz

  // Set up interrupt 1 and 2
  writeRegister(CTRL_REG3, readRegister(CTRL_REG3) & ~(0x02)); // clear bits 0, 1 
  writeRegister(CTRL_REG3, readRegister(CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts
     
 // writeRegister(0x2C, 0x02);  // Active high, push-pull interrupts

  writeRegister(CTRL_REG4, readRegister(CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
  writeRegister(CTRL_REG4, readRegister(CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled
   
  writeRegister(CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2

  MMA8452Active();  // Set to active to start reading
}

// Sets the MMA8452 to standby mode.
// It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(0x2A);
  writeRegister(CTRL_REG1, c & ~(0x01));
}

// Sets the MMA8452 to active mode.
// Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(0x2A);
  writeRegister(CTRL_REG1, c | 0x01);
}

// Read i registers sequentially, starting at address into the dest byte array
void readRegisters(byte address, int i, byte * dest)
{
  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS<<1)); // write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address);	// write register address
  i2cWaitForComplete();

  i2cSendStart();
  i2cSendByte((MMA8452_ADDRESS<<1)|0x01); // write 0xB5
  i2cWaitForComplete();
  for (int j=0; j<i; j++)
  {
    i2cReceiveByte(TRUE);
    i2cWaitForComplete();
    dest[j] = i2cGetReceivedByte(); // Get MSB result
  }
  i2cWaitForComplete();
  i2cSendStop();

  cbi(TWCR, TWEN); // Disable TWI
  sbi(TWCR, TWEN); // Enable TWI
}

// Read a single byte from address and return it as a byte
byte readRegister(uint8_t address)
{
  byte data;

  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address);	// Write register address
  i2cWaitForComplete();

  i2cSendStart();

  i2cSendByte((MMA8452_ADDRESS<<1)|0x01); // Write 0xB5
  i2cWaitForComplete();
  i2cReceiveByte(TRUE);
  i2cWaitForComplete();

  data = i2cGetReceivedByte();	// Get MSB result
  i2cWaitForComplete();
  i2cSendStop();

  cbi(TWCR, TWEN);	// Disable TWI
  sbi(TWCR, TWEN);	// Enable TWI

  return data;
}

// Writes a single byte (data) into address
void writeRegister(unsigned char address, unsigned char data)
{
  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS<<1)); // Write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address);	// Write register address
  i2cWaitForComplete();

  i2cSendByte(data);
  i2cWaitForComplete();

  i2cSendStop();
}

