/*  Use the Arduino IDE to load this code on an Arduino Nano 33 IOT (master) to periodically 
    read/write bytes from/to the AquaPing (slave) on the I2C bus. The Arduino connects to the 
    host PC with a USB-to-Micro-B cable and communicates via serial UART at 9600 baud. 
    Requires the companion program aquaping_gui.py on the PC.
    
    The Arduino can monitor the state of the leak and noise alerts via interrupts on LEAK_PIN 
    and NOISE_PIN, respectively. These interrupts have lower latency compared to periodic 
    reads on I2C. 

    The sensor status is polled at a period set by polling_interval in milliseconds. Each I2C 
    read is signaled by blinking the Arduino LED. To allow the user to send sensor configuration 
    commands with reasonably low latency, the GUI is polled for input at an interval set by serial_poll.

    Modify the code as needed for different Arduino hardware. Most of the error checking
    was provided by claude.ai.
  
    Licensed under Creative Commons
    MicroPhonon October 2026 */

#include <Wire.h>

#define LEAK_PIN 3        //Connect D3 on Nano 33 to ALARM on sensor (interrupt-capable on IoT and BLE)
#define NOISE_PIN 2       //Connect D2 on Nano 33 to NOISE on sensor (interrupt-capable on IoT and BLE)
#define SBYTES 12         //Number of status bytes
#define ISTRING 8         //Overflow characters on serial input

const unsigned long polling_interval = 1000; //Delay in ms; raise this to decrease polling rate
const long serial_poll = 200;                //Specified in ms
const unsigned long min_interval = 0;        //Delay to escape polling loop when alarm occurs; should be 0
const unsigned long cmd_gap = 40;            //A command ends after this many ms with no new input bytes
const uint8_t I2C_SLAVE = 0x77;              //AquaPing slave address
const unsigned long baud_rate = 9600;        //Ignored by USB serial, kept for compatibility

int i, j, k = 0;                             //k is number of bytes in the current command
bool overflow = false;                       //True if the command exceeded ISTRING bytes
volatile uint8_t leak_edge, noise_edge;      //Glitch flags for interrupt handlers         
uint8_t Status_array[SBYTES];
uint8_t input_string[ISTRING];
unsigned long currentMillis, trigger, lastRx;
unsigned long previousMillis = 0;

void alarm_handler() //No delays allowed inside interrupt; set flags only, no delays or printing
{
  if (digitalRead(LEAK_PIN) == HIGH) leak_edge = 1;
  else leak_edge = 0;
}

void noise_handler() //Same rules as alarm_handler
{
  if (digitalRead(NOISE_PIN) == HIGH) noise_edge = 1;
  else noise_edge = 0;
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);     //LED off (active HIGH on Nano 33)
  pinMode(LEAK_PIN, INPUT_PULLDOWN);  //Prevents a floating/unconnected ALARM line from firing interrupts
  pinMode(NOISE_PIN, INPUT_PULLDOWN); //Same protection for the noise line
  Serial.begin(baud_rate);            //USB virtual serial
  while (!Serial && millis() < 3000); //Wait up to 3 s for a terminal; run standalone otherwise
  Wire.begin();                       //I2C on A4 (SDA) / A5 (SCL)
  attachInterrupt(digitalPinToInterrupt(LEAK_PIN), alarm_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(NOISE_PIN), noise_handler, CHANGE);
  for (i = 0; i < SBYTES; i++) Status_array[i] = 0; //Clear status array
  previousMillis = millis();
}

void loop()
{
  /* Check for input from control program on the serial port.
  No terminator character is required and none is added. 
  A command is complete when no new byte arrives for cmd_gap ms. */
  if (Serial.available() > 0)
  {
    k = 0;
    overflow = false;
    lastRx = millis();
    while (millis() - lastRx < cmd_gap)
    {
      while (Serial.available() > 0)
      {
        uint8_t s = Serial.read();
        lastRx = millis();
        if (k < ISTRING) input_string[k++] = s;
        else overflow = true;
      }
    }
    if (overflow)
    {
      Serial.println("Command too long; ignored.");
    }
    else
    {
      //Send byte string to slave on I2C exactly as received 
      Wire.beginTransmission(I2C_SLAVE);
      for (i = 0; i < k; i++) Wire.write(input_string[i]);
      uint8_t err = Wire.endTransmission(); //0 if successful
      if (err != 0)
      {
        Serial.print("I2C command error ");
        Serial.println(err);
      }
    }
    k = 0;
  }
  //End of serial input read and I2C send

  currentMillis = millis(); 
  //Check for leak or noise alert from interrupts; do not wait for polling loop to finish.
  //A flag is 1 only if the pin was still high at the last edge, so glitches are ignored.
  if (leak_edge == 1 || noise_edge == 1) trigger = min_interval; 
  else trigger = polling_interval;
  leak_edge = 0;
  noise_edge = 0;

  /* The following conditional code block retrieves data from sensor.
     This occurs at designated polling time; otherwise skip it and perform another polling iteration. */
  if (currentMillis - previousMillis >= trigger)
  {
    digitalWrite(LED_BUILTIN, HIGH); //Turn on Arduino LED
    previousMillis = currentMillis;
    Wire.requestFrom(I2C_SLAVE, (uint8_t)SBYTES);
    i = Wire.available();
    if (i == 0) //Getting here would be unusual, but allow slave to reset just in case
    {
      Serial.println("No bytes available on I2C. Retry...");
      delay(700); //Wait for I2C bus to reset
      Wire.requestFrom(I2C_SLAVE, (uint8_t)SBYTES);
      i = Wire.available();
      if (i == 0) Serial.println("Retry failed."); //Communication problem. Check I2C configuration
      else Serial.println("Retry successful.");
    }
    //Get data from slave via I2C, convert to decimal, and send to master on serial port
    j = 0;
    while (Wire.available() && j < SBYTES) //Slave may send fewer than requested bytes
    {
      Status_array[j] = Wire.read(); //Receive a byte
      Serial.print(Status_array[j], DEC); //Print each byte
      Serial.print(" "); //Space between bytes
      j++;
    }
    Serial.println();
    digitalWrite(LED_BUILTIN, LOW); //LED off

    if (j != SBYTES)
    { 
      Serial.print("Short read: ");
      Serial.print(j);
      Serial.println(" of 12 bytes");
    }
  }
  delay(serial_poll); //Needed to read serial port
} //End of main polling loop
