/*  Use the Arduino IDE (with the Earle Philhower "Raspberry Pi Pico/RP2040" board package)
    to load this code on a Raspberry Pi Pico (master) to periodically read/write bytes
    from/to the AquaPing (slave) on the I2C bus. The Pico connects to the host PC with a
    USB-to-Micro-B cable and communicates via USB virtual serial (baud rate is ignored).
    Requires the companion program aquaping_gui.py on the PC.

    Wiring (Pico is 3.3 V logic; do not apply 5 V to any pin):
      GP4 (pin 6)  SDA  <-> AquaPing SDA   (needs external pull-up, e.g. 4.7k to 3.3 V)
      GP5 (pin 7)  SCL  <-> AquaPing SCL   (needs external pull-up, e.g. 4.7k to 3.3 V)
      GP3 (pin 5)  LEAK_PIN  <- ALARM on sensor
      GP2 (pin 4)  NOISE_PIN <- NOISE on sensor
      GND common with sensor

    GPIO23 drives the Pico's SMPS PS pin. Setting it HIGH forces PWM mode, which reduces
    the 3.3 V supply ripple (at the cost of some light-load efficiency). Plain Pico only;
    on a Pico W, GPIO23 is the wireless power pin and does not control the SMPS.

    The Pico can monitor the state of the leak and noise alerts via interrupts on LEAK_PIN
    and NOISE_PIN, respectively. These interrupts have lower latency compared to periodic
    reads on I2C.

    The sensor status is polled at a period set by polling_interval in milliseconds. Each I2C
    read is signaled by blinking the Pico LED. To allow the user to send sensor configuration
    commands with reasonably low latency, the GUI is polled for input at an interval set by serial_poll.

    Licensed under Creative Commons
    MicroPhonon October 2026 */

#include <Wire.h>

#define SDA_PIN 4         //I2C0 SDA
#define SCL_PIN 5         //I2C0 SCL
#define LEAK_PIN 3        //Connect GP3 to ALARM on sensor (any Pico GPIO supports interrupts)
#define NOISE_PIN 2       //Connect GP2 to NOISE on sensor
#define SMPS_PS_PIN 23    //Pico SMPS power-save control: HIGH = PWM mode, lower ripple
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
  //Reduce supply ripple first: force the on-board SMPS into PWM mode
  pinMode(SMPS_PS_PIN, OUTPUT);
  digitalWrite(SMPS_PS_PIN, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);       //GP25 on Pico
  digitalWrite(LED_BUILTIN, LOW);     //LED off (active HIGH)
  pinMode(LEAK_PIN, INPUT_PULLDOWN);  //Prevents a floating/unconnected ALARM line from firing interrupts
  pinMode(NOISE_PIN, INPUT_PULLDOWN); //Same protection for the noise line
  Serial.begin(baud_rate);            //USB virtual serial
  while (!Serial && millis() < 3000); //Wait up to 3 s for a terminal; run standalone otherwise
  Wire.setSDA(SDA_PIN);               //Must be set before Wire.begin()
  Wire.setSCL(SCL_PIN);
  Wire.begin();
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
    digitalWrite(LED_BUILTIN, HIGH); //Turn on LED
    previousMillis = currentMillis;
    Wire.requestFrom(I2C_SLAVE, (size_t)SBYTES);
    i = Wire.available();
    if (i == 0) //Getting here would be unusual, but allow slave to reset just in case
    {
      Serial.println("No bytes available on I2C. Retry...");
      delay(700); //Wait for I2C bus to reset
      Wire.requestFrom(I2C_SLAVE, (size_t)SBYTES);
      i = Wire.available();
      if (i == 0) Serial.println("Retry failed."); //Communication problem. Check I2C configuration
      else Serial.println("Retry successful.");
    }
    //Get data from slave via I2C, convert to decimal, and send to host on serial port
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
