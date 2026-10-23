#include <Wire.h>
/* Periodically read/write bytes from/to the AquaPing slave on the I2C bus using an 
Arduino Nano 33 IOT. Connect to the Arduino serial port using a USB Micro-B cable and 
a terminal program (eg. putty). The AquaPing is slave to the Arduino Nano 33 IoT master 
on I2C with 3V3 logic. The Nano 33 IoT does not require or use external pullup resistors. 
Check if your Arduino variant requires them.

   Wiring for Nano 33 IOT:
     A4 (SDA) -> SDA on AquaPing header
     A5 (SCL) -> SCL on AquaPing header
     D2       -> NOISE pin on AquaPing header (optional)
     D3       -> ALARM pin on AquaPing header (optional)
     3V3      -> 3V3
     GND      -> GND

The sensor status is polled at a period set by polling_interval in milliseconds. The retrieved
status bytes are then sent to the terminal in decimal format. Each I2C read is signaled by blinking 
the Arduino LED. 

The Arduino monitors sensor leak and noise alerts on the first two data bytes. There is a low latency
alert option available using interrupts attached to LEAK_PIN and NOISE_PIN. 

Command characters are read from the serial terminal as user input. Characters are echoed to
the terminal then formatted into command strings as a sequence of hex bytes that are sent to the slave.
Syntax is in separate documentation. To allow the user to send sensor configuration commands 
with reasonably low latency, the terminal is polled for input at an interval set by serial_poll.

Licensed under Creative Commons
MicroPhonon October 2026                */

#define LEAK_PIN 3   //Connect D3 on Nano 33 IoT to ALARM pin on sensor
#define NOISE_PIN 2 //Connect D2 to NOISE pin on sensor
#define SBYTES 12  //Number of status bytes
#define ISTRING 24  //Max characters from terminal input; for error checking
#define CSTRING 16  //Max characters sent to slave on I2C

const long polling_interval = 5000; //Specified in ms
const long serial_poll = 200; //Specified in ms
const long min_interval = 0;  //Delay to escape polling loop when alarm occurs; should be 0
const uint8_t I2C_SLAVE = 0x77;  //AquaPing slave address
const uint16_t baud_rate = 9600;   

int i, j, k = 0, bsize = 0;
volatile uint8_t leak_edge, noise_edge;  //Glitch flags for the two interrupt handlers 
volatile uint8_t first_pass = 1;
volatile uint8_t leak_print_once = 1, noise_print_once = 1;
uint8_t Status_array[SBYTES];
char input_string[ISTRING], carray[CSTRING];
unsigned long currentMillis, trigger;

void alarm_handler() //Leak alert interrupt. No delays allowed inside interrupt. 
{
  if (digitalRead(LEAK_PIN) == HIGH) leak_edge = 1; //Rising edge detected
  else leak_edge = 0; //Falling edge detected
}

void noise_handler() //Same rules as alarm_handler: set flags only, no delays or printing
{
  if (digitalRead(NOISE_PIN) == HIGH) noise_edge = 1; //Rising edge detected
  else noise_edge = 0; //Falling edge detected
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); //Amber LED on Nano 33 IOT
  digitalWrite(LED_BUILTIN, LOW);  //LED off (active HIGH on Nano 33 IoT)
  pinMode(LEAK_PIN, INPUT_PULLDOWN); //Prevents a floating/unconnected ALARM line from firing interrupts
  pinMode(NOISE_PIN, INPUT_PULLDOWN); //Same protection for the noise line
  Serial.begin(baud_rate);              
  while (!Serial && millis() < 3000);    //Wait up to 3 s for a terminal; run standalone otherwise
  Wire.begin();   //I2C on A4 (SDA) / A5 (SCL)
  attachInterrupt(digitalPinToInterrupt(LEAK_PIN), alarm_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(NOISE_PIN), noise_handler, CHANGE);
  for (i = 0; i < SBYTES; i++) Status_array[i] = 0; //Clear status array
}

unsigned long previousMillis = millis();

void loop()
{
  //Flush the serial port on first pass
  if (first_pass == 1)
  {
    while (Serial.available() > 0) Serial.read();
    first_pass = 0;
  }
  //Check for user input from the terminal
  k = 0;
  while (Serial.available() > 0)
  {
    char s = Serial.read(); // Receive a byte as character
    if (s != 0x0D) //Check for the return character
    {
      input_string[k] = s;
      k++;
      if (k == ISTRING)
      {
        // Big string encountered, likely caused by terminal program startup
        while (Serial.available() > 0) Serial.read(); //Finish reading garbage input
        Serial.println();
        Serial.println("Terminal input ignored.");
        for (i = 0; i < ISTRING; i++) input_string[i] = 0x00;
        break;
      }
    }
    else //Return byte entered, command entry complete, message length k
    {
      Serial.print("Input hex bytes read ");
      for (i = 0; i < k; i++)
      {
        Serial.print(input_string[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      //Check to see if commands entered correctly
      if (create_command(k, input_string) == 1)
      {
        if (carray[0] == 0x72) i = 2; //Reset found in command string
        else
        {
          i = 0;
          while (carray[i] != 0x00) i++;
        }
        bsize = i;
        Serial.print("Command string converted to ");
        Serial.print(bsize, DEC);
        Serial.print(" hex bytes: ");
        for (i = 0; i < bsize; i++)
        {
          Serial.print(carray[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        //Send the byte string to slave on I2C
        Wire.beginTransmission(I2C_SLAVE);
        for (i = 0; i < bsize; i++) Wire.write(carray[i]);
        Wire.endTransmission(); //Should equal 0 if successful
      }
      else Serial.println("Incorrectly formatted command string.");
      //Reset the input character counter and array
      k = 0;
      bsize = 0;
      for (i = 0; i < ISTRING; i++) input_string[i] = 0x00;
      for (i = 0; i < CSTRING; i++) carray[i] = 0x00;
    }
  } //End of terminal input code

  currentMillis = millis(); 
 /* Check for leak or noise alert from interrupts; do not wait for polling loop to finish.
  A flag is 1 only if the pin was still high at the last edge, so glitches are ignored. */
  if (leak_edge == 1 || noise_edge == 1) trigger = min_interval; 
  else trigger = polling_interval;
  leak_edge = 0;
  noise_edge = 0;

  /* The following conditional code block retrieves data from sensor.
     This occurs at designated polling time; otherwise skip it and perform another polling iteration. */
  if (currentMillis - previousMillis >= trigger)
  {
    digitalWrite(LED_BUILTIN, HIGH); //Enable LED on Arduino
    previousMillis = currentMillis;
    Wire.requestFrom(I2C_SLAVE, (uint8_t)SBYTES);
    i = Wire.available();
    if (i == 0) //Getting here would be unusual, but allow slave to reset just in case
    {
      Serial.println("No bytes available on I2C. Retry...");
      delay(700); //Wait for I2C bus to reset
      Wire.requestFrom(I2C_SLAVE, (uint8_t)SBYTES);
      i = Wire.available();
      if (i == 0) Serial.print("Retry failed."); //Communication problem. Check I2C configuration
      else Serial.println("Retry successful.");
    }
    //Get data from slave via I2C
    j = 0;
    Serial.println("Status bytes retrieved from sensor:");
    while (Wire.available() && j < SBYTES) // slave may send less than requested bytes
    {
      Status_array[j] = Wire.read(); // receive a byte
      Serial.print(Status_array[j], DEC); // print each byte in decimal format
      Serial.print(" "); //space between bytes
      j++;
    }
    Serial.println();
    digitalWrite(LED_BUILTIN, LOW); //Arduino LED off

    if (Status_array[0] == 1 && leak_print_once == 1)
    {
      Serial.println("Leak detected!");
      leak_print_once = 0;
    }
    if (Status_array[0] == 0 && leak_print_once == 0)
    {
      Serial.println("Leak alert cleared.");
      leak_print_once = 1;
    }

    if (Status_array[1] == 1 && noise_print_once == 1)
    {
      Serial.println("Noise detected!");
      noise_print_once = 0;
    }
    if (Status_array[1] == 0 && noise_print_once == 0)
    {
      Serial.println("Noise alert cleared.");
      noise_print_once = 1;
    }
  }
  delay(serial_poll);
} //End of main polling loop

//This function parses and processes user input commands; see user guide for syntax
int create_command(int k, char input_string[]) //Returns 1 if user entered a valid command string
//Valid commands are lower case: r, a, p, b, t, e, o, l
{
  int jj = 0;
  int len = 0; //Number of bytes in command string
  char c1, c2, c3, c4, arraysize;
  for (jj = 0; jj < k; jj++)
  {
    c1 = input_string[jj++]; //First character must be lower case or space
    if (c1 == 0x20) ; //Do nothing if space encountered
    else if (islower(c1))
    {
      if (c1 == 0x72) //Reset; ASCII r
      {
        carray[0] = 0x72;
        carray[1] = 0x72;
        len = 2;
        return 1;
      }
      else if (c1 == 0x6C) //LED status; ASCII l
      {
        carray[len++] = 0x6C;
        c1 = input_string[jj++];
        c2 = input_string[jj];
        if (c1 == 0x30 || c1 == 0x31) carray[len++] = c1; //Only 0 or 1 are possible
        else return 0;
        if (c2 == 0x00) return 1;
        else if (c2 == 0x20) ;
        else return 0;
      }
      else if (c1 == 0x61) //Alarm status; ASCII a
      {
        carray[len++] = 0x61;
        c1 = input_string[jj++];
        c2 = input_string[jj];
        if (c1 == 0x30 || c1 == 0x31) carray[len++] = c1; //Only 0 or 1 are possible
        else return 0;
        if (c2 == 0x00) return 1;
        else if (c2 == 0x20) ;
        else return 0;
      }
      else if (c1 == 0x70) //Power/sleep status; ASCII p
      {
        carray[len++] = 0x70;
        c1 = input_string[jj++];
        c2 = input_string[jj];
        if (c1 == 0x30 || c1 == 0x31) carray[len++] = c1; //Only 0 or 1 are possible
        else return 0;
        if (c2 == 0x00) return 1;
        else if (c2 == 0x20) ;
        else return 0;
      }
      else if (c1 == 0x6F) //Polling period; ASCII o
      {
        carray[len++] = 0x6F;
        c1 = input_string[jj++];
        c2 = input_string[jj];
        //Only allowed arguments are single digits 1--9
        if (c1 >= 0x31 && c1 <= 0x39) carray[len++] = c1 - 48; //Convert ASCII to hex byte
        else return 0;
        if (c2 == 0x00) return 1;
        else if (c2 == 0x20) ;
        else return 0;
      }
      else if (c1 == 0x62) //Background count; ASCII b
      {
        carray[len++] = 0x62;
        c1 = input_string[jj++];
        c2 = input_string[jj++];
        c3 = input_string[jj++];
        c4 = input_string[jj];
        if (isdigit(c1) && isdigit(c2) && !isdigit(c3)) //Checks for 10--99
        {
          char byte_array[] = {c1, c2, 0x00}; //Null-terminated for atoi
          arraysize = atoi(byte_array);
          if (arraysize < 0x0A) return 0; //Return error if < 10
          carray[len++] = arraysize;
          jj = jj - 1;
          if (c3 == 0x00) return 1;
          else if (c3 == 0x20) ;
          else return 0;
        }
        else if (isdigit(c1) && isdigit(c2) && isdigit(c3) && !isdigit(c4))
        {
          char byte_array[] = {c1, c2, c3, 0x00}; //Checks for 100-255
          arraysize = atoi(byte_array);
          carray[len++] = arraysize;
          if (c4 == 0x00) return 1;
          else if (c4 == 0x20) ;
          else return 0;
        }
      }
      else if (c1 == 0x65) //Array size; ASCII e
      {
        carray[len++] = 0x65;
        c1 = input_string[jj++];
        c2 = input_string[jj++];
        c3 = input_string[jj++];
        c4 = input_string[jj];
        if (isdigit(c1) && isdigit(c2) && !isdigit(c3)) //Checks for 10--99
        {
          char byte_array[] = {c1, c2, 0x00};
          arraysize = atoi(byte_array);
          if (arraysize < 0x0A) return 0; //Return error if < 10
          carray[len++] = arraysize;
          jj = jj - 1;
          if (c3 == 0x00) return 1;
          else if (c3 == 0x20) ;
          else return 0;
        }
        else if (isdigit(c1) && isdigit(c2) && isdigit(c3) && !isdigit(c4))
        {
          char byte_array[] = {c1, c2, c3, 0x00}; //Checks for 100-255
          arraysize = atoi(byte_array);
          carray[len++] = arraysize;
          if (c4 == 0x00) return 1;
          else if (c4 == 0x20) ;
          else return 0;
        }
      }
      else if (c1 == 0x74) //Trigger size; ASCII t
      {
        carray[len++] = 0x74;
        c1 = input_string[jj++];
        c2 = input_string[jj++];
        c3 = input_string[jj++];
        c4 = input_string[jj];
        if (isdigit(c1) && isdigit(c2) && !isdigit(c3)) //Checks for 10--99
        {
          char byte_array[] = {c1, c2, 0x00};
          arraysize = atoi(byte_array);
          if (arraysize < 0x0A) return 0;
          carray[len++] = arraysize;
          jj = jj - 1;
          if (c3 == 0x00) return 1;
          else if (c3 == 0x20) ;
          else return 0;
        }
        else if (isdigit(c1) && isdigit(c2) && isdigit(c3) && !isdigit(c4)) //Checks for 100-255
        {
          char byte_array[] = {c1, c2, c3, 0x00};
          arraysize = atoi(byte_array);
          if (arraysize < 0x0A) return 0;
          carray[len++] = arraysize;
          if (c4 == 0x00) return 1;
          else if (c4 == 0x20) ;
          else return 0;
        }
      }
      else return 0;
    }
  }
  return 0; //Reached the end without a complete command (a missing return here is undefined behavior)
}

//Converts the polling rate byte into a decimal period (seconds) 
uint16_t Rate(uint8_t rindx)
{
  switch (rindx) {
    case 0x01: return 1;  //1 sec
    case 0x02: return 2;  //2 sec
    case 0x03: return 3;  //3 sec
    case 0x04: return 5;  //5 sec
    case 0x05: return 10; //10 sec
    case 0x06: return 15; //15 sec
    case 0x07: return 20; //20 sec
    case 0x08: return 25; //25 sec
    case 0x09: return 30; //30 sec
    default:   return 0;
  }
}
