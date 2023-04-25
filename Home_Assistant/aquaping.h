/*  Example code to interface the AquaPing leak sensor with Home Assistant using an ESP8266 WiFi module.  Requires installation of the ESPHome and the File editor Add-ons to modify this sketch. An example .yaml file is included in this directory. Developed and tested with a Lolin D1-mini, but should adapt to other devices using the Arduino IDE and equivalent libraries. This file must be placed in the directory config/esphome/ and shown as an #include in the yaml file. The sensor status is polled by the D1-mini at a period set by 'update_interval' in milliseconds. The retrieved status bytes are parsed, formatted, and sent to the HA frontend over WiFi and the serial terminal. 

The D1-mini module provides a UART to USB interface to allow serial communication with a PC. Any serial terminal program (eg. puTTY) should work; be sure to set equivalent baud rates. Configuration characters can be sent from the terminal as user input. Characters are echoed to the terminal then formatted into command strings as a sequence of hex bytes that are sent to the AquaPing. Syntax is in the user guide. 

Two-way communication between the AquaPing and D1-mini is via I2C. Pullup resistors are present on the module. The D1-mini can supply 3V3 external power to the sensor. Make sure a large electrolytic filter capacitor is on this line.

Consult the read.me file for more information.
 
Licensed under Creative Commons. Author: MPH MicroPhonon April 2023 */

#include "esphome.h"
#include <Wire.h>
#define SDA_PIN 4 //This is pin D2 on D1-mini
#define SCL_PIN 5 //This is pin D1 on D1-mini
#define ALARM_PIN D4 //Connect D4 to ALARM on AquaPing
// The noise pin interrupt is not used in this sketch. It is set in the yaml file on pin D3
  
#define SBYTES 12 //Number of status bytes
#define ISTRING 24 //Max characters from terminal input
#define CSTRING 16 //Max characters sent to slave on I2C
   
    const long update_interval = 10000; //Period to update Home Assistant with sensor data (ms)
    const long min_interval = 0; //Delay to escape polling loop and notify the network when alarm occurs; should be 0
    const uint16_t I2C_SLAVE = 0x77; //AquaPing address on I2C
    const uint16_t baud_rate = 9600; //UART serial baud rate for terminal program
  
    int i, j, error_code, k=0, bsize=0;
    volatile uint8_t flag_high = 0; 
    volatile uint8_t flag_low = 0; 
    volatile uint8_t first_pass = 1; 
    static char Status_array[SBYTES];
    char input_string[ISTRING], carray[CSTRING], led_byte = 0x01;
    unsigned long currentMillis, trigger; 
   
/* Home Assistant will query the D1 mini with a period defined by 'update_interval'. This is independent and distinct from the rate that AquaPing monitors the acoustic environment (polling_period). The HA GUI displays the 3 running counts and other monitoring parameters. */
  class AquaPing : public PollingComponent {
    public:
    Sensor *quiet_count = new Sensor();
    Sensor *leak_count = new Sensor();
    Sensor *noise_count = new Sensor();
    Sensor *polling_period = new Sensor();
    Sensor *event_array_size = new Sensor();
    Sensor *trigger_threshold = new Sensor();
    Sensor *background_array_size = new Sensor();
    AquaPing() : PollingComponent(update_interval) {}  
    float get_setup_priority() const override { return esphome::setup_priority::BUS; } //Access I2C bus
    
    void setup() override {
        pinMode(LED_BUILTIN, OUTPUT);
        Serial.begin(baud_rate);  //Start serial UART for terminal output via USB
        Wire.begin(SDA_PIN, SCL_PIN); //Setup I2C bus
        //attachInterrupt(digitalPinToInterrupt(ALARM_PIN), alarm_handler, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ALARM_PIN), alarm_low, FALLING);
        attachInterrupt(digitalPinToInterrupt(ALARM_PIN), alarm_high, RISING);
        for(i=0;i<SBYTES;i++)Status_array[i]=0; //Clear status array
    }
    
    unsigned long previousMillis = millis();
    
// Update Home Assistant using the following loop running at "update_interval" milliseconds.
void update() override {
  
    //Flush the serial port on first pass
    if(first_pass == 1)
    {
        while (Serial.available() > 0) Serial.read(); 
        first_pass = 0;
    } 
  
/* There is an option to check for user input using a serial terminal. This provides access to some configuration parameters not available on the HA front-end. Commands sent on the I2C bus. */
    k=0;
    while (Serial.available() > 0) 
    {
        char s = Serial.read(); // Receive a byte as character
        if (s != 0x0D) //Check for the return character
        {                     
            input_string[k]=s;
            k++;
            if (k == ISTRING) 
            {
                // Big string encountered, likely caused by terminal program startup
                while (Serial.available() > 0) Serial.read(); //Finish reading garbage input
                Serial.println();
                Serial.println("Terminal input ignored."); 
                for(i=0; i < ISTRING; i++) input_string[i]=0x00; 
                break;
            }
        }      
            else //Return byte entered, command entry complete, message length k
            { 
              Serial.print("Input hex bytes read ");
              for(i=0; i < k; i++)
                {  
                    Serial.print(input_string[i], HEX);  
                    Serial.print(" "); 
                }
                Serial.println();
                //Check to see if commands entered correctly
                if(create_command(k, input_string)==1)
                {
                  if(carray[0] == 0x72) i=2; //Reset found in command string
                  else
                  {
                      i=0;
                      while(carray[i] != 0x00) i++;                 
                  }
                  bsize = i;
                  Serial.print("Command string converted to "); 
                  Serial.print(bsize,DEC);
                  Serial.print(" hex bytes: ");
                  for(i=0; i < bsize; i++)
                  {
                    Serial.print(carray[i], HEX);  
                    Serial.print(" "); 
                  }
                 Serial.println();
                //Send the byte string to slave on I2C
                 Wire.beginTransmission(I2C_SLAVE); 
                 for (i=0; i < bsize; i++)  Wire.write(carray[i]);
                 Wire.endTransmission(); //Should equal 0 if successful
                }
             else  Serial.println("Incorrectly formatted command string."); 
             
             //Reset the input character counter and array
             k=0; 
             bsize=0; 
             for(i=0; i<ISTRING; i++) input_string[i]=0x00;
             for(i=0; i<CSTRING; i++) carray[i]=0x00;             
            }           
          } 
//End of terminal input code
          
    currentMillis = millis(); //Get current value of timer
    if (flag_high == 1) //Alarm interrupt occurred. Do not wait for polling loop to finish
    {
      trigger = min_interval; //Should be 0
    }
    else trigger = update_interval;
       
/* The following conditional code block retrieves data from the AquaPing using the I2C bus. This occurs at designated polling time; otherwise skip it and perform another polling iteration. */
    if (currentMillis - previousMillis >= trigger) 
    {
        digitalWrite(LED_BUILTIN, LOW); //Pulling this pin low turns on blue LED
        previousMillis = currentMillis;
        flag_high = 0;
        Wire.requestFrom(I2C_SLAVE, SBYTES);  
        i = Wire.available();
      if (i == 0) //Getting here would be unusual, but allow slave to reset just in case
      {
        Serial.println("No bytes available on I2C. Retry...");
        delay(700); //Wait for I2C bus to reset
        Wire.requestFrom(I2C_SLAVE, SBYTES);
        i = Wire.available();
        if (i == 0) Serial.print("Retry failed."); //Communication problem. Check I2C configuration
        else Serial.println("Retry successful.");      
      }
        //Get data from slave via I2C
        j=0;
        Serial.println("Status bytes retrieved from sensor:");
        while (Wire.available()) // slave may send less than requested bytes
        { 
          char c = Wire.read(); // receive a byte as character
          Status_array[j]=c;
          Serial.print( Status_array[j],DEC);  // print each byte
          Serial.print(" "); //space between bytes 
          j++;    
        }
        Serial.println();
        
//Display the continuously updated counting data on the Home Assistant GUI
        quiet_count->publish_state(Status_array[2]);
        leak_count->publish_state(Status_array[3]);
        noise_count->publish_state(Status_array[4]);
        
/* Display the fixed monitoring parameters as read from sensor on the HA GUI. This allows the user to confirm the desired configuration has been set correctly. */
        background_array_size->publish_state(Status_array[5]);
        event_array_size->publish_state(Status_array[6]);
        trigger_threshold->publish_state(Status_array[7]);
        polling_period->publish_state(Rate(Status_array[8]));
        
//Check for user inputs on the HA GUI. Update the sensor with I2C writes as necessary.
       char background_size = id(background_input).state;
       if(background_size != Status_array[5])
       {
            carray[0] = 0x62; 
            carray[1] = background_size; 
            Status_array[5] = background_size;
            send_byte_pairs();
       }
       
        char array_size = id(event_input).state;
        if(array_size != Status_array[6])
       {
            carray[0] = 0x65; 
            carray[1] = array_size; 
            Status_array[6] = array_size;
            send_byte_pairs();
       }
       
        char trigger_size = id(trigger_input).state;
        if(trigger_size != Status_array[7])
        {
            carray[0] = 0x74; 
            //Trigger size can't exceed array size; coerce it to array size
            if(trigger_size > Status_array[6]) trigger_size = Status_array[6];
            carray[1] = trigger_size; 
            Status_array[7] = trigger_size;
            send_byte_pairs();
        }

        char polling_index = id(polling_input).state;
        if(polling_index != Status_array[8])
        {
            carray[0] = 0x6F; 
            carray[1] = polling_index; 
            Status_array[8] = polling_index;
            send_byte_pairs();
        }
             
        bool led_state = id(led).state; //State of the LED on/off switch
       //Convert to a T/F byte
       if(led_state == true) led_byte = 0x01;
       else led_byte = 0x00;
       if(led_byte != Status_array[9])
       {
            carray[0] = 0x6C;
            Status_array[9] = led_byte;
            if (led_byte == 0x01) carray[1] = 0x31;
            else carray[1] = 0x30;
            send_byte_pairs();
        }
                    
        digitalWrite(LED_BUILTIN, HIGH);    //D1 mini LED off
        
         if ((Status_array[0] == 1) && (flag_low != 1))
        { 
            Serial.println("Alarm occurred."); 
        }
        if (flag_low == 1)
        { 
            Serial.println("Alarm cleared."); 
            flag_low = 0;
        }
    }
  } //End of polling loop

//Interrupts
static void IRAM_ATTR alarm_high()
{
    flag_high=1;
}
static void IRAM_ATTR alarm_low()
{
    flag_low=1;
}

void send_byte_pairs() //Send single commands on I2C
{
    Wire.beginTransmission(I2C_SLAVE); 
    for (i=0; i < 2; i++)  Wire.write(carray[i]);
    Wire.endTransmission(); 
}

/* This function parses and processes user input commands for the terminal; see user guide for syntax.
   Valid commands are lower case: r, a, p, b, t, e, o, l  */
int create_command(int k, char input_string[]) //Returns 1 if user entered a valid command string
{   
    int jj=0;
    int len=0; //Number of bytes in command string
    char c1,c2,c3,c4,arraysize;
    for(jj=0;jj<k;jj++)
    {
        c1 = input_string[jj++]; //First character must be lower case or space
        if (c1==0x20) ; //Do nothing if space encountered
        else if(islower(c1)) 
        {
        if (c1==0x72) //Reset; ASCII r
        {
          carray[0]=0x72;
          carray[1]=0x72;
          len=2;
          return 1;
        }
        else if (c1==0x6C) //LED status; ASCII l
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
        else if (c1==0x61) //Alarm status; ASCII a
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
        else if (c1==0x70) //Power/sleep status; ASCII p
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
        else if (c1==0x6F) //Polling period; ASCII o
        {
          carray[len++] = 0x6F;
          c1 = input_string[jj++];
          c2 = input_string[jj];
          //Only allowed arguments are single digits 1--9
          if (c1 >= 0x31 && c1 <= 0x39) carray[len++] = c1-48; //Convert ASCII to hex byte
          else return 0;
          if (c2 == 0x00) return 1;
          else if (c2 == 0x20) ;
          else return 0;        
        }
        else if (c1==0x62) //Background count; ASCII b
        {
          carray[len++] = 0x62;
          c1 = input_string[jj++];
          c2 = input_string[jj++];
          c3 = input_string[jj++];
          c4 = input_string[jj];
          if (isdigit(c1) && isdigit(c2) && !isdigit(c3)) //Checks for 10--99
          {
            char byte_array[] = {c1,c2};
            arraysize = atoi((char *)byte_array); 
            if (arraysize < 0x0A) return 0; //Return error if < 10
            carray[len++] = arraysize;
            jj=jj-1;
            if (c3==0x00) return 1; 
            else if(c3 == 0x20) ; 
            else return 0;    
          }
          else if (isdigit(c1) && isdigit(c2) && isdigit(c3) && !isdigit(c4))
          {
              char byte_array[] = {c1,c2,c3}; //Checks for 100-255
              arraysize = atoi((char *)byte_array); 
              //if (arraysize < 0x0A) return 0; 
              carray[len++] = arraysize;
              //Serial.println(arraysize,HEX); 
              if (c4==0x00) return 1; 
              else if(c4 == 0x20) ; 
              else return 0; 
          }       
        }
       else if (c1==0x65) //Array size; ASCII e
        {
          carray[len++] = 0x65;
          c1 = input_string[jj++];
          c2 = input_string[jj++];
          c3 = input_string[jj++];
          c4 = input_string[jj];
          if (isdigit(c1) && isdigit(c2) && !isdigit(c3)) //Checks for 10--99
          {
            char byte_array[] = {c1,c2};
            arraysize = atoi((char *)byte_array); 
            if (arraysize < 0x0A) return 0; //Return error if < 10
            carray[len++] = arraysize;
            jj=jj-1;
            if (c3==0x00) return 1; 
            else if(c3 == 0x20) ; 
            else return 0;    
          } 
          else if (isdigit(c1) && isdigit(c2) && isdigit(c3) && !isdigit(c4))
          {
              char byte_array[] = {c1,c2,c3}; //Checks for 100-255
              arraysize = atoi((char *)byte_array); 
              //if (arraysize < 0x0A) return 0; 
              carray[len++] = arraysize;
              //Serial.println(arraysize,HEX); 
               if (c4==0x00) return 1; 
              else if(c4 == 0x20) ; 
              else return 0; 
          }       
        } 
      else if (c1==0x74) //Trigger size; ASCII t
      {
          carray[len++] = 0x74;
          c1 = input_string[jj++];
          c2 = input_string[jj++];
          c3 = input_string[jj++];
          c4 = input_string[jj];
          if (isdigit(c1) && isdigit(c2) && !isdigit(c3)) //Checks for 10--99
          {
            char byte_array[] = {c1,c2};
            arraysize = atoi((char *)byte_array); 
            if (arraysize < 0x0A) return 0;
            carray[len++] = arraysize;
            jj = jj-1;
            if (c3==0x00) return 1; 
            else if(c3 == 0x20) ; 
            else return 0;    
          }
          else if (isdigit(c1) && isdigit(c2) && isdigit(c3) && !isdigit(c4)) //Checks for 100-255
            {
              char byte_array[] = {c1,c2,c3};
              arraysize = atoi((char *)byte_array); 
              if (arraysize < 0x0A) return 0; 
              carray[len++] = arraysize;
              if (c4==0x00) return 1; 
              else if(c4 == 0x20) ; 
              else return 0; 
            }       
      }
      else return 0;
    }    
  } 
  return 0;
} 

//Converts the polling rate byte into a decimal period (seconds) 
uint16_t Rate(uint8_t rindx) 
 {
      switch(rindx){
      case 0x01: return 1; //1 sec
                 break;
      case 0x02: return 2; //2 sec
                 break;
      case 0x03: return 3; //3 sec
                 break;
      case 0x04: return 5; //5 sec
                 break;
      case 0x05: return 10; //10 sec
                 break;
      case 0x06: return 15; //15 sec
                 break;
      case 0x07: return 20; //20 sec
                 break;
      case 0x08: return 25; //25 sec
                 break;
      case 0x09: return 30; //30 sec
                 break;
      default:   return 0;
                 break;
      }
 }  

};



