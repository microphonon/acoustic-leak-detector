/*  D1 mini firmware modified to work with ThingSpeak API.
    Make sure the ThingSpeak library is installed in the Arduino IDE.
    
    MPH MicroPhonon April 2022 */

  #include <Wire.h>
  #include <ESP8266WiFi.h>
  #include "ThingSpeak.h"
  //WiFi router access
  const char * ssid = "";   //Network SSID (name) 
  const char * password = "";   //Network password
  WiFiClient  client;
  //Modify the following as needed
  unsigned long myChannelNumber = 1;
  const char * myWriteAPIKey = "";
 //Enable the automatic light sleep on D1 mini
  extern "C"
  { 
    #include "user_interface.h" 
  } 
  //Pin setup will depend on specific device used; consult the datasheet
  #define SDA_PIN 4 //This is pin D2 on D1 mini
  #define SCL_PIN 5 //This is pin D1 on D1 mini
  #define INTERRUPT_PIN D3 //Connect D3 on D1 mini to ALARM on sensor
  
  #define SBYTES 12 //Number of status bytes
  #define ISTRING 24 //Max characters from terminal input
  #define CSTRING 16 //Max characters sent to slave on I2C

  const long polling_interval = 20000; 
  //Minimum update interval set by ThingSpeak is 15 seconds. Add 1 sec for latency
  const long min_interval = 16000; 
  const long sleep_delay = 200; //Kill off some time in light sleep mode without adding much latency
//  const long min_interval = 0; //Delay to escape polling loop and notify the network when alarm occurs; should be 0
  const uint16_t I2C_SLAVE = 0x77; //AquaPing slave address
  const uint16_t baud_rate = 9600; //UART baud rate
  
  int i, j, error_code, k=0, bsize=0;
  volatile uint8_t flag_high = 0; 
  volatile uint8_t flag_low = 0; 
  volatile uint8_t first_pass = 1; 
  char Status_array[SBYTES];
  char input_string[ISTRING],carray[CSTRING];
  unsigned long currentMillis, trigger; 

  void setup() 
  {    
    WiFi.mode(WIFI_STA);     
    ThingSpeak.begin(client);  // Initialize ThingSpeak
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(baud_rate);  //Start serial UART for terminal output via USB
    Wire.begin(SDA_PIN, SCL_PIN); //I2C bus
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), alarm_handler, CHANGE);
    for(i=0;i<SBYTES;i++)Status_array[i]=0; //Clear status array
  }

  unsigned long previousMillis = millis();
 
  void loop() 
  {
        //Flush the serial port on first pass
        if(first_pass == 1)
        {
          while (Serial.available() > 0) Serial.read(); 
          first_pass = 0;
        }
       //Check for user input from the terminal
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
          } //End of terminal input code
          
    currentMillis = millis(); //Get current value of timer
    if (flag_high == 1) //Alarm interrupt occurred. Do not wait for polling loop to finish
    {
      trigger = min_interval; //Should be 0
    }
    else trigger = polling_interval;
    
    /* The following conditional code block retrieves data from sensor.
     This occurs at designated polling time; otherwise skip it and 
     perform another polling iteration. */
    if (currentMillis - previousMillis >= trigger) 
    {
      previousMillis = currentMillis;
      flag_high = 0;
    // Connect or reconnect to WiFi if necessary
      if(WiFi.status() != WL_CONNECTED)
      {
          Serial.print("Attempting to connect to WiFi.");
          while(WiFi.status() != WL_CONNECTED)
          {
            WiFi.begin(ssid, password); 
            delay(5000);  
            previousMillis += 5000;
            Serial.print(".");   
          } 
          Serial.println("WiFi connected.");
      }
      digitalWrite(LED_BUILTIN, LOW); //Pulling this pin low turns on LED
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
      digitalWrite(LED_BUILTIN, HIGH);    //LED off

      if ((Status_array[0] == 1) && (flag_low != 1))
      { 
        Serial.println("Alarm occurred."); 
      }
       if (flag_low == 1)
      { 
        Serial.println("Alarm cleared."); 
        flag_low = 0;
      }
      int poll = Rate(Status_array[8]);
      
      ThingSpeak.setField(1, Status_array[0]); //Leak alert
      ThingSpeak.setField(2, Status_array[1]); //Noise alert
      ThingSpeak.setField(3, Status_array[2]); //Quiet count
      ThingSpeak.setField(4, Status_array[3]); //Leak count
      ThingSpeak.setField(5, Status_array[4]); //Noise count
      ThingSpeak.setField(6, Status_array[5]); //Background array size
      ThingSpeak.setField(7, Status_array[6]); //Event array size
      ThingSpeak.setField(8, Status_array[7]); //Trigger array size
      ThingSpeak.setField(9, poll); //Polling period
      ThingSpeak.setField(10, Status_array[9]); //LED status
      ThingSpeak.setField(11, Status_array[10]); //Sleep status
      ThingSpeak.setField(12, Status_array[11]); //Firmware version
     
      error_code = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      if(error_code == 200) Serial.println("Channel data sent to server.");
      else  Serial.println("Problem connecting with ThingSpeak channel. HTTP error code " + String(error_code));
    } 
     delay(sleep_delay); 
} //End of main polling loop

IRAM_ATTR void alarm_handler() //No delays allowed inside interrupt
    {
      if(digitalRead(INTERRUPT_PIN) == HIGH) flag_high=1;
      else flag_low = 1;
    }

//This function parses and processes user input commands; see user guide for syntax
int create_command(int k, char input_string[]) //Returns 1 if user entered a valid command string
//Valid commands are lower case: r, a, p, b, t, e, o, l
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



  
