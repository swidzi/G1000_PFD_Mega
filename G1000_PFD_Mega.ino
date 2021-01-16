#include <Rotary.h> //Rotary encoder library
#include <Adafruit_MCP23017.h>  //MCP23017 library
#include <CommonBusEncoders.h>  //Rotary Encoders on bus
#include <Wire.h> // Generic I2C control library
#include <avr/pgmspace.h> // Library allowing for storing constants in progmem
#include <Bounce2.h> //Library for using a debounced switch


// Declarations:
const byte IsAPPanelActive = 1; //If this constant is 1 the AP keyboard is on and IC3 is populated and I/O is set for Input with pull_up resistor on. Else IC3 is unpopulated and not connected via I2C

//Pins:
//Encoders

//Common bus
const int EncBusAPin = 17;
const int EncBusBPin = 18;
const int EncBusSwPin = A2;
const int Enc_VOL_LEFT_Pin = 23;
const int Enc_NAV1_Pin = 25;
const int Enc_NAV2_Pin = 27;
const int Enc_HDG_Pin = 29;
const int Enc_ALT1_Pin = 31;
const int Enc_ALT2_Pin = 33;
const int Enc_VOL_RIGHT_Pin = 35;
const int Enc_COM1_Pin = 37;
const int Enc_COM2_Pin = 39;
const int Enc_CRS1_Pin = 41;
const int Enc_CRS2_Pin = 43;
const int Enc_FMS1_Pin = 45;
const int Enc_FMS2_Pin = 47;

//Backlight
const int Backlight_Pin = A5;

//Joystick encoder
const int Range_Rotary_A = A6;
const int Range_Rotary_B = A7;

//Debounced switch for backlight transistor
const int Backlight_Sw_Pin = A8; 

//MCP Addresses:
const uint8_t SoftKeyMCP_Addr = 0;
const uint8_t APMCP_Addr = 1;
const uint8_t FMSMCP_Addr = 2;



//Object declarations:
//Common bus encoders
//Create new Encoder bank (all 14)
CommonBusEncoders encoders(EncBusAPin, EncBusBPin, EncBusSwPin, 14);  // 14 = number of encoders

//Standalone range encoder
Rotary Range_Enc(Range_Rotary_A, Range_Rotary_B);

//Switch for Backlight
Bounce Backlight = Bounce();


//MCP23017 declaraions
Adafruit_MCP23017 SoftKeyMCP;
Adafruit_MCP23017 APMCP;
Adafruit_MCP23017 FMSMCP;

//MCP23017 Debouncers:
long time = 0;         // the last time the output pin was sampled (has to be long 
const int debounce_count = 10; // number of millis/samples to consider before declaring a debounced input
const int mcp_active_pins = 16;  //number of pins (switches) connected to MCP23017. All are considered connected. Disconnected are handled in code, as switches are not in sequence

//SoftKeyMCP
int counterSoft[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  };      // how many times we have seen new value - each for every pin
int readingSoft[16];           // the current value read from the input pin - each for every pin
int current_stateSoft[16] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};    // the debounced input value - each for every pin

//APMCP
int counterAP[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  };      // how many times we have seen new value - each for every pin
int readingAP[16];           // the current value read from the input pin - each for every pin
int current_stateAP[16] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};    // the debounced input value - each for every pin

//FMSMCP
int counterFMS[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  };      // how many times we have seen new value - each for every pin
int readingFMS[16];           // the current value read from the input pin - each for every pin
int current_stateFMS[16] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};    // the debounced input value - each for every pin


//Arrays storing all possible G1000 messages in a char* array (array of strings). Array has to be stored in progmem to reduce the RAM footprint
const char* Button_Commands[] = { 
  "BTN_NAV_FF",
  "",
  "BTN_SOFT_7",
  "BTN_SOFT_8",
  "BTN_SOFT_9",
  "BTN_SOFT_10",
  "BTN_SOFT_11",
  "BTN_SOFT_12",
  "BTN_SOFT_6",
  "BTN_SOFT_5",
  "BTN_SOFT_4",
  "BTN_SOFT_3",
  "BTN_SOFT_2",
  "BTN_SOFT_1",
  "",
  "BTN_COM_FF",
  "",
  "",
  "BTN_FLC",
  "BTN_VS",
  "BTN_APR",
  "BTN_NAV",
  "BTN_HDG",
  "BTN_AP",
  "BTN_ALT",
  "BTN_VNAV",
  "BTN_BC",
  "BTN_NOSE_UP",
  "BTN_NOSE_DN",
  "BTN_FD",
  "",
  "",
  "",
  "",
  "",
  "",
  "BTN_MENU",
  "BTN_DIRECT",
  "BTN_ENT",
  "BTN_PROC",
  "BTN_FPL",
  "BTN_CLR",
  "",
  "BTN_PAN_UP",
  "BTN_PAN_SYNC",
  "BTN_PAN_RIGHT",
  "BTN_PAN_LEFT",
  "BTN_PAN_DN",
};

const char* Encoder_Commands_Inc[] = { 
"ENC_NAV_VOL_UP",
"ENC_NAV_OUTER_UP",
"ENC_NAV_INNER_UP",
"ENC_HDG_UP",
"ENC_ALT_OUTER_UP",
"ENC_ALT_INNER_UP",
"ENC_COM_VOL_UP",
"ENC_COM_OUTER_UP",
"ENC_COM_INNER_UP",
"ENC_BARO_UP",
"ENC_CRS_UP",
"ENC_FMS_INNER_UP",
"ENC_FMS_OUTER_UP",
"ENC_RANGE_UP"

};

const char* Encoder_Commands_Dec[] = { 
"ENC_NAV_VOL_DN",
"ENC_NAV_OUTER_DN",
"ENC_NAV_INNER_DN",
"ENC_HDG_DN",
"ENC_ALT_OUTER_DN",
"ENC_ALT_INNER_DN",
"ENC_COM_VOL_DN",
"ENC_COM_OUTER_DN",
"ENC_COM_INNER_DN",
"ENC_BARO_DN",
"ENC_CRS_DN",
"ENC_FMS_INNER_DN",
"ENC_FMS_OUTER_DN",
"ENC_RANGE_DN"

};

const char* Encoder_Commands_Sw[] = { 
"BTN_NAV_VOL",
"BTN_NAV_TOG",
"",
"BTN_HDG_SYNC",
"BTN_ALT_SYNC",
"",
"BTN_COM_VOL",
"BTN_COM_TOG",
"",
"BTN_CRS_SYNC",
"",
"BTN_FMS",
"",

};




void setup() {

  //start-up the I2C bus
  Wire.begin();

  //Set-up encoders:
  encoders.setDebounce(16); //set debounce - medium
  encoders.resetChronoAfter(250); //milliseconds between switching encoders

  //Create encoder objects 
    int Encoder_Pin_Numbers[] = { 
      Enc_VOL_LEFT_Pin,
      Enc_NAV1_Pin,
      Enc_NAV2_Pin,
      Enc_HDG_Pin,
      Enc_ALT1_Pin,
      Enc_ALT2_Pin,
      Enc_VOL_RIGHT_Pin,
      Enc_COM1_Pin,
      Enc_COM2_Pin,
      Enc_CRS1_Pin,
      Enc_CRS2_Pin,
      Enc_FMS1_Pin,
      Enc_FMS2_Pin
     } ;
  for (int i = 0;i == 13;i++) {
      encoders.addEncoder(i + 1, 4, Encoder_Pin_Numbers[i], 1, (i * 100) + 100, (i* 100) + 150); // Each encoder has a code equal 100 multiplied by encoder number.
    
  }

  // Initialize Bounce button for Backlight
  // Set pin mode
  pinMode(Backlight_Sw_Pin, INPUT_PULLUP);
  pinMode(Backlight_Pin, OUTPUT);
  //set transistor off
  digitalWrite(Backlight_Pin, LOW);
  Backlight.attach(Backlight_Sw_Pin);
  Backlight.interval(25);
  
  

  // Initialize MCP23017
  SoftKeyMCP.begin(SoftKeyMCP_Addr);
  if(IsAPPanelActive == 1) { APMCP.begin(APMCP_Addr) ;}; //If AP panel is inactive, do not initialize MCP
  FMSMCP.begin(FMSMCP_Addr);

  //set pinMode and pullups for MCP. Set unused pins for output to reduce noise
  //SoftKeyMCP:
  SoftKeyMCP.pinMode(8, INPUT);
  SoftKeyMCP.pullUp(8, HIGH);
  SoftKeyMCP.pinMode(9, INPUT);
  SoftKeyMCP.pullUp(9, HIGH);
  SoftKeyMCP.pinMode(10, INPUT);
  SoftKeyMCP.pullUp(10, HIGH);
  SoftKeyMCP.pinMode(11, INPUT);
  SoftKeyMCP.pullUp(11, HIGH);
  SoftKeyMCP.pinMode(12, INPUT);
  SoftKeyMCP.pullUp(12, HIGH);
  SoftKeyMCP.pinMode(13, INPUT);
  SoftKeyMCP.pullUp(13, HIGH);
  SoftKeyMCP.pinMode(14, OUTPUT);
  SoftKeyMCP.pullUp(14, LOW);
  SoftKeyMCP.pinMode(15, INPUT);
  SoftKeyMCP.pullUp(15, HIGH);
  SoftKeyMCP.pinMode(0, INPUT);
  SoftKeyMCP.pullUp(0, HIGH);
  SoftKeyMCP.pinMode(1, OUTPUT);
  SoftKeyMCP.pullUp(1, LOW);
  SoftKeyMCP.pinMode(2, INPUT);
  SoftKeyMCP.pullUp(2, HIGH);
  SoftKeyMCP.pinMode(3, INPUT);
  SoftKeyMCP.pullUp(3, HIGH);  
  SoftKeyMCP.pinMode(4, INPUT);
  SoftKeyMCP.pullUp(4, HIGH);
  SoftKeyMCP.pinMode(5, INPUT);
  SoftKeyMCP.pullUp(5, HIGH);
  SoftKeyMCP.pinMode(6, INPUT);
  SoftKeyMCP.pullUp(6, HIGH);
  SoftKeyMCP.pinMode(7, INPUT);
  SoftKeyMCP.pullUp(7, HIGH);

  // APMCP
  if (IsAPPanelActive == 1) {
    APMCP.pinMode(8, INPUT);
    APMCP.pullUp(8, HIGH);
    APMCP.pinMode(9, INPUT);
    APMCP.pullUp(9, HIGH);
    APMCP.pinMode(10, INPUT);
    APMCP.pullUp(10, HIGH);
    APMCP.pinMode(11, INPUT);
    APMCP.pullUp(11, HIGH);
    APMCP.pinMode(12, INPUT);
    APMCP.pullUp(12, HIGH);
    APMCP.pinMode(13, INPUT);
    APMCP.pullUp(13, HIGH);
    APMCP.pinMode(14, OUTPUT);
    APMCP.pullUp(14, LOW);
    APMCP.pinMode(15, OUTPUT);
    APMCP.pullUp(15, LOW);
    APMCP.pinMode(0, OUTPUT);
    APMCP.pullUp(0, LOW);
    APMCP.pinMode(1, OUTPUT);
    APMCP.pullUp(1, LOW);
    APMCP.pinMode(2, INPUT);
    APMCP.pullUp(2, HIGH);
    APMCP.pinMode(3, INPUT);
    APMCP.pullUp(3, HIGH);  
    APMCP.pinMode(4, INPUT);
    APMCP.pullUp(4, HIGH);
    APMCP.pinMode(5, INPUT);
    APMCP.pullUp(5, HIGH);
    APMCP.pinMode(6, INPUT);
    APMCP.pullUp(6, HIGH);
    APMCP.pinMode(7, INPUT);
    APMCP.pullUp(7, HIGH);

  }

  //FMSMCP
  FMSMCP.pinMode(8, INPUT);
  FMSMCP.pullUp(8, HIGH);
  FMSMCP.pinMode(9, INPUT);
  FMSMCP.pullUp(9, HIGH);
  FMSMCP.pinMode(10, OUTPUT);
  FMSMCP.pullUp(10, LOW);
  FMSMCP.pinMode(11, INPUT);
  FMSMCP.pullUp(11, HIGH);
  FMSMCP.pinMode(12, INPUT);
  FMSMCP.pullUp(12, HIGH);
  FMSMCP.pinMode(13, INPUT);
  FMSMCP.pullUp(13, HIGH);
  FMSMCP.pinMode(14, INPUT);
  FMSMCP.pullUp(14, HIGH);
  FMSMCP.pinMode(15, INPUT);
  FMSMCP.pullUp(15, HIGH);
  FMSMCP.pinMode(0, OUTPUT);
  FMSMCP.pullUp(0, LOW);
  FMSMCP.pinMode(1, OUTPUT);
  FMSMCP.pullUp(1, LOW);
  FMSMCP.pinMode(2, OUTPUT);
  FMSMCP.pullUp(2, LOW);
  FMSMCP.pinMode(3, OUTPUT);
  FMSMCP.pullUp(3, LOW);  
  FMSMCP.pinMode(4, INPUT);
  FMSMCP.pullUp(4, HIGH);
  FMSMCP.pinMode(5, INPUT);
  FMSMCP.pullUp(5, HIGH);
  FMSMCP.pinMode(6, INPUT);
  FMSMCP.pullUp(6, HIGH);
  FMSMCP.pinMode(7, INPUT);
  FMSMCP.pullUp(7, HIGH);


  // start serial connection
  Serial.begin(115200); while (!Serial);





}

void SEND_SERIAL(int Calling, int Message) {
// procedure to send proper message. 
// Thinking of 2 incoming variables - calling function (1 - MCP ON, 2 - MCP OFF, 3 - CommonBus, 4 - Range Enc) and message in int form
// For MCP the pin number with added 0 for SoftKey, 16 for AP and 32 for FMS.
// For encoder the encoder code (100-1450)

switch (Calling) {
  case 1:
    //Lookup the proper message from array
    Serial.write(Button_Commands[Message]);
    //send ON signal and an newline symbol
    Serial.write("=1\n");
  break;
  case 2:
    //Lookup the proper message from array
    Serial.write(Button_Commands[Message]);
    //send OFF signal and an newline symbol
    Serial.write("=0\n");
  break;
  case 3:
    //Find type of message - Inc = 0, Dec = 1, Sw = 50
    int Type = Message % 100;
    //Find encoder
    int Encod = Message / 100;
    //Lookup the proper
    switch (Type) {
      case 0: //Inc
    //Lookup the proper message from array
    Serial.write(Encoder_Commands_Inc[Message]);
    //send newline symbol
    Serial.write("\n");        
      break;
      case 1: //Dec
    //Lookup the proper message from array
    Serial.write(Encoder_Commands_Dec[Message]);
    //send newline symbol
    Serial.write("\n");      
      break; //Sw
      case 50:
    //Lookup the proper message from array
    Serial.write(Encoder_Commands_Sw[Message]);
    //send newline symbol
    Serial.write("\n");    
      break;
    }
  break;
  case 4:
  // send messages from encoder
  //no sure if I understand correctly, by DIR_CW == 0x10 and DIR_CCW == 0x20. I need to test it
  if (Message == 0x10) {
    Serial.write("ENC_RANGE_UP\n"); //If rotating right Range goes up
  }
  if (Message == 0x20) {
    Serial.write("ENC_RANGE_DN\n"); //If rotating left range goes down
  }
 
  break;
}

}




void PROCESS_ENCODERS() {
  // Proces common bus encoders
  int EncRead = encoders.readAll();
  if (EncRead != 0) {
    SEND_SERIAL(3, EncRead);
  }
  // Proces stand alone encoder
  int Result = Range_Enc.process();
  if(Result != 0 ) {
    SEND_SERIAL(4, Result);
  }
}


void MCP23017_DEBOUNCER() {



  //if(millis() != time) //Reduce number of comparissons to once per millisecond
  if(micros() % 1000 == 0 ) //Do a comparison once per millisecond (every 1000 microseconds)
  {
    //Check the SoftKeyMCP
    for(int i = 0; i < mcp_active_pins ; i++) //iterate via all pin ports
    {
      if(i == 1 || i == 14) { //For SoftKey MCP if pin is 1 or 14 do not read pin, there is no button attached
        continue;
      }
      
      readingSoft[i] = SoftKeyMCP.digitalRead(i); //read port
  
      if(readingSoft[i] == current_stateSoft[i] && counterSoft[i] > 0) //if value has "bounced" reduce count number
      {
        counterSoft[i]--;
      }
      if(readingSoft[i] != current_stateSoft[i]) //if value is "held" add counter
      {
         counterSoft[i]++; 
      }
      // If the Input has shown the same value for long enough let's switch it
      if(counterSoft[i] >= debounce_count)
      {
        counterSoft[i] = 0;  //clear counter
        
        //Execute proper action here. RealSimGear requires action both on HIGH and LOW.
        SEND_SERIAL(1, i);     
      }
      current_stateSoft[i] = readingSoft[i]; //update current state
    }

    //Check the APMCP, if it is activated
    if(IsAPPanelActive == 1) {
      for(int i = 0; i < mcp_active_pins ; i++) //iterate via all pin ports
      {
        if(i == 0 || i == 1 || i == 14 || i == 15) { //For AP MCP if pin is 0, 1, 14 or 15 do not read pin, there is no button attached
          continue;
        }
        
        readingSoft[i] = APMCP.digitalRead(i); //read port
    
        if(readingAP[i] == current_stateAP[i] && counterAP[i] > 0) //if value has "bounced" reduce count number
        {
          counterAP[i]--;
        }
        if(readingAP[i] != current_stateAP[i]) //if value is "held" add counter
        {
           counterAP[i]++; 
        }
        // If the Input has shown the same value for long enough let's switch it
        if(counterAP[i] >= debounce_count)
        {
          counterAP[i] = 0;  //clear counter
          
          //Execute proper action here. RealSimGear requires action both on HIGH and LOW.
        SEND_SERIAL(1, i + 16);
        }
        current_stateAP[i] = readingAP[i]; //update current state
      }
    }
        //Check the FMSMCP
    for(int i = 0; i < mcp_active_pins ; i++) //iterate via all pin ports
    {
      if(i == 0 || i == 1 || i == 2 || i == 3 || i == 10) { //For FMS MCP if pin is 0 to 3 or 10 do not read pin, there is no button attached
        continue;
      }
      
      readingFMS[i] = FMSMCP.digitalRead(i); //read port
  
      if(readingFMS[i] == current_stateFMS[i] && counterFMS[i] > 0) //if value has "bounced" reduce count number
      {
        counterFMS[i]--;
      }
      if(readingFMS[i] != current_stateFMS[i]) //if value is "held" add counter
      {
         counterFMS[i]++; 
      }
      // If the Input has shown the same value for long enough let's switch it
      if(counterFMS[i] >= debounce_count)
      {
        counterFMS[i] = 0;  //clear counter
        
        //Execute proper action here. RealSimGear requires action both on HIGH and LOW.
        SEND_SERIAL(1, i + 32);
      }
      current_stateFMS[i] = readingFMS[i]; //update current state
    }
  
    //time = millis(); //update timer
  }
   
}
  



void loop() {

  // Code snippet from hktony - Thanks!
  // keep alive for RSG connection
  if(millis() % 1000 == 0)
    Serial.write("\\####RealSimGear#RealSimGear-G1000XFD#1#3.1.9#656E6B776FA39/\n"); // 3.1.9 = latest firmware; 756E6B776Fd39 = RANDOM ID
  
  PROCESS_ENCODERS();
  MCP23017_DEBOUNCER();

  // Update backlight switch status
  Backlight.update();
  if (Backlight.fell()) {
    //if backlight switch is 0, switch off the transistor (backlight)
    digitalWrite(Backlight_Pin, LOW);
  }
  if (Backlight.rose()) {
    //if backlight switch is 1, switch on the transistor (backlight)
    digitalWrite(Backlight_Pin, HIGH);    
  }
  
}
