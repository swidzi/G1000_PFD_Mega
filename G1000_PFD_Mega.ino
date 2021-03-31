/*
    PFD/MFD Arduino Mega Firmware using RealSimGear Flight Simmulator Plugin
    Copyright (C) 2021  Wojciech Swidzinski

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// Libraries
#include <Adafruit_MCP23017.h>  //MCP23017 library
#include <CommonBusEncoders.h>  //Rotary Encoders on bus
#include <Wire.h> // Generic I2C control library
#include <Bounce2.h> //Library for using a debounced switch
#include <Encoder.h> //Rotary Encoder library

// Panel settings:
const byte IsAPPanelActive = 1; //If this constant is 1 the AP keyboard is on and IC3 is populated and I/O is set for Input with pull_up resistor on. Else IC3 is unpopulated and not connected via I2C
const int MCP_Update_Time = 50; // Check the MCPs every 50ms
const int SYNC_Send_Time = 100;
const int RSG_Keep_Alive = 1000; // Send keep alive message every second
// Are encoders flipped (Inside with outside):
bool Is_NAV_flipped = true;
bool Is_ALT_flipped = false;
bool Is_COM_flipped = false;
bool Is_CRS_flipped = false;
bool Is_FMS_flipped = false;

// Is clockwise turn (even) increasing value. (Encoders are in order of the commands array. If you flipped encoders remember to "flip" them here as well).
const bool Is_Clockwise_Inc[] = {
  true, true, true, true,
  true, true, true, true,
  true, true, true, true,
  true, true, true, true
};

// Declarations:
//MCP Addresses:
const uint8_t SoftKeyMCP_Addr = 0;
const uint8_t APMCP_Addr = 1;
const uint8_t FMSMCP_Addr = 2;

//Pins:
//Common bus encoders
const int EncBusAPin = 17;
const int EncBusBPin = 18;
const int EncBusSwPin = 19;
const int Enc_VOL_LEFT_Pin = 23;
const int Enc_NAV_Out_Pin = 25;
const int Enc_NAV_In_Pin = 27;
const int Enc_HDG_Pin = 29;
const int Enc_ALT_Out_Pin = 31;
const int Enc_ALT_In_Pin = 33;
const int Enc_VOL_RIGHT_Pin = 35;
const int Enc_COM_Out_Pin = 37;
const int Enc_COM_In_Pin = 39;
const int Enc_CRS_Out_Pin = 41;
const int Enc_CRS_In_Pin = 43;
const int Enc_FMS_Out_Pin = 45;
const int Enc_FMS_In_Pin = 47;

//Joystick encoder
const int Range_Rotary_A = A6;
const int Range_Rotary_B = A7;

//Backlight
const int Backlight_Pin = A5;

//Debounced switch for backlight transistor
const int Backlight_Sw_Pin = A8;


//Object declarations:
//Common bus encoders
//Create new Encoder bank (all 13)
CommonBusEncoders encoders(EncBusAPin, EncBusBPin, EncBusSwPin, 13);  // 13 = number of encoders

//Standalone range encoder
Encoder myEnc(Range_Rotary_A, Range_Rotary_B);

//Switch for Backlight
Bounce Backlight = Bounce();

//MCP23017 declaraions
Adafruit_MCP23017 SoftKeyMCP;
Adafruit_MCP23017 APMCP;
Adafruit_MCP23017 FMSMCP;


//Variable declarations:
//SoftkeyMCP state storing
byte OldValuesSoft[] = { 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0
                       };
byte NewValuesSoft[] = { 0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0
                       };
//Autopilot MCP state storing
byte OldValuesAP[] = { 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0
                     };
byte NewValuesAP[] = { 0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0, 0, 0,
                       0, 0, 0, 0
                     };
//FMS MCP state storing
byte OldValuesFMS[] = { 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0
                      };
byte NewValuesFMS[] = { 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0
                      };


long Last_Keep_Alive = 0; // to store last keep alive time
long SYNC_Triggered_Time = 0;  //used to store last time SYNC button was triggered
bool Send_SYNC = false; // flag to send SYNC
int oldPosition = 0; // variable to store last position of the encoder. Used to determine CW or CCW rotation.

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
  "BTN_PAN_SYNC",
  "BTN_PAN_UP",
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
  "",
  "BTN_NAV_TOG",
  "BTN_HDG_SYNC",
  "",
  "BTN_ALT_SYNC",
  "BTN_COM_VOL",
  "",
  "BTN_COM_TOG",
  "",
  "BTN_CRS_SYNC",
  "",
  "BTN_FMS",

};

void setup() {
  // start serial connection
  Serial.begin(115200); while (!Serial);
  //start-up the I2C bus
  Wire.begin();

  //Set-up encoders:
  encoders.setDebounce(32); //set debounce - high
  encoders.resetChronoAfter(250); //milliseconds between switching encoders

  //Create encoder objects
  int Encoder_Pin_Numbers[] = {
    Enc_VOL_LEFT_Pin,
    Enc_NAV_Out_Pin,
    Enc_NAV_In_Pin,
    Enc_HDG_Pin,
    Enc_ALT_Out_Pin,
    Enc_ALT_In_Pin,
    Enc_VOL_RIGHT_Pin,
    Enc_COM_Out_Pin,
    Enc_COM_In_Pin,
    Enc_CRS_Out_Pin,
    Enc_CRS_In_Pin,
    Enc_FMS_Out_Pin,
    Enc_FMS_In_Pin
  } ;
  // Make sure that order of encoders matches the encoder type
  if (Is_NAV_flipped == true) {

    Encoder_Pin_Numbers[1] = Enc_NAV_In_Pin;
    Encoder_Pin_Numbers[2] = Enc_NAV_Out_Pin;
  }

  if (Is_ALT_flipped == true) {
    Encoder_Pin_Numbers[4] = Enc_ALT_In_Pin;
    Encoder_Pin_Numbers[5] = Enc_ALT_Out_Pin;

  }
  if (Is_COM_flipped == true) {
    Encoder_Pin_Numbers[7] = Enc_COM_In_Pin;
    Encoder_Pin_Numbers[8] = Enc_COM_Out_Pin;
  }
  if (Is_CRS_flipped == true) {
    Encoder_Pin_Numbers[9] = Enc_CRS_In_Pin;
    Encoder_Pin_Numbers[10] = Enc_CRS_Out_Pin;

  }

  if (Is_FMS_flipped == true) {
    Encoder_Pin_Numbers[11] = Enc_FMS_In_Pin;
    Encoder_Pin_Numbers[12] = Enc_FMS_Out_Pin;

  }
  // Create encoder objects
  for (int i = 0; i < 13; i++) {
    encoders.addEncoder(i + 1, 4, Encoder_Pin_Numbers[i], 1, ((i * 100) + 100), ((i * 100) + 150)); // Each encoder has a code equal 100 multiplied by encoder number.
  }

  // Initialize Bounce button for Backlight
  // Set pin mode
  // NOT TESTED:
  pinMode(Backlight_Sw_Pin, INPUT_PULLUP);
  pinMode(Backlight_Pin, OUTPUT);
  pinMode(Range_Rotary_B, INPUT_PULLUP);
  pinMode(Range_Rotary_A, INPUT_PULLUP);


  //set transistor off
  //NOT TESTED:
  digitalWrite(Backlight_Pin, LOW);
  Backlight.attach(Backlight_Sw_Pin);
  Backlight.interval(25);



  // Initialize MCP23017
  SoftKeyMCP.begin(SoftKeyMCP_Addr);
  FMSMCP.begin(FMSMCP_Addr);
  if (IsAPPanelActive == 1) {
    APMCP.begin(APMCP_Addr) ;
  }; //If AP panel is inactive, do not initialize MCP

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

}

void SEND_SERIAL(int Calling, int Message) {
  // procedure to send proper message to RSG Plugin.
  // 2 incoming variables - calling function (1 - MCP ON, 2 - MCP OFF, 3 - CommonBus, 4 - Range Enc) and message in int form
  // For MCP the pin number with added 0 for SoftKey, 16 for AP and 32 for FMS.
  // For Common busencoder the encoder code (100-1450)
  // For Normal encoder direction of rotation (-1 or 1)

  switch (Calling) {
    case 1: //Push switch
      {
        //Lookup the proper message from array
        Serial.write(Button_Commands[Message]);
        //send ON signal and an newline symbol
        Serial.write("=1\n");
      }
      break;

    case 2: //Release switch
      {
        //Lookup the proper message from array
        Serial.write(Button_Commands[Message]);
        //send OFF signal and an newline symbol
        Serial.write("=0\n");
      }
      break;
    case 3: //Send rotary encoder
      {
        //Find type of message - Inc = 0, Dec = 1, Switch = 50
        int Type = Message % 100;
        //Find which encoder was clicked
        int Encod = Message / 100;
        Encod = Encod - 1; // Messages are 0 indexed.
        // Check if encoder is CW for Inc
        if ( (Is_Clockwise_Inc[Encod] == false ) && (Type == 1)) {
          Type = 0; // if it is CCW for Inc flip it
        }
        else if  ( (Is_Clockwise_Inc[Encod] == false ) && (Type == 0))
        {
          Type = 1; // if it is CCW for Inc flip it
        }
        switch (Type) {
          case 0: //Inc
            //Lookup the proper message from array
            Serial.write(Encoder_Commands_Inc[Encod]);
            //send newline symbol
            Serial.write("\n");
            break;
          case 1: //Dec
            //Lookup the proper message from array
            Serial.write(Encoder_Commands_Dec[Encod]);
            //send newline symbol
            Serial.write("\n");
            break; //Sw
          case 50:
            //Lookup the proper message from array
            Serial.write(Encoder_Commands_Sw[Encod]);
            //send newline symbol
            Serial.write("\n");
            break;
        }
      }
      break;
    case 4:
      {
        // send messages from encoder
        if (Message < 0) {
          Serial.write("ENC_RANGE_UP\n"); //If rotating right Range goes up
        }
        if (Message > 0) {
          Serial.write("ENC_RANGE_DN\n"); //If rotating left range goes down
        }
      }
      break;
  }
}

void PROCESS_ENCODERS() {
  // Proces common bus encoders
  int EncRead = encoders.readAll(); //Get updated state

  if (EncRead > 0) { //If any encoder clicked, send to PC
    SEND_SERIAL(3, EncRead);
  }

  // Proces stand alone encoder
  int newPosition = myEnc.read(); //Get the new position of the encoder
  if (newPosition != oldPosition) { //if it changed do something with it.
    SEND_SERIAL(4, newPosition); // Send to Plugin
    newPosition = newPosition + (newPosition * -1); // set the newPosition back to 0
    myEnc.write(newPosition); // and write it back to encoder library
    oldPosition = newPosition; // also make sure that old postion is updated
  }
}

void BUTTONS() {
  // SoftKey MCP
  for (int i = 0; i < 16 ; i++) //iterate via all pin ports
  {
    if (i == 1 || i == 14) { //For SoftKey MCP if pin is 1 or 14 do not read pin, there is no button attached
      continue; //Skip to next for
    }
    NewValuesSoft[i] = SoftKeyMCP.digitalRead(i); //Read the pin
    if (NewValuesSoft[i] != OldValuesSoft[i]) { // On change of pin
      OldValuesSoft[i] = NewValuesSoft[i]; // Update pin list
      if (NewValuesSoft[i] == 1) {
        SEND_SERIAL(2, i); //on change to high, release button
      }
      if (NewValuesSoft[i] == 0) {
        SEND_SERIAL(1, i); //on change to low press button
      }
    }
  }
  // AP MCP
  if (IsAPPanelActive == 1) { // Only check for AP MCP if the AP panel is activated
    for (int i = 0; i < 16 ; i++) //iterate via all pin ports
    {
      if (i == 0 || i == 1 || i == 14 || i == 15) { //For AP MCP if pin is 0, 1, 14 or 15 do not read pin, there is no button attached
        continue; //Skip to next for
      }
      NewValuesAP[i] = APMCP.digitalRead(i); //Read the pin
      if (NewValuesAP[i] != OldValuesAP[i]) { // On change of pin
        OldValuesAP[i] = NewValuesAP[i]; // Update pin list
        if (NewValuesAP[i] == 1) {
          SEND_SERIAL(2, i + 16); //on change to high, release button. Add 16 to find proper message.
        }
        if (NewValuesAP[i] == 0) {
          SEND_SERIAL(1, i + 16); //on change to low press button. Add 16 to find proper message.
        }
      }
    }
  }
  //FMS MCP
  for (int i = 0; i < 16 ; i++) //iterate via all pin ports
  {
    if (i == 0 || i == 1 || i == 2 || i == 3 || i == 10) { //For FMS MCP if pin is 0 to 3 or 10 do not read pin, there is no button attached
      continue; //Skip to next for
    }
    NewValuesFMS[i] = FMSMCP.digitalRead(i); //Read the pin
    if (NewValuesFMS[i] != OldValuesFMS[i]) { // On change of pin
      OldValuesFMS[i] = NewValuesFMS[i]; // Update pin list
      if (i == 11) {       //Check if this is BTN_PAN_SYNC pin
        SYNC_Triggered_Time = millis(); //mark the time
        Send_SYNC = true; //Mark "Send SYNC" flag,
        continue; //and Skip to next for
      }
      // If there is one of the direction pins reset the "Send SYNC" flag
      if (i == 12 | i == 13 || i == 14 || i == 15) {
        Send_SYNC = false; //Do not want to send SYNC together with PAN
      }
      if (NewValuesFMS[i] == 1) {
        SEND_SERIAL(2, i + 32); //on change to high, release button. Add 32 to find proper message.
      }
      if (NewValuesFMS[i] == 0) {
        SEND_SERIAL(1, i + 32); //on change to low press button. Add 32 to find proper message.
      }
    }
  }
  // Process sending the BTN_PAN_SYNC. I want to send it only if there was no direction pins for at least some time (100ms default)
  if ( (millis() > (SYNC_Triggered_Time + SYNC_Send_Time)) && (Send_SYNC == true)) {
    SEND_SERIAL((NewValuesFMS[11] + 1), 43); //Send the value
    Send_SYNC = false; //reset the flag
  }
}

void loop() {

  // Modified code snippet from hktony. Changed keep alive to be less vulnarable for CPU occupancy
  // keep alive for RSG connection
  if (millis() > (Last_Keep_Alive + RSG_Keep_Alive))
  {
    Serial.write("\\####RealSimGear#RealSimGear-G1000XFD#1#3.1.9#656E6B776FA39/\n"); // 3.1.9 = latest firmware; 756E6B776Fd39 = RANDOM ID
    Last_Keep_Alive = millis();
  }
  // Make sure to read encoders every loop to avoid missing clicks
  PROCESS_ENCODERS();
  if (millis() % MCP_Update_Time == 0 ) //Do a comparison once per MCP_Update Time. Default: 50ms. Since we are reading very ralely no debounce needed
  {
    BUTTONS();
  }

  // Update backlight switch status
  // NOT TESTED!
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
