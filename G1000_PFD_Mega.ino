#include <Rotary.h>
#include <Adafruit_MCP23017.h>  //MCP23017 library
#include <CommonBusEncoders.h>  //Rotary Encoders on bus
#include <Wire.h> // Generic I2C control library

// Declarations:
const byte IsPFD = 1; //If IsPFD is 1 then all functionalities of the sketch deal with left screen - PFD. Else = MFD
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




//Autopilot Switches


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

//MCP23017 declaraions
Adafruit_MCP23017 SoftKeyMCP;
Adafruit_MCP23017 APMCP;
Adafruit_MCP23017 FMSMCP;


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

void PROCESS_ENCODERS() {

  
}


void loop() {

  // Code snippet from hktony - Thanks!
  // keep alive for RSG connection
  if(millis() % 1000 == 0)
    Serial.write("\\####RealSimGear#RealSimGear-G1000XFD#1#3.1.9#656E6B776FA39/\n"); // 3.1.9 = latest firmware; 756E6B776Fd39 = RANDOM ID

  PROCESS_ENCODERS();

}
