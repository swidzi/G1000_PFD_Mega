This Arduino program is designed to send G1000 instructions to RealSimGear plugin for MSFS2020.

Requirements:
Arduino Mega 2560
Rotary encoder library
Adafruit_MCP23017 library
CommonBusEncoders library
Wire - Generic I2C control library
My Arduino breakout board/shield (can be modified to work in different configuration)

Code is designed to be uploaded for both PFD and MFD G1000 with or without Autopilot panel.
Constant IsPFD toggles PFD/MFD functionality (PFD = 1, MFD = 0)
Constant IsAPPanelActive toggles AP functionality. (1 = IC3 is populated. 0 = IC3 is not populated and not activated)

MCP23017 Port Extenders are configured with following addresses:
SoftKey switches = 0 (0x20 / 38)
Autopilot switches = 1 (0x21 / 39)
FMS/Range switches = 2 (0x22 / 40)
