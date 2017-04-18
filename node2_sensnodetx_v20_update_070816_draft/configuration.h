#ifndef CONFIGURATION_H
#define CONFIGURATION_H
//************ RADIO MODULE *************
#define BATTERY_ONLY  //use this set where measure is propagatet through previous colection of data and is being send periodicaly i.e. pulse measure 
#define NODEID 2 // (1 to 30)  //  sensnode with DS set ID 1-5 , sensnode without DS set ID 6-9, senspowermon with tank level set ID 10, other senspowermon set ID 11-15, other nodes 16-39, basenode 30   
#define NETWORK 210
#define NEW_REV // comment for old revision (3.0 or 3.4)
#define BAND 868 // (433 or 868)
#define TXPOWER 0 ////txpower RF12b control variable byte txPower=0;  0 -max power, 7 -minimum there is last byte of comand control rf12_control(0x9850) -means max, rf12_control(0x9857); -means min -21dbm
#define BASEDNODE 31 //define ID of BASENODE where data are colected
#define RFM69 0    // RFM12 compat mode for RFM69CW
//#define SECONDNODE 30 //define ID of second BASENODE i.e. GLCDNODE where data are colected
// #define LOWRATE // use on sensbase too !!
//*********** SETTINGS ******************/
#define RADIO_SYNC_MODE 2 // 1 for power 2 for WDT , battery mode Power-down mode during wait: 0 = NORMAL, 1 = IDLE, 2 = STANDBY  http://jeelabs.net/pub/docs/jeelib/RF12_8h.html#a6843bbc70df373dbffa0b3d1f33ef0ae
#define PULSE_MEASURE //for pulse measure enabling for gas water flow - 04.05.15 added for gas and water pulse
//*********** OTHER ******************
#define LED_ON // use act led for transmission
//#define DEV_MODE // display output only on console, do not send packages via radio
#define DEBUG // debug mode - serial output
#define DEBUG_BAUD 9600 // if not define baud rate = 9600bps

#endif //__CONFIGURATION_H

