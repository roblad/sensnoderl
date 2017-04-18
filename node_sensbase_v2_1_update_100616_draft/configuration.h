#ifndef CONFIGURATION_H
#define CONFIGURATION_H
/*************************************************************/
// Output
#define ACT_LED 9
#define NEW_REV
//#define BINARY   // comment for converter rf12demo https://github.com/jcw/jeelib/blob/master/examples/RF12/RF12demo/RF12demo.ino
//#define DEBUG 
//#define SECONDNODE 30
#define NODEID  31  // 31 for any node(all) // sensnode with DS ID 1-5 , sensnode without DS ID 6-9, senspowermon with tank level 10, other senspowermon ID 11-15, other nodes 16-29, gold basenode 30   
#define NETWORK    210 // 0-255
#define BAND 868 // (433 or 868)
#define RFM69 0
//#define DEBUG 0 // debug mode - serial output (do zmiany na deklaracje)
#define DEBUG_BAUD 9600 // if not define baud rate = 9600bps
#define RADIO_SYNC_MODE 1 //1 for power 2 for WDT , battery mode Power-down mode during wait: 0 = NORMAL, 1 = IDLE, 2 = STANDBY  3 power-off WDT to be tested ,for standby of radio
/*************************************************************/
//*********** DEFINE STRUCTURE FIRST BEFORE GO ON ******************
//#define RTC_CLOCK  //uncomment when RTC is connected to sensbase, last record in readable form will be timestamp
//#define DS1307_CLOCK    //uncoment proper RTC chip
//#define DS3231_CLOCK  // uncoment proper RTC chip
//#define RF12_EEPROM_VERSION // eeprom parameters when RTC_CLOCK with EPROM is plugged rf12demo https://github.com/jcw/jeelib/blob/master/examples/RF12/RF12demo/RF12demo.ino
 
//*********** SETTINGS ******************
//*********** DS18B20 ******************
#define DS_COUNT 5 // if not used set to 0 from base side, for node comment it, or set 1-10 number DS - max 10 sensors - 10 x 2 bytes (2 bytes for 1 DS) in case when emon is not used or used for 1 phase
//*********** DS18B20 end ******************
//*********** POWER MONITOR SENSORS ******************
// Used devices (comment or uncomment if not used)
//***********SESPOWERMONTX SENSORS 1 or 3 PHASE************** 
#define NSENSORS 3// change number only if you use 1 node with emon, 1 -3 are allowed,
#define RELAY_AMOUNT //define relays converting - for base node amount of relays not needed, do not forgot set RELAY SETTINGS IN MY PROGRAM in node!!!
#define PULSE_MEASURE //for pulse measure enabling for gas water flow - 04.05.15 added for gas and water pulse
#define TANK_LEVEL //for digital tank level measurement

#endif //__CONFIGURATION_H

