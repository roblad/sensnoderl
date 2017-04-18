#ifndef CONFIGURATION_H
#define CONFIGURATION_H
//************ RADIO MODULE *************
//#define BATTERY_ONLY  //use this set where measure is propagatet through previous colection of data and is being send periodicaly i.e. pulse measure 
#define NODEID 1 // (1 to 30)  //  sensnode with DS set ID 1-5 , sensnode without DS set ID 6-9, senspowermon with tank level set ID 10, other senspowermon set ID 11-15, other nodes 16-39, basenode 30   
#define NETWORK 210
#define NEW_REV // comment for old revision (3.0 or 3.4)
#define BAND 868 // (433 or 868)
#define TXPOWER 4 ////txpower RF12b control variable byte txPower=0;  0 -max power, 7 -minimum there is last byte of comand control rf12_control(0x9850) -means max, rf12_control(0x9857); -means min -21dbm
#define BASEDNODE 31 //define ID of BASENODE where data are colected
#define RFM69 0    // RFM12 compat mode for RFM69CW
//#define SECONDNODE 30 //define ID of second BASENODE i.e. GLCDNODE where data are colected
// #define LOWRATE // use on sensbase too !!
//*********** SETTINGS ******************/
#define WAIT_TIME_MULTIPLY 1 //it gives multiply of Sleepy::loseSomeTime(60000);  65535 - is maximum value for waiting ~65 s and sleepy watchdog
#define RADIO_SYNC_MODE 1 // 1 for power 2 for WDT , battery mode Power-down mode during wait: 0 = NORMAL, 1 = IDLE, 2 = STANDBY  http://jeelabs.net/pub/docs/jeelib/RF12_8h.html#a6843bbc70df373dbffa0b3d1f33ef0ae
/*****************************REMAPING I2C *********************************************/
//there is remaping I2C soft see http://www.arduino.cc/en/Reference/PortManipulation
//Software I2C library http://playground.arduino.cc/Main/SoftwareI2CLibrary
//#define I2C_REMAP //used fior any arduino not for sensnode - no goldpins on board

//*********** SENSORS ******************
#define LDR_SENSOR // use LDR sensor
//*********** DEFINE STRUCTURE FIRST BEFORE GO ON ******************
//*********** DS18B20 ******************
#define DS_COUNT 5 //  if not used and set 1-10 number DS - max 10 sensors - 10 x 2 bytes (2 bytes for 1 DS) in case when emon is not used or used for 1 phase
//#define DS_AS_MAIN_TEMP // uncomment when DS will be main temperature measure
#define DS_BY_ADDRESS_GETTING // define way how these DS sensors temperature getting will be performed, comment for getting by index from 0 to DS_COUNT, in other hand set addresses for each DS[n] device also
//*********** DS18B20 end ******************
//***********SESPOWERMONTX SENSORS 1 or 3 PHASE************** 
//#define NSENSORS 1 // change number only if you use 1 node with emon, 1 -3 are allowed, comment if you not use emon
//*********** END SESPOWERMONTX SENSORS 1 or 3 PHASE ************** 
//#define SHT21_SENSOR // use SHT21
#define BMP_SENSOR // use BMP085 or BMP180
#define DHT_SENSOR // DHT11 or DHT22
#define DHT_SENSOR_TYPE DHT22
// Used devices (comment or uncomment if not used)

//*****************************RELAYS**********************************
#define RELAY_AMOUNT 3  //define amount of relays maximum 12 devided for 4 expanders 3 for each (need to be recoded for different amount of relays on expander)- do not forgot set EXPANDER_AMOUNT IN MAIN PROGRAM !!! - it not work with emon !!!
//maximum 4 expanders 
#if defined RELAY_AMOUNT && !defined BATTERY_ONLY && defined RELAY_AMOUNT > 0 && RELAY_AMOUNT < 13 

#define EXPANDERS_AMOUNT 1  //change value only that amount will be initialised
//define expanders, each expander has adress and  3 relays - relays get app. 20-40 mA each - 12 xgives app 0.25-0.4 A, to be read. define  in main program the addresses for each expander in ADDRESS_x
//set amount of expanders , do not forgot to set aray 
//set expanders addresses in array
//3 relays each expander maximum 4 expanders -set adresses in to array
//change value
#define ADDRESS_0 0x3F   //for low binding for PCF8574A - A0-A2 - see documentation , for PCF8574 0x20
//#define ADDRESS_1 0x3F
//#define ADDRESS_2 //0x39
//#define ADDRESS_3 //0x3d
#endif
//*****************************RELAYS end **********************************


//#define PULSE_MEASURE //for pulse measure enabling for gas water flow - 04.05.15 added for gas and water pulse
//#define TANK_LEVEL //for digital tank level measurement

//*********** OTHER ******************
#define LED_ON // use act led for transmission
//#define DEV_MODE // display output only on console, do not send packages via radio
//#define DEBUG // debug mode - serial output
#define DEBUG_BAUD 9600 // if not define baud rate = 9600bps


#endif //__CONFIGURATION_H

