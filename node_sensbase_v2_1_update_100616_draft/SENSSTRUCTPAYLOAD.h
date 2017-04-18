#ifndef SENSSTRUCTPAYLOAD_H
#define SENSSTRUCTPAYLOAD_H
/*********************************************************************************************************************/
// DATA STRUCTURE ALWAYS THE SAME FROM BOTH SIDE COMENT IF NOT USED - CURENT FIXED - CAN BE SET as VARIABKLE STRUCTURE - TO DO 
//(according to structure variables coment not used blocks   - it must be the same from node side always)
/**************************** STRUCTURE SIZE DEFINITION FOR EMITER NODE AND RECEIVER***********************************/
#define STRUCT_FIXED_BLOCK 11 //not commen, number set is for lengh of the fixed structure int + int + int + int + byte + byte + int 
#define STRUCT_CONTROL_BYTE 1 //not coment on both side when any DS is used or emon is used
#define STRUCT_DS_COUNT 5 //set max number of DS in network of nodes, coment if there is no any DS in the network coment it in both side, otherwice there is parameter for seting in config.h for nodes to be sent, set maximum amount the same as in highest sensmon amount for base - max 10 sensors - 10 x 2 bytes (2 bytes for 1 DS)
//*********** POWER MONITOR SENSORS ******************
//uncomment when SENSPOWERMONTX, - pin for Voltage sensor A0 (pin 0), current sensor CT1 A1 (pin 1) for CT2 A4 (pin 4) and CT3 A5 (pin 5) // could used with other sensors only for 1 phase control
// could be more sensors,min is the same amount as phases measured - amount of phases 1 or 3. Also see http://boredomprojects.net/index.php/projects/home-energy-monitor***
//***********SESPOWERMONTX SENSORS 1 or 3 PHASE************** 
#define STRUCT_NSENSORS 3 // coment if you do not use emon at all, set phase number, there is NSENSORS parameter in config.h for sensnode, 1-3 are allowed, do not comment if you use any emon - phase * (int + float + int + int) - it will be (loop phase * (int float int ))+ int
#define STRUCT_RELAY_STATUS 1 //do not comment when you use it on node side, status received is codded, relays declared in RELAY_AMOUNT in config.h
//#define STRUCT_RESERVERD 1 // to be changed for any other measurment
#define STRUCT_PULSE_MEASURE 4 //pulse measure for gas and water metter
#define STRUCT_TANK_LEVEL 1 //you can coment on both side when is not used - tank level, always last byte -  text value
/*************************************************************/
//CONVERTER STRUCTURE 
/*
structure is FIXED for any sensmon and any base, data transmitted are set automatically for choosing option in the beginning of the structure.h file
max size of data - 66 bytes
date time - set by RTC localy in basenode incompatibility with sensnodetx - incompatibility with sensnodetx [could be group added for network of nodes 0]
node number received in header
status of data CRC
-------------------------------------------------------------------------------
11 bytes - of data - 1st block -fixed block compatible with sensmon  
1 byte - received from node for control printing data there is a word type coded 4 bits for DS and 4 bits for emon phase
10 - (5 sensors) bytes- maximum 10 DS sensors - can be extended or reduced- maximum amount for highest amount of sensors on the node need to be set, all not used bytes will have 0 value
26 bytes - 3 phases data for emon, could be reduced for 1 phase if 1 phase is measured, if you want to measure 1 phase from other node control byte will decide for print correct data
4 bytes - pulse 
2 bytes - sent values to node
2 bytes - pt1000
2 bytes - lambda
2 bytes - gas status
2 bytes - fan speed
1 byte - tank level 
2 byte - for future usage
max 66 bytes available
---------------------------------------------------------------------------------
    readable format structure taken by converter  set BINARY to 0
   always will be shown from nodes  DATE-DD.MM.YYYY|TIME- HH:MM:SS|CRC-OK/ERR|NODEID- NUM 1-30|LIGHT/FLAME-%|HUMIDITY-%|TEMP-*C|PREASURE – hPa | BAT status 1=bad/0=OK|POWER - V|
   other data for extended DS:  
   
   |[n] DS send from node and amount of phases - coded |TEMPDS[n] - *C | depends of received data from NODES not used has 0 values 
   it can be extended automatically to maximum amount for all nodes defined in structure config [n] 
   emon data:
   |CT1 sensor no. phase 1 and data starts for that phase |Current - A |Power Factor - value 0.xx|Real power -W |Apparent power -VA|
   for 3 phase it will be repeated twice more.
   summarized data for emon for 3 phases:
   |Voltage - V| real power sum - W| apparent power sum -VA| actual consumption - kWh|
   |[n] - coded relay amount and status for each|
   
   |18 bytes| - set for particular meassures
   |tank level - TEXT last value 
 

  
*/


// for emon, each sensor we will send out real power and power factor
#if  defined STRUCT_NSENSORS

typedef struct {
//  byte sensorno : 3;

  int Irms;    // 3 or 1 depends of phase measure A
  float realPower;    // 3 or 1 depends of phase measure W
  int powerFactor;  // 3 or 1 depends of phase measure cos fi

//float apparentPower; // 3 or 1 depends of phase measure VA  reduced payload send
  
} measure_powermon;

#endif 
// structure of send data, with emon data
typedef struct {
  int light;
  int humi;
  int temp;
  int pressure;
  byte lobat :  1;
  int battvol;
#if defined STRUCT_CONTROL_BYTE
  byte controlbits;
#endif
#if defined STRUCT_DS_COUNT
  //array set for DS also required from sensnode side to be defined - max amount of DS on highest DS amount of sensors  

  int temp_DS[STRUCT_DS_COUNT];   //change value for maximum amount // FLEX STRUCTURE WILL BE DONE

#endif

#if defined STRUCT_NSENSORS
//unsigned long secTick;  // time stamp since start
measure_powermon sensor[STRUCT_NSENSORS];  // number of CT sensor 1,2,3 - for 1st phase number will be 0 for 2nd phase 1, for 3th phase 2  int Voltage;  // 1 - shows from phase 1 always // TODO FOR DFLOATING 
  //float realPowersum; //sum of power consumption
    int Voltage;  // 1 - shows from phase 1 always 
#endif

#if defined STRUCT_RELAY_STATUS   //always 13 fields in converter will be printed 12 relays from node maximum
  unsigned int relaystatus; // 1 byte printed in converter will show [n] of relays, next 12 will show they status  read info avaliable in readRelaystatus () function
#endif

//#if defined STRUCT_WIND

//byte windpower; // measured from 0~5 to 50 - 33 m/s = 120 km/h - storm power !!!
//byte winddirection; // compass rose

//#endif 

#if defined STRUCT_PULSE_MEASURE

unsigned long pulse; //pulse reading for  measurment

#endif

#if defined STRUCT_RESERVERD
// ad maximum free bytes size variables in this place, for future reuirements
#endif

#ifdef STRUCT_TANK_LEVEL
  byte tanklevel : 4;
#endif
}Payload;
Payload measure;

/******************COMUNICATION STRUCTURE RECEIVING DATA BY NODE ****************************/

typedef struct {
byte destnode;
byte cmd;
byte state :1;
unsigned long receivedvalue;
} Payloadrx;
Payloadrx rxdata;


/******************COMUNICATION STRUCTURE RECEIVING DATA BY NODE ****************************/

#endif //__SENSSTRUCTPAYLOAD_H
