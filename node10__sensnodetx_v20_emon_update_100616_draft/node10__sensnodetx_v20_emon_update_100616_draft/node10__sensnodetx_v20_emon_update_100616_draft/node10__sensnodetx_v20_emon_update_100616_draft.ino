#include <Arduino.h>

// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3
/*
SensnodeTX v3.4-dev
 Written by Artur Wronowski <artur.wronowski@digi-led.pl>
 Need Arduino 1.0 to compile
 TODO:
 - wiartomierz //#define ObwAnem 0.25434 // meters
 sensnodeTX v2,0-by RL  
 senspowermon  http://openenergymonitor.org,tankmon ,DS [n] sensors by RL source 
*/
#include "configuration.h"

//*********** BATTERY SAVING ********
#ifdef BATTERY_ONLY

  #define VCC_OK 3400 // enough power for normal 1-minute sends
  #define VCC_LOW 3200 // sleep for 1 minute, then try again
  #define VCC_DOZE 3000 // sleep for 5 minutes, then try again
// sleep for 60 minutes, then try again
  #define VCC_SLEEP_MINS(x) ((x) >= VCC_LOW ? 1 : (x) >= VCC_DOZE ? 5 : 60)
  #define VCC_FINAL 2900 // send anyway, might be our last swan song

#endif

/*
I2C - ports change for any unused ports in Arduino - not for sensnode
PORTD maps to Arduino digital pins 0 to 7  (digital pin 8 to 13), C (analog input pins), D (digital pins 0 to 7) 
PORTD maps to Arduino digital pins 0 to 7: DDRD - The Port D Data Direction Register - read/write, PORTD - The Port D Data Register - read/write, PIND - The Port D Input Pins Register - read only 
PORTB maps to Arduino digital pins 8 to 13 The two high bits (6 & 7) map to the crystal pins and are not usable: DDRB - The Port B Data Direction Register - read/write, PORTB - The Port B Data Register - read/write, PINB - The Port B Input Pins Register - read only. 
PORTC maps to Arduino analog pins 0 to 5. Pins 6 & 7 are only accessible on the Arduino Mini: DDRC - The Port C Data Direction Register - read/write, PORTC - The Port C Data Register - read/write, PINC - The Port C Input Pins Register - read only 
*/
//I2C remaping for getting more analog pins avaliable
//remap pin from standard A5 - SCL to port PD6 and A4 - SDA to port PD5
//
//Library avaliable hiere https://github.com/felias-fogg/SoftI2CMaster
/*
#ifdef I2C_REMAP
#define SCL_PIN 6  //SCL A5 pin remaped
#define SCL_PORT PORTD
#define SDA_PIN 5 //SDA A4 pin remaped
#define SDA_PORT PORTD
#include <SoftI2CMaster.h>
#include <avr/io.h>
#define I2C_TIMEOUT 50
#define I2C_FASTMODE 1 
#endif
*/

// libs for I2C and DS18B20
#include <Wire.h>
#include <OneButton.h> // one buton functions to operate in functions (sensbase GOLD) http://www.mathertel.de/Arduino/OneButtonLibrary.aspx 
//http://majsterkowo.pl/zegar-ds1307/ - zegar
#include <OneWire.h>
#include <DallasTemperature.h>
// lisb for SHT21 and BMP085
#include <BMP085.h>
#include <SHT2x.h>
//#include <Adafruit_BMP085.h>
// lib for RFM12B from https://github.com/jcw/jeelib
#if RFM69 == 1
   #define RF69_COMPAT 1
#else
   #define RF69_COMPAT 0
#endif
#include <JeeLib.h>
// avr sleep instructions
#include <avr/sleep.h>
// DHT11 and DHT22 from https://github.com/adafruit/DHT-sensor-library
#include "DHT.h"
//#include "EmonLib.h"  // Include Emon Library for standard mon for 1 phase, updated by EmonLib3_PH.h
//Arduino Energy Monitoring Library - compatible with Arduino 1.0
//Designed for use with emonTx: http://openenergymonitor.org/emon/Modules
//**************************************************************
#ifdef NSENSORS
#include "EmonLib_3PH.h"     //https://github.com/openenergymonitor/ArduinoDue_3phase
//Based on emonLib: https://github.com/openenergymonitor/EmonLib
//Modifed for Arduino Due and 3 Phase by icboredman (boredman@boredomprojects.net) - Jan 2014
//http://boredomprojects.net/index.php/projects/home-energy-monitor
//Hardware senspowermon http://boredomprojects.net/images/Articles/HomeEnergy/EMS_schematic.pdf 
#endif
#ifndef NSENSORS // does not work with 3 phase
//*********************add PCF8574 lib can be used only for emon 1 phase ***********************************
// there are spom problems when emon 3 phase  and relays exist !!! -- analog pins must be changed - only any arduino can be used for 3 phase and relays
// * Dependencies */ <Wire.h> - it cannot be used in sensmon , "PCint.h" library taken for http://skyduino.wordpress.com/ in GitHub - PCF8574_INTERRUPT_SUPPORT 
//https://github.com/skywodd/pcf8574_arduino_library
#include  <PCF8574.h>  // expander I2C - http://akademia.nettigo.pl/starter_kit_030/programowanie_cd.html
// try to modyfy to lib http://playground.arduino.cc/Main/PCF8574Class
//*************************************************************
#endif

#include "SENSSTRUCTPAYLOAD.h"
//txpower RF12b control variable
byte txPower = TXPOWER;   //0 -max power, 7 -minimum there is last byte of comand control rf12_control(0x9850) -means max, rf12_control(0x9857); -means min -21dbm


//set mili timer for sent package periodicaly and switch to receiving mode  //sending timer
MilliTimer sendTimer;   // see https://github.com/jcw/jeelib/blob/master/examples/RF12/timedSend/timedSend.ino

//add additional control timer variables
//static
byte pendingOutput;

//sending time for RELAYS - check sending timer before set
#if defined RELAY_AMOUNT
    
    byte delayed = 1710 / WAIT_TIME_MULTIPLY;
    
    #else
    byte delayed = 0;
    
#endif

// Pulse counting settings
#ifdef PULSE_MEASURE 
 
//

volatile unsigned long pulseCount = 0;                                           // Number of pulses, used to measure gass 0,01 m3 for each.
volatile unsigned long lastBlink = 0;
volatile byte flag_pulse = 0;
#endif
/**************RELAYS SETTINGS***********************************/ 
// controlbits 12 byte after fixing 11 bytes in structure - if you want to extend fixed data block above 11 change sequence of control bitsvariable
#if defined RELAY_AMOUNT && defined STRUCT_RELAY_STATUS && !defined BATTERY_ONLY && defined EXPANDERS_AMOUNT && defined RELAY_AMOUNT > 0 && RELAY_AMOUNT < 13 

volatile unsigned int count_relaystatus=0; //relaystatus 
volatile byte coderelay = 0; //coderelayctatus execution check
volatile static byte RelayValue[RELAY_AMOUNT];  //sum of relays from all expanders

#endif


//bit control for sending post fixed data and decoding it in receiver side 
#ifdef STRUCT_CONTROL_BYTE

 byte dsSensoramount;
 byte phaseAmount;

#endif

//DHT requirements
#ifdef DHT_SENSOR
float h; // define humidity DTH variable
float t; //define temperature DTH variable
#define DHTTYPE DHT_SENSOR_TYPE
#define DHTPIN 3 // port P1 = digital 4 fo nano 3
#endif

//i2c condition
#if defined SHT21_SENSOR || defined BMP_SENSOR || defined RELAY_AMOUNT
//switch emon to 1 phase when I2C is used
 
 #if !defined NSENSORS || NSENSORS  == 1  
 //set i2c 
    #define I2C
    

//set expanders instances   
  #if defined RELAY_AMOUNT && !defined BATTERY_ONLY && defined EXPANDERS_AMOUNT && defined ADDRESS_0 || defined ADDRESS_1 || defined ADDRESS_2 || defined ADDRESS_3

  #define LIST_OF_VARIABLES  X(EXPANDER0) X(EXPANDER1)  X(EXPANDER2) X(EXPANDER3)
  #define X(list) PCF8574 list;
  LIST_OF_VARIABLES
  #undef X
 
  #endif
 #endif
#endif

//set debug for RS printing from node side
#ifdef DEV_MODE
#define DEBUG
#endif

#ifndef DEBUG_BAUD
#define DEBUG_BAUD 9600
#endif



// settings, pin choice, voltage and curent sensors calibration, measurment delay
#if defined NSENSORS  
// Undefine sensors when 3 phase is set

 #if defined SHT21_SENSOR || defined BMP_SENSOR || defined RELAY_AMOUNT || NSENSORS == 3 //check if defined emon sensors for 3 phase , undefine because of pin occupation for I2C
   #undef SHT21_SENSOR
   #undef BMP_SENSOR
   #undef STRUCT_RELAY_STATUS
   #undef I2C
 #endif

// Input/Output definition for emon
// Analog

 #if NSENSORS == 1 || NSENSORS == 3 //set pin for CTS1 and Voltage pin
   #define VOLTAGE_PIN 0  //P1
   #define CURRENT_SENSOR1_PIN 1  //P2
 #endif
 
 #if NSENSORS == 3  //set pins for CTS2 and CTS 3
   #define CURRENT_SENSOR2_PIN 2   //SCL nano 2 standard 5
   #define CURRENT_SENSOR3_PIN 3   //SDA nano 3 standard 4
 #endif

 #if NSENSORS == 1 ||  NSENSORS == 3
// end Input/Output definition for emon
//voltage calibration only for one phase where AC/AC adapter is plugged

 #define VCAL 246.7778727
 #define VCAL_CURENT_PHASE_SHIFT 1.7  //default is 1.7
//current calibration
 #define CURRENT_SENSOR1_CALIBRATION 30.25 //calibration for CT1

 #endif

 #if NSENSORS == 3
 #define CURRENT_SENSOR2_CALIBRATION 29.95 //calibration for CT2
 #define CURRENT_SENSOR3_CALIBRATION 30.25 //calibration for CT3

 #endif
/*see http://openenergymonitor.org/emon/buildingblocks/ct-and-ac-power-adaptor-installation-and-calibration-theory
http://openenergymonitor.org/emon/buildingblocks/explanation-of-the-phase-correction-algorithm
description of voltage calibration:
voltage constant = 230 x 11 / (9 x 1.20) = 234.26
voltage constant calibration coefficient = 230[V measured] x 11[transformer ratio]  / (9[V ideal output] x 1.20 (%power jumping and differences between ideal output and real output without load ) = 234.26
example gives 234.26 real counted for TEZ 0.5/D 230/6 real max voltage without load - 9.8 V , % jumping - 1,303333333 = 248,2370942					
Voltage: input pin, calibration - -error 0,03 % 	
	Check the voltage calibration again. It might need a slight adjustment if the phase angle calibration was altered significantly. 
Recheck the phase angle calibration [see below].				
Correction of calibration:  
New calibration = existing calibration x (correct reading / emon reading)
http://openenergymonitor.org/emon/buildingblocks/explanation-of-the-phase-correction-algorithm
If you are using the voltage input, with the load connected adjust the phase angle calibration 1.7 in lines like this const float phase_shift=                  1.7 – default so that real power and apparent power read the same value (and power factor is as close to 1.00 as possible). –resistance load need to be measured for that calibration. Your meter is not needed for this, there is small error to carry out of that. The phase calibration coefficient should not normally go outside the range 0.0 - 2.0 degr.	 [my test gives 1.01 value].By modifying the software to report the time it takes to complete the inner measurement loop and the number of samples recorded, the time between samples was measured as 377 micros. This equates to 6.79 deg. (360deg. takes 20 ms). Therefore a value of 1 applies no correction, zero and 2 apply approximately 7 percent of correction in opposite directions. A value of 1.28 will correct the inherent software error of 2 percent that arises from the delay between sampling voltage and current.
Description of current calibration:
The supplied current is measured using a current transformer, and the resulting (small) current converted into a voltage by the burden resistor. This voltage is then measured by the analog input of the controler.
Thus the number seen by the controller is:
Irms = count x a constant = current constant x (3.3 / 1024)
here:
input pin voltage = secondary current x burden resistance
and:
secondary current = primary current / transformer ratio
current constant[for CTS 100A] = (100 / 0.050) / 18 = 111.11
If you use a current transformer with a built-in burden (voltage output type) As CTS 030A
The Yhdc SCT-013-030 gives 1 V at a rated current of 30 A, so for this transformer you have: current constant = 30 x 1 = 30
*/

// Instances of EmonTX sensors according to phase
 EnergyMonitor emon[NSENSORS];
#endif

// Input/Output definition for sensmon
// Analog
#ifdef NEW_REV //3.4/4
#define LDR_PIN 6 //standard 2 nano 6 new 7
#define BAT_VOL 7 //standard 3 nano 7 new 6
#else //3.0
#define LDR_PIN 0
#define BAT_VOL 1
#define CustomA3 3
#endif
// Digital
#ifdef NEW_REV //3.4
#define MOSFET 7  //(can be used for other reason when mosfet is not avaliable)
#define ONEWIRE_DATA 8
#define ACT_LED 9
#define EXPANDERIRQ 5 //set PIN P2 DIO interput PCF 
#else //3.0
#define CustomD3 3
#define CustomD4 4
#define MOSFET 5
#define ONEWIRE_DATA 8
#define ACT_LED 9 //standard 9 uno 13 
#endif

#ifdef TANK_LEVEL && !defined I2C

byte low_level_digi_pin = 5;
byte medium_level_digi_pin = 6;
byte high_level_digi_pin = 7;


#endif

//measure variables for sensmon
byte count = 0;
unsigned int adcreading;  //for bang up voltage
int volts;

// ***********set parameters for DS18B20****************
#if defined DS_COUNT && DS_COUNT > 0 
DeviceAddress tempDeviceAddress; //for automatic getting DS device parameters in loops

// define precision of DS temperature displaying 9,10,11,12  - 12 is the best but longer reading is needed
#define TEMPERATURE_PRECISION 11  

byte numberOfDevices_DS;  // number of DS devices found variable used for measure 

OneWire oneWire(ONEWIRE_DATA);
DallasTemperature sensors(&oneWire);

// arrays to hold DS device results for transformation to integer
int ds_array[DS_COUNT];

// Set device addresses if you want to get temperature by address of DS device (check printed DS addresses in DEBUG mode) for each HEX add 0x...) and put here accordingly
  #ifdef DS_BY_ADDRESS_GETTING

 static byte Address_DS[DS_COUNT][8];  //variable for DS addresses
 
const byte Address_DS1[8] = { 0x28, 0xC4, 0xCE, 0x4C, 0x05, 0x00, 0x00, 0x1A };
const byte Address_DS2[8] = { 0x28, 0x92, 0x64, 0x4D, 0x05, 0x00, 0x00, 0x87 };
const byte Address_DS3[8] = { 0x28, 0xF6, 0xDD, 0x4B, 0x05, 0x00, 0x00, 0x2D };
const byte Address_DS4[8] = { 0x28, 0xC1, 0x08, 0x38, 0x05, 0x00, 0x00, 0xE9 };
const byte Address_DS5[8] = { 0x28, 0x0D, 0xEB, 0x4C, 0x05, 0x00, 0x00, 0x39 };


//Address_DS[n][8] = { 0x28, 0x59, 0xBE, 0xDF, 0x02, 0x00, 0x00, 0x9F };  
//Address_DS[n][8] set according to DS_COUNT [n]=[DS_COUNT] - address must be found in DEBUG mode DS device
  
  #endif


#endif

// emon variables 
// Time (ms) to allow the filters to settle before sending data
#if defined NSENSORS && (NSENSORS == 1 || NSENSORS == 3)
#define FILTERSETTLETIME 10000 //used for complete sample getting for emon  
//boolean settled = false;  //used for complete sample getting for emon
//unsigned long lastTick = 0; //used for complete sample getting for emon
int lastSampleTime_kwh, sampleDuration_kwh;
long lastmillis_kwh;   //time
float realPowersumcollect; //temp variable for summarizing power
//float realPowersum; //local variable for summarizing power
#endif 

//WDT set for uno and specyfic bootloaders - ned to be werified whether Watchdog working on
ISR(WDT_vect) { Sleepy::watchdogEvent(); }


#ifdef NEW_REV
Port p1 (1); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p2 (2); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p3 (3); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p4 (4); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20

#endif

#ifdef DHT_SENSOR
DHT dht(DHTPIN, DHTTYPE);
#endif



//i2cscanner TODO

/*
http://aeroquad.com/archive/index.php/t-1064.html?s=1569d8a6149fdf30b10ecb13926e062b

tool for find everything

http://cbxdragbike.com/arduino/arduino_exp.html

*/



//***************************SETUP BEGIN*********************//


void setup() {



#ifdef TANK_LEVEL

pinMode(low_level_digi_pin, INPUT); 
pinMode(medium_level_digi_pin, INPUT);  
pinMode(high_level_digi_pin, INPUT);

#endif  

//setings BAND - declared in setup
#if BAND == 433
  rf12_initialize(NODEID, RF12_433MHZ, NETWORK);
#endif
#if BAND == 868
  rf12_initialize(NODEID, RF12_868MHZ, NETWORK);
#endif

#if defined LOWRATE && defined  RFM69 == 0

rf12_control(0xC040); // 2.2v low
#endif

#if defined LOWRATE  && defined RFM69 == 0
  
  rf12_control(0xC623); // ~9.6kbps
#endif

//#if defined LOWRATE & defined RFM69 == 1
  //rf12_control(0xC623); // ~9.6kbps
//#endif

#if  defined  RFM69 == 0
rf12_control(0x985|(txPower > 7 ? 7 : txPower)); //power control 0 means max and maximum power consumption

#else

rf12_control(0x985|(txPower > 7 ? 7 : txPower)); //power control 0 means max and maximum power consumption
RF69::control(0x91, 0x9F); // Maximum TX power

#endif
//rf12_sleep(RF12_SLEEP);   //sleep RF module
//rf12_control(0x8280);

//define INT1 in pin 3
#ifdef PULSE_MEASURE 
//digitalWrite(3, HIGH);
pinMode(3, INPUT_PULLUP);
//digitalWrite(3, HIGH);
attachInterrupt(1, onPulse, FALLING); 
#endif

//DHT initialize
#ifdef DHT_SENSOR
  dht.begin();
#endif

#ifdef I2C
  //switch emon to 1 phase when DHT is set
  #ifdef NSENSORS
  #define NSENSORS 1
  #endif
  Wire.begin();

#endif

#ifdef DEBUG
  Serial.begin(DEBUG_BAUD);
#endif

#if !defined DEBUG && defined RELAY_AMOUNT || defined PULSE_MEASURE

Serial.begin(DEBUG_BAUD);

#endif


//emon set parameters - calibration
#if defined NSENSORS && (NSENSORS == 1 || NSENSORS == 3)
  //set reference voltage 3.3 V
  analogReference(DEFAULT);
  //emon parameters   - voltage is for phase 1 and is set for all phases       
  emon[0].voltage(VOLTAGE_PIN,VCAL, VCAL_CURENT_PHASE_SHIFT,1); //Voltage: input pin, calibration, phase_shift for phase 1
  emon[0].current(CURRENT_SENSOR1_PIN, CURRENT_SENSOR1_CALIBRATION);       // Current: input pin, calibration.
  #if NSENSORS == 3
  emon[1].voltage(VOLTAGE_PIN,VCAL,1.7,2); //Voltage: input pin, calibration, phase_shift for phase 2
  emon[1].current(CURRENT_SENSOR2_PIN, CURRENT_SENSOR2_CALIBRATION);       // Current: input pin, calibration.
  emon[2].voltage(VOLTAGE_PIN,VCAL,3,3); // Voltage: input pin, calibration, phase_shift for phase 3
  emon[2].current(CURRENT_SENSOR3_PIN, CURRENT_SENSOR3_CALIBRATION);       // Current: input pin, calibration.
  #endif 
#endif // closing ifdef POWERMON 


//expander initialisation with defined addresses set on the begining
#if defined EXPANDERS_AMOUNT && EXPANDERS_AMOUNT < 5

//set PIN P2 DIO interput PCF EXPANDERIRQ pin

pinMode(5, INPUT);
digitalWrite(5, HIGH);



//initialise expanders
 #ifdef ADDRESS_3 
  #if EXPANDERS_AMOUNT > 3
 for (byte i=0; i<3; i++) {
       EXPANDER3.pinMode(i, OUTPUT);
       }
     //attached  for P4-P6 (one P3 and P8 free for usage ) EXPANDER0
    for (byte i=4; i<7; i++) {
       EXPANDER3.pinMode(i, INPUT);
       EXPANDER3.pullUp(i);
       }     
  
       EXPANDER3.set();
       EXPANDER3.begin(ADDRESS_3);
  
  #endif
 #endif

 #ifdef ADDRESS_2
  #if EXPANDERS_AMOUNT > 2
 for (byte i=0; i<3; i++) {
       EXPANDER2.pinMode(i, OUTPUT);
       }
     //attached  for P4-P6 (one P3 and P8 free for usage ) EXPANDER0
 for (byte i=4; i<7; i++) {
       EXPANDER2.pinMode(i, INPUT);
       EXPANDER2.pullUp(i);
       }
       EXPANDER2.set();
       EXPANDER2.begin(ADDRESS_2);

  #endif
 #endif


 #ifdef ADDRESS_1    
  #if EXPANDERS_AMOUNT  >1
   for (byte i=0; i<3; i++) {
       EXPANDER1.pinMode(i, OUTPUT);
       }
    //attached  for P4-P6 (one P3 and P8 free for usage ) EXPANDER1
    for (byte i=4; i<7; i++) {
       EXPANDER1.pinMode(i, INPUT);
       EXPANDER1.pullUp(i);
       }
       EXPANDER1.set();
       EXPANDER1.begin(ADDRESS_1);
  
  #endif
 #endif

 #ifdef ADDRESS_0
  #if EXPANDERS_AMOUNT > 0
    
  
   for (byte i=0; i<3; i++) {
       EXPANDER0.pinMode(i, OUTPUT);
       }
   //attached  for P4-P6 (one P3 and P8 free for usage ) EXPANDER0
    for (byte i=4; i<7; i++) {
       EXPANDER0.pinMode(i, INPUT);
       EXPANDER0.pullUp(i);
       }
 
       EXPANDER0.set();
       EXPANDER0.begin(ADDRESS_0);
       
  EXPANDER0.enableInterrupt(5, onInterrupt_EXPANDER0);
  EXPANDER0.attachInterrupt(4, P1_onPin4_EXPANDER0, FALLING);
  EXPANDER0.attachInterrupt(5, P2_onPin5_EXPANDER0, FALLING);
  EXPANDER0.attachInterrupt(6, P3_onPin6_EXPANDER0, FALLING);       

  #endif
 #endif
 
#endif 



//test of relay values

//EXPANDER0.digitalWrite(0, 1);
//EXPANDER0.digitalWrite(1, 0);
//EXPANDER0.digitalWrite(2, 0);
//EXPANDER0.digitalWrite(3, 1);
//EXPANDER0.digitalWrite(4, 1);
//EXPANDER0.digitalWrite(5, 1);
//EXPANDER0.digitalWrite(6, 1);
//EXPANDER0.digitalWrite(7, 1);


} //end setup



//Functions declaration

#if defined STRUCT_CONTROL_BYTE 
//set control bits function 
static void setControlbits() {

//coder for controlbits

#if defined NSENSORS && (NSENSORS == 1 || NSENSORS == 3)

//phaseAmount=0;

if (NSENSORS > 3) {
  phaseAmount=0; 
} else {
  phaseAmount = NSENSORS;
}
#endif


#if defined DS_COUNT

//dsSensoramount=0;

if (DS_COUNT > 10 ) {

  dsSensoramount=0; 

   } else {
   dsSensoramount=DS_COUNT;
}

#endif

measure.controlbits=(dsSensoramount * 16) + phaseAmount;  //sumaraise 2 digits to HEX 16 word wher 10ns are as a first 4 bits and phase as second 4 bits 






#if defined DS_AS_MAIN_TEMP && defined DS_COUNT  && defined DS_BY_ADDRESS_GETTING && defined  DHT_SENSOR && defined SHT21_SENSOR || defined NSENSORS 

//decoder used also in base side
phaseAmount=measure.controlbits & 15;
dsSensoramount= measure.controlbits  >> 4;
  #if defined  DEBUG 
Serial.println();
Serial.print("Control byte set: |");
Serial.print(measure.controlbits);


Serial.print("| Amount of DS sensors set: |");
Serial.print(dsSensoramount);


Serial.print("| Amount of power measure phases send: ");
Serial.println(phaseAmount);
delay (2);
  #endif

#endif

}
#endif


//sending packages through radio - only for battery
#ifdef BATTERY_ONLY 
#undef RELAY_AMOUNT
#define RADIO_SYNC_MODE 2
#else
#define RADIO_SYNC_MODE 1
//sending packages through radio for battery only -removed ArtekW conception
#endif
static void doReport() {

  if (rf12_canSend()){
  rf12_sendStart(BASEDNODE, &measure, sizeof measure); //send in sync mode last partametr 1
  
  }
           if (RF12_WANTS_ACK) {
              rf12_sendStart(RF12_ACK_REPLY,0,0);
             
              } else {
              
                rf12_sendStart (BASEDNODE, &measure, sizeof measure);  //sending data second time when no ACK requested, for to be sure that package can be received.
                
             }
  #ifdef SECONDNODE
       if (rf12_canSend()){
       rf12_sendStart(SECONDNODE, &measure, sizeof measure);
       
       }
            if (RF12_WANTS_ACK) {
                rf12_sendStart(RF12_ACK_REPLY,0,0);
                
            }  

  #endif

  rf12_sendWait(RADIO_SYNC_MODE);
  
}



//reference voltage bangup

long readVcc() {
  long resultVcc; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(20); // Wait for Vref to settle - 2 was inadequate
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  resultVcc = ADCL;
  resultVcc |= ADCH<<8;
  resultVcc = 1126400L / resultVcc; // Back-calculate AVcc in mV
  return resultVcc; 
}
//battery control 
int battVolts(void) {


#if defined(__AVR_ATmega168__)
 analogReference(DEFAULT);
#else
 analogReference(INTERNAL);
#endif
   for (int i=0; i<5;i++ ) {
    readVcc();
    delay (1);
   }
  double vccref = readVcc()/1000.0;
  adcreading = analogRead(BAT_VOL) * 2;
  vccref = readVcc()/1000.0;
  adcreading = analogRead(BAT_VOL) * 2;  
  delay(2);
  double battvol = (adcreading / 1023.0) * vccref;
  return battvol * 1000;
}
//LED activation
static void activityLed (byte on) {
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, on);
  delay(50);
}



//get temperature by address function it is only for potential usage - test for optymalisation neede

/*
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print((float)tempC,2);
}
*/

//get temperature by address function

/*
//with integer conversion get temperature from DS
int getTemperature (DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  return tempC*100;
}
*/


// function to print a DS device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) {
    Serial.print(", ");
    }
  }
}

//get values and set for transmission, DEBUG print to serial
static void transmissionRS() {
#ifdef DEV_MODE
  Serial.println("==DEV MODE==");
  delay(2);
#endif

  
  //activityLed(1);
  Serial.println(' ');
  delay(2);


#if defined DS_COUNT && DS_COUNT > 0 

#ifdef DEBUG
  Serial.print("DS18B20 found: ");
 
  
  numberOfDevices_DS=sensors.getDeviceCount();
  Serial.println(numberOfDevices_DS, DEC);
  
 
  for (byte i=0; i<numberOfDevices_DS; i++) 
   { 

      
      if(!sensors.getAddress(tempDeviceAddress, i))
      {
      
      Serial.println();
      Serial.print("Unable to find DS address check connection of DS device");
      Serial.println();
      delay (2);
      break;
              
      } else {
      
      Serial.print(" Address for DS [");
      Serial.print(i);
      Serial.print("] = ");
      delay (2);
      printAddress(tempDeviceAddress);
       
      Serial.print(" TEMP: ");
      Serial.print((float) (ds_array[i]*0.01),2);
      Serial.print(" *C");
      Serial.print (" Resolution setting is: ");
      Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
      Serial.println(' ');
      delay(2);  
      
        
      }
  }

  #endif
#endif

  Serial.println();
  #ifdef LDR_SENSOR
  Serial.print("LIGHT ");
  Serial.println((float)measure.light/10,2);
  delay(2);
  #endif
  #if defined  SHT21_SENSOR || defined DHT_SENSOR 
  Serial.print("HUMI ");
  Serial.println((float) measure.humi/10,2);
  delay(2);
  #endif
  #if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING 
  Serial.print("TEMP ");
  #endif
  //internal temp *
  #if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING && !defined  DHT_SENSOR && !defined SHT21_SENSOR  
  Serial.print("INTERNAL ");
  #endif
  #if defined DS_AS_MAIN_TEMP && defined DS_COUNT  && DS_COUNT > 0
  Serial.print("DS0 ");
  #endif 
  Serial.println((float)measure.temp*0.1,2);
  delay(2);
  
  #if defined BMP_SENSOR
  Serial.print("PRES ");
  Serial.println((float)measure.pressure/10,2);
  delay(2);
  #endif
  Serial.print("LOBAT " );
  Serial.println(measure.lobat, DEC);
  delay(2);
  Serial.print("BATVOL ");
  Serial.println((float)measure.battvol/1000,3);
  delay(2);
  Serial.println(' ');
  delay(2);
 #ifdef PULSE_MEASURE 
        
          Serial.print("PULSE COUNT TO SEND: ");
          delay(10);
          Serial.println((unsigned long)measure.pulse,DEC);
          delay(10);
          Serial.println(' ');
          delay(2);
  #endif
           

 // emon print
#if defined NSENSORS && (NSENSORS == 1 || NSENSORS == 3)

Serial.println("Power measurment:");

for(byte i=0; i<NSENSORS; i++) {
   
    Serial.print(" Phase: "); Serial.print(i+1);Serial.print(" CT sensor: "); Serial.print(i); 
    Serial.print(" <> Measured: ");
    delay(2);
    Serial.print(" Current="); Serial.print(float((measure.sensor[i].Irms)*0.01)); Serial.print(" A, ");
    delay(2);
    Serial.print(" Real Power="); Serial.print((measure.sensor[i].realPower)); Serial.print(" W, ");
    delay(2);
    Serial.print(" Appartent Power="); Serial.print((float)((measure.sensor[i].realPower)/((measure.sensor[i].powerFactor)*0.01)),2); Serial.print(" VA, ");
    delay(2);
    Serial.print(" Power Factor cos "); Serial.write(216); Serial.print(" ="); Serial.println(float((measure.sensor[i].powerFactor)*0.01));
    Serial.println(' ');
    delay(2);
    
   // emon[i].serialprint(); test value for proper displaying - debug for DEBUG
  }
  Serial.print("Voltage: "); Serial.print(((measure.Voltage)*0.01)); Serial.print(" V ");
  Serial.println(' ');
  delay(2);
  Serial.print("  Real Power sum=");  Serial.print((realPowersumcollect));  Serial.print(" W ");
  Serial.println(' ');
  delay(2);
  
  sampleDuration_kwh = ((millis() - lastSampleTime_kwh)/1000); // Sample duration in seconds for counting kWh
  delay(2);
  Serial.print("  Power consumption="); Serial.print(float ((((realPowersumcollect) / 1000)/3600)* sampleDuration_kwh),4); Serial.print(" kWh ");  //aktual consumption kWh
  Serial.println(' ');
    
  delay (5);


#endif
}

//get measure for sending
static void doMeasure() {       //main measure
  
  count++;
  adcreading = 0;
  measure.lobat = rf12_lowbat();  //probably if RF 69 it should be commented and measure should be changed 
 
  measure.battvol = battVolts();

#ifdef LDR_SENSOR
  //if ((count % 2) == 0) {  // measuring once per two cycles only for photoresistor usage - uncoment if declaration and closing bracket

  #if defined (__AVR_ATmega168__)  //test proc - internal function
    analogReference(DEFAULT);
  #else
    analogReference(INTERNAL);
  #endif
    //measure.light = (analogRead(LDR_PIN));
    // use for photoresistor
    double vccref_ldr = readVcc()/1000.0;
    unsigned int adcreading_ldr = analogRead(LDR_PIN);
    delay (5);
    double light_ldr = ((adcreading_ldr / 1023.0) * vccref_ldr);
    double measure_ldr = light_ldr * 1000;
    measure.light = map(measure_ldr,((vccref_ldr*1000)),0,0,1000);
    // it is for sensors 5V/3.3V-0 - ((vccref_ldr*1000)-1700) - for 5V (vccref_ldr*1000) for 3.3V
    // use for any Chinese arduino sensors with LM386 chip,
    // plug to 3.3V and A3 to LDR  - check polarity,
    // remove R3 resistor from PCB. type http://botland.com.pl/373-czujniki-do-arduino */
 // }

//sensors initialise and getting values for measure

#endif

#ifdef I2C
  
  #ifdef SHT21_SENSOR
  float shthumi = SHT2x.GetHumidity();
  measure.humi = shthumi * 10;

  float shttemp = SHT2x.GetTemperature();
  measure.temp = shttemp * 10;
  #endif

  #ifdef BMP_SENSOR
  
 
   //Sleepy::loseSomeTime(250);

  
  BMP085.getCalData();
  BMP085.readSensor();
  delay (10);
  measure.pressure = ((BMP085.press*10*10) + 16);
  #endif

#endif


#ifdef DHT_SENSOR
  #ifndef SHT21
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    return;
  }
  else {
    measure.humi = h*10;
    measure.temp = t*10;
  }
  #endif

#endif



#if defined DS_COUNT && DS_COUNT > 0 
  
  sensors.begin();
  oneWire.reset_search();
    //measure.controlbits=0;numberOfDevices_DS=0;
    numberOfDevices_DS = sensors.getDeviceCount();
 if (numberOfDevices_DS > DS_COUNT) numberOfDevices_DS=DS_COUNT;   //set proper values for DS amount per found real attached DS - only declared value in configuration will be used
 if (numberOfDevices_DS > 0) {// check number of DS devices found
    dsSensoramount=numberOfDevices_DS;
    //set precision for all found devices
    for(byte i=0; i < numberOfDevices_DS; i++) {
    sensors.getAddress(tempDeviceAddress, (i));
    sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
    delay (10); //wait for seting - can be increased for many devices
    //Sleepy::loseSomeTime(100);  //it causes hang of processor it is replaced in baterry only set WDT is used there in this place it should be not nesscessary
    }
    sensors.setWaitForConversion(false); //being tested asynhrounous reading -  needed to be checked for faster reading temperature test required
    sensors.requestTemperatures();
    
    
     //Sleepy::loseSomeTime(1500);  //can be increased when amount of devices ic huge - it causes hang of processor it is replaced in baterry only set WDT is used there in this place it should be not nesscessary
  
   
    #ifndef DS_BY_ADDRESS_GETTING   
    
    for(byte i=0; i < numberOfDevices_DS; i++) {
        float temp = sensors.getTempCByIndex(i);
        ds_array[i] = temp * 100;
        measure.temp_DS[i] = ds_array[i];
      
      }
    #endif

    #if defined DS_BY_ADDRESS_GETTING  //set DeviceAddress Address_DS[n] if you want to use that, uncoment  addresses in the "set parameters for DS18B20" and DS_BY_ADDRESS_GETTING in config
       
      for(byte i=0; i < numberOfDevices_DS; i++) {
        oneWire.search(Address_DS[i]);
        float temp = sensors.getTempC(Address_DS[i]);
        ds_array[i] = temp * 100;
        measure.temp_DS[i] = ds_array[i];
      }
    #endif
 }

#endif

//set DS main temperature, default DS sensor 0, you can change it to any

#if defined DS_AS_MAIN_TEMP && defined DS_COUNT && DS_COUNT > 0
measure.temp = measure.temp_DS[0]*0.1;
#endif

#if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING && !defined  DHT_SENSOR && !defined SHT21_SENSOR
 
 measure.temp = readTemp()/1000;
 
#endif

}


#if defined NSENSORS && (NSENSORS == 1 || NSENSORS == 3)
// powermon colection values main function
static void doPowermon () {
 
  
  delay (2);
  // start when emon defined
 
  lastSampleTime_kwh = millis();     // Store time for kwh calc
  realPowersumcollect = 0;
  analogReference(DEFAULT); //set proper sample reference voltage
  
  for (byte i=0; i<NSENSORS; i++) {
    // perform measurements of emon
    emon[i].calcVI(16,2000);    // 2 h.c. for buffering + 14 h.c. for measuring
/*
  Serial.println();
  Serial.println("Print out all variables (realpower, apparent power, Vrms, Irms, power factor)");
  emon[i].serialprint();
  Serial.println();
*/ 
    // convert and pack data for emon
    
    //measure.sensor[i].sensorno = (i);
    //delay (10);
    measure.sensor[i].Irms = emon[i].Irms*100;
    //delay (2);
    measure.sensor[i].realPower = emon[i].realPower;
    //delay (2);
    //measure.sensor[i].apparentPower = emon[i].apparentPower;
    measure.sensor[i].powerFactor = emon[i].powerFactor*100;
    //delay (2);
    realPowersumcollect += measure.sensor[i].realPower;
    
  }
  // Voltage sensor is connected to 1st phase line where CT1 is connected - pin A0 / P1 / (0)
  measure.Voltage = ((float)emon[0].Vrms)*100;
  //realPowersum = realPowersumcollect;
  

   //Sleepy::loseSomeTime(2000); //not needed it can hang controler - potentialy need to be set when battery only mode is used

}
#endif

//code relay function
#if defined RELAY_AMOUNT && !defined BATTERY_ONLY && defined STRUCT_RELAY_STATUS  && EXPANDERS_AMOUNT && NSENSORS < 2 

byte coderelayStatus() {
//*to modify - bad library - change function

// create expander status with the coder there are 1 bits for amount of relays and 12 for status for each relay
//http://starter-kit.nettigo.pl/2011/11/pcf8574-czyli-jak-latwo-zwiekszyc-liczbe-pinow-w-arduino/
// prepare start parameter
 measure.relaystatus =0;

//code of amount
measure.relaystatus = RELAY_AMOUNT * 4096;


#ifdef DEBUG
//initialize expanders set addresses for expander/expanders 
Serial.print("Relays amount: |");
Serial.print(RELAY_AMOUNT);
delay (2);

Serial.print("| Expanders ammount: |");Serial.print(EXPANDERS_AMOUNT);
delay (2);

Serial.print("| Coded amount of relays: ");Serial.println(measure.relaystatus);
delay (2);
#endif     


  delay(5);
 
  Serial.println("PCF8574 real value is opposite logic value I/O");
  
  
  delay(5);
  
  for (byte i=0; i< RELAY_AMOUNT; i++) {
    delay(50);
// it can be changed when 1 expander has different amount of relays - each case in switch should be  set accordingly
// read each pin for particular expander and get code for sending	    


  //expander 1
  #if defined ADDRESS_0 && EXPANDERS_AMOUNT > 0	
   switch (i) {	 
	          case 0:
		        RelayValue[i] = ((EXPANDER0.digitalRead(0) == 0? 1:0));   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]);
                        delay(20);
//== 1? 0:1);           
		  break;
		    
		  case 1:
		        RelayValue[i]= ((EXPANDER0.digitalRead(1) == 0? 1:0));   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]);
                        delay(20);
//== 1? 0:1);
		  break;
		  case 2:
		  	RelayValue[i] = ((EXPANDER0.digitalRead(2)== 0? 1:0));   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]);
                        delay(20);
//== 1? 0:1);
		  break;
		 }
 #endif 
 /*
  // expander 2
#if defined ADDRESS_0 && defined ADDRESS_1 && EXPANDERS_AMOUNT > 1 
  switch (i) {	 
	          case 3:
		        RelayValue[i] = ((EXPANDER1.digitalRead(3) == 0? 1:0));   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]);
                        delay(3);
//== 1? 0:1);
		  break;
		    
		  case 4:
		        RelayValue[i]= ((EXPANDER1.digitalRead(4) == 0? 1:0));   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]);
                        delay(3);
//== 1? 0:1);
		  break;
		  case 5:
		  	RelayValue[i] = ((EXPANDER1.digitalRead(5)== 0? 1:0));   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]);
                        delay(3);
//== 1? 0:1);
		  break;
		 }
 #endif


//expander 3

  #if defined ADDRESS_0 && defined ADDRESS_1 && defined ADDRESS_2 && EXPANDERS_AMOUNT > 2	

   switch (i) {	 
	          case 6:
		        RelayValue[i] = EXPANDER2.digitalRead(0);   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]== 1? 0:1);
		  break;
		    
		  case 7:
		        RelayValue[i]= EXPANDER2.digitalRead(1);   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]== 1? 0:1);
		  break;
		  case 8:
		  	RelayValue[i] = EXPANDER2.digitalRead(2);   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]== 1? 0:1);
		  break;
		 }
  #endif

//expander 4
   #if defined ADDRESS_0 && defined ADDRESS_1 && defined ADDRESS_2 && defined ADDRESS_3 && EXPANDERS_AMOUNT  > 3	

   switch (i) {	 
	          case 9:
		        RelayValue[i] = EXPANDER3.digitalRead(0);   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]== 1? 0:1);
		  break;
		    
		  case 10:
		        RelayValue[i]= EXPANDER3.digitalRead(1);   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]== 1? 0:1);
		  break;
		  case 11:
		  	RelayValue[i] = EXPANDER3.digitalRead(2);   //check for proper expander I/O pins
			Serial.print(" Relay |");Serial.print(i+1);Serial.print("| Relay value: ");Serial.println(RelayValue[i]== 1? 0:1);
		  break;
		 }

  #endif

*/

//code of relays status
   
    //count_relaystatus will code status of  relays

    
    measure.relaystatus+= (RelayValue[i]==0) ? (pow(2,(i))):0;
   delay(2);
  // #ifdef DEBUG
  // Serial.print(" Binary value sum collected: ");Serial.println(count_relaystatus,BIN);
  // #endif
 }


  #ifdef DEBUG
  Serial.print(" Coded relays amount + statuses: ");
  Serial.println(measure.relaystatus);
  delay (2);


  #endif
return 1;
 }
#endif


#if defined ADDRESS_0 && EXPANDERS_AMOUNT > 0	 //for EXPANDER_0 switch setting functions
// relays interput functions for switches (add similar section for more expanders)

// PCF interput function
void onInterrupt_EXPANDER0() {
  delay (10);
#ifdef DEBUG
  Serial.println("PCF interput IRQ PIN5");
#endif
//set expander number set - EXPANDER0
  EXPANDER0.checkForInterrupt();
  delay (20);
}


void P1_onPin4_EXPANDER0() {
//EXPANDER0  
  delay (5);
#ifdef DEBUG  
  Serial.println("Switch for P1 - Pin 4 received");
#endif   

   // static unsigned long lastMillis = 0;   //function for switch unstable status - usualy used for atmega for direct attaching
   // unsigned long newMillis = millis();
   //if (newMillis - lastMillis <50) { 
   //Serial.println("migotanie");
   //} else {
      if (EXPANDER0.digitalRead(0) == 0) {
      delay(3);
        // jesli tak to zapala diode LED
      // expander.digitalWrite(0, 1);
      EXPANDER0.toggle(0);
      } else  {
      // jesli nie to wylacza
      //expander.digitalWrite(0, 0);
      delay(3);
      EXPANDER0.toggle(0);
      }
      // lastMillis=newMillis;
      // }
coderelay = coderelayStatus();
delay (5);
coderelayStatusConfirmation();
delay(20);

}

void P2_onPin5_EXPANDER0() {
//EXPANDER0
delay (5);
#ifdef DEBUG
  Serial.println("Switch for P2 - Pin 5 received");
#endif 
 // static unsigned long lastMillis = 0;
 // unsigned long newMillis = millis();
 // if (newMillis - lastMillis <50) { 
   //Serial.println("migotanie");
   //} else {
      if (EXPANDER0.digitalRead(1) == 0) {
      delay(3);
        // jesli tak to zapala diode LED
      //expander.digitalWrite(1, 1);
      EXPANDER0.toggle(1);
      } else  {
      delay(3);
        // jesli nie to wylacza
      //expander.digitalWrite(1, 0);
      EXPANDER0.toggle(1);
      }
      //lastMillis=newMillis;
      // }
coderelay = coderelayStatus();
delay (5);
coderelayStatusConfirmation();
delay(20);

}

void P3_onPin6_EXPANDER0(){
//EXPANDER0
  delay (5); 
#ifdef DEBUG
  Serial.println("Switch for P3 - Pin 6 received");
#endif
   // static unsigned long lastMillis = 0;
   // unsigned long newMillis = millis();
   //if (newMillis - lastMillis <50) { 
   //Serial.println("migotanie");
   //} else {
      if (EXPANDER0.digitalRead(2) == 0) {
      delay(3);
        // jesli tak to zapala diode LED
      //expander.digitalWrite(2, 1);
      EXPANDER0.toggle(2);
      } else  {
      delay(3);
        // jesli nie to wylacza
      //expander.digitalWrite(2, 0);
      EXPANDER0.toggle(2);
      }
     // lastMillis=newMillis;
     // }
coderelay = coderelayStatus();
delay (5);
coderelayStatusConfirmation();
delay(20);
}

#endif 




static void receiveRXrelayPayload () {


#ifdef DEBUG

#ifdef RELAY_AMOUNT

if (rxdata.destnode == NODEID && rxdata.state == 1 || rxdata.state ==0 ) 
{
Serial.println("Proccessing commands received for relays. ");
} else {
  
   Serial.println("Nothing to process after data receiving. ");
} 
#endif 

#if defined PULSE_MEASURE

if (rxdata.destnode == NODEID && rxdata.receivedvalue != 0) 
 {
 Serial.println("Proccessing commands received for pulse value. ");
 }else{
 Serial.println("Nothing to process after data receiving. ");
 }

#endif 

#endif

#ifdef RELAY_AMOUNT 

  #ifdef ADDRESS_0
  //proccessing commands for node relay  //do zmiany na expanderach docelowyc
 if (rxdata.destnode == NODEID) {
   //Serial.println("czy ja tu bylem ?");
   if (rxdata.cmd == 1) {   //convert from binary
    // process cmd //

    //check command received from base
    switch (rxdata.state) {
     case 0:
     EXPANDER0.digitalWrite(0, HIGH);
     //P1_onPin4_EXPANDER0(); //test
     delay(10);
     Serial.println("EXECUTE RELAY 1");
     break;
     case 1:
     EXPANDER0.digitalWrite(0, LOW);
      //P1_onPin4_EXPANDER0();  //test
     delay(10); 
     Serial.println("EXECUTE RELAY 1");
     break;

    }
   } else if (rxdata.cmd == 2) {
// process cmd 
    switch (rxdata.state) {
     case 0:
     EXPANDER0.digitalWrite(1, HIGH);
     delay(10);
	 Serial.println("EXECUTE RELAY 2");
     break;
     case 1:
     EXPANDER0.digitalWrite(1, LOW);
     delay(10);
     Serial.println("EXECUTE RELAY 2");
     break;

    }
   
   } else if (rxdata.cmd == 3) {
  // process cmd 
    switch (rxdata.state) {
     case 0:
     EXPANDER0.digitalWrite(2, HIGH);
     delay(10);
     Serial.println("EXECUTE RELAY 3");
     
     break;
     case 1:
     EXPANDER0.digitalWrite(2, LOW);
     delay(10);
     Serial.println("EXECUTE RELAY 3");
     break;

    }
   } else {
     
     Serial.println("Nothing to do");
   }
  }


  #endif

/*
  //uncoment it and test when more expanders will be attached

#ifdef ADDRESS_1
//proccessing commands for node relay  //do zmiany na expanderach docelowyc
 if (rxdata.destnode == NODEID) {
//Serial.println("czy ja tu bylem ?");
   if (rxdata.cmd == 4) {   //convert from binary
    // process cmd 

    //check command received from base
    switch (rxdata.state) {
     case 0:
     EXPANDER1.digitalWrite(0, HIGH);
     Serial.println("EXECUTE RELAY 4");
     break;
     case 1:
     EXPANDER1.digitalWrite(0, LOW);
      Serial.println("EXECUTE RELAY 4");
     break;
    
    }
   } else if (rxdata.cmd == 5) {
// process cmd 
    switch (rxdata.state) {
     case 0:
     EXPANDER1.digitalWrite(1, HIGH);
     Serial.println("EXECUTE RELAY 5");
     break;
     case 1:
     EXPANDER1.digitalWrite(1, LOW);
     Serial.println("EXECUTE RELAY 5");
     break;
     
    }
   
   } else if (rxdata.cmd == 6) {
  // process cmd 
    switch (rxdata.state) {
     case 0:
     EXPANDER1.digitalWrite(2, HIGH);
     Serial.println("EXECUTE RELAY 6");
     break;
     case 1:
     EXPANDER1.digitalWrite(2, LOW);
     Serial.println("EXECUTE RELAY 6");
     break;
    
    }
   }
  }
#endif
#ifdef ADDRESS_2
//proccessing commands for node relay  //do zmiany na expanderach docelowyc
 if (rxdata.destnode == NODEID) {
//Serial.println("czy ja tu bylem ?");
   if (rxdata.cmd == 7) {   //convert from binary
    // process cmd 

    //check command received from base
    switch (rxdata.state) {
     case 0:
     EXPANDER2.digitalWrite(0, HIGH);
     Serial.println("EXECUTE RELAY 7");
     break;
     case 1:
     EXPANDER2.digitalWrite(0, LOW);
      Serial.println("EXECUTE RELAY 7");
     break;

    }
   } else if (rxdata.cmd == 8) {
// process cmd 
    switch (rxdata.state) {
     case 0:
     EXPANDER2.digitalWrite(1, HIGH);
     Serial.println("EXECUTE RELAY 8");
     break;
     case 1:
     EXPANDER2.digitalWrite(1, LOW);
     Serial.println("EXECUTE RELAY 8");
     break;

    }
   
   } else if (rxdata.cmd == 9) {
  // process cmd 
    switch (rxdata.state) {
     case 0:
     EXPANDER2.digitalWrite(2, HIGH);
     Serial.println("EXECUTE RELAY 9");
     break;
     case 1:
     EXPANDER2.digitalWrite(2, LOW);
     Serial.println("EXECUTE RELAY 9");
     break;

    }
   }
  }
#endif
#ifdef ADDRESS_3
//proccessing commands for node relay  //do zmiany na expanderach docelowyc
 if (rxdata.destnode == NODEID) {
//Serial.println("czy ja tu bylem ?");
   if (rxdata.cmd == 10) {   //convert from binary
    // process cmd 

    //check command received from base
    switch (rxdata.state) {
     case 0:
     EXPANDER3.digitalWrite(0, HIGH);
     Serial.println("EXECUTE RELAY 10");
     break;
     case 1:
     EXPANDER3.digitalWrite(0, LOW);
      Serial.println("EXECUTE RELAY 10");
     break;

    }
   } else if (rxdata.cmd == 11) {
// process cmd 
    switch (rxdata.state) {
     case 0:
     EXPANDER3.digitalWrite(1, HIGH);
     Serial.println("EXECUTE RELAY 11");
     break;
     case 1:
     EXPANDER3.digitalWrite(1, LOW);
     Serial.println("EXECUTE RELAY 11");
     break;

    }
   
   } else if (rxdata.cmd == 12) {
  // process cmd 
    switch (rxdata.state) {
     case 0:
     EXPANDER3.digitalWrite(2, HIGH);
     Serial.println("EXECUTE RELAY 12");
     break;
     case 1:
     EXPANDER3.digitalWrite(2, LOW);
     Serial.println("EXECUTE RELAY 12");
     break;

    }
   }
  }
#endif
*/
#endif   //end if RELAY_AMOUNT

#ifdef PULSE_MEASURE

if (rxdata.destnode == NODEID && rxdata.receivedvalue != 0) {
 measure.pulse=rxdata.receivedvalue;
 
 delay (2);
 Serial.println("VALUES FROM BASENODE SET FOR PULSE VALUE: ");
 delay (2);
 Serial.print(measure.pulse);
 delay (2);
 
 }
#endif 
}





static void CheckIfdataCome () {
  
  //#ifdef DEBUG
    Serial.println();
    Serial.print("RX:");
    Serial.print("|");
    Serial.print((int) rf12_hdr);
    for (byte i = 0; i < rf12_len; ++i) {
    Serial.print("|");
    Serial.print((int) rf12_data[i]); //data print binary
    }
    
    Serial.println("|  Receiving data from basenode to process for node: ");
    
    Serial.println((int) rf12_data[0]); //data print binary
    //Serial.println(rf12_len);
 //#endif 
}







#if defined PULSE_MEASURE || defined RELAY_AMOUNT  //curently for relays and pulse - add next if you want get data from basenode
//report status of relays or pylse values received  after receiving command for change state
void coderelayStatusConfirmation(){
   
//#if !defined RELAY_AMOUNT  
   #if defined BATTERY_ONLY 
   rf12_sleep(RF12_WAKEUP);
   delay(2);
   #endif
   rf12_recvDone();
   //rf12_sendWait(2);

byte var = 0;

//while(var < 2){
          
  if (rf12_canSend()) {
               rf12_sendStart(BASEDNODE, &measure, sizeof measure);  //sync mode parameter 1 last parameter set
               delay (2);
               //rf12_sendWait(1);
                   if (RF12_WANTS_ACK) {
                    rf12_sendStart(RF12_ACK_REPLY,0,0);
                    //rf12_sendWait(1);
                    }
            }
         // var++;
    //} 
//rf12_sendWait(RADIO_SYNC_MODE);
}

#endif



//sending/printing  process function 
static void sendPayload() {

//rf12_sleep(RF12_WAKEUP);   //wake up RF module  - need to be measured for current consumption by multimeter if it is required amd sending was mising 


#ifdef LED_ON
  activityLed(1);
#endif


#if defined RELAY_AMOUNT && !defined BATTERY_ONLY && defined STRUCT_RELAY_STATUS && defined EXPANDERS_AMOUNT && NSENSORS < 3
 coderelay = coderelayStatus();
 delay (2);
#endif

#if defined STRUCT_CONTROL_BYTE 
  setControlbits(); //set control bits function run
#endif 
  
#if defined NSENSORS && (NSENSORS == 1 || NSENSORS == 3)
  doPowermon (); //do power measure
  delay (2);
#endif

  doMeasure(); // measure other sensors

#ifdef DEBUG
  delay (2);
  transmissionRS();  //printout data to serial
#endif

#ifdef TANK_LEVEL

TankLevel ();

#endif

#ifdef LED_ON
  activityLed(0);
#endif



}

//function for received data
// setd and receive for node see http://jeelabs.org/2011/05/07/rf12-skeleton-sketch/#comments
// when something will be received for node
static void consumeInData () {

#ifdef DEBUG
  CheckIfdataCome ();
#endif

receiveRXrelayPayload ();
}

//function for sending data
// data send for some period time
static byte produceOutData () {

#if defined BATTERY_ONLY 
  unsigned int vcc = battVolts();
  if (vcc <= VCC_FINAL) { // last data payload to be send - after that pray for lost data :-)

    sendPayload();
    vcc = 1; // don't even try reading VCC after this send
  }
  if (vcc >= VCC_OK) { // battery is ok
    
    sendPayload();
  }
  int minutes = VCC_SLEEP_MINS(vcc);
  while (minutes-- > 0);

  return 1;
#else

sendPayload();
return 1;
#endif

}

#if defined PULSE_MEASURE
//used for pulse measure 
void onPulse() {
    
    
    unsigned long newBlink = micros();  
    unsigned long interval = newBlink-lastBlink;
       
       if (interval<500000L) { // Sometimes we get interrupt on RISING
         //flag_pulse = 0;
         return;
       } else {
                    
          pulseCount++;           //pulseCounter 
          measure.pulse++;       //used for pulse measure 
         
          #ifdef DEBUG
            Serial.print("PULSE COUNT GET: "); Serial.println(pulseCount);
            delay(1);
            Serial.print("measure: "); Serial.println(measure.pulse);
          #endif
          
          flag_pulse = 1;

       }
       
       lastBlink = newBlink;  
}

#endif


//internal temp measure

 #if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING && !defined  DHT_SENSOR && !defined SHT21_SENSOR

  long readTemp() {
  long result; // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3); 
  delay(20); // Wait for Vref to settle - 2 was inadequate
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = ((result - 125) * 1075)-30000;; //Don't forget to change "x" with multiplication sign
  return result; 
}

#endif


#if defined TANK_LEVEL


//function to print Tank level in converter 
static void TankLevel () {

measure.tanklevel=0;

 //Sleepy::loseSomeTime(60);
 delay (5);
byte low=digitalRead(low_level_digi_pin);
delay(2);
byte medium=digitalRead(medium_level_digi_pin);
delay(2);
byte full=digitalRead(high_level_digi_pin);
delay(2);
measure.tanklevel=low+medium+full;


  #ifdef DEBUG
  Serial.println();
  Serial.print("TANK LEVEL: ");
 
   switch (measure.tanklevel) {
    case 0:
      //do something when var equals 1
      Serial.print("FULL"); 
      //Serial.print("|");
     
      
      //add for additional digital pin output for control other stuff
      break;
    case 1:
      Serial.print("MEDIUM"); 
      //Serial.print("|");
      
      
      //add for additional digital pin output for control other stuff
      break;
    case 2:
      Serial.print("LOW"); 
      //Serial.print("|");
      //add for additional digital pin output for control other stuff
      break;
    case 3:
      Serial.print("ERR"); //sikac do wiaderka
      //Serial.print("|");
      //add for additional digital pin output for control other stuff
      break;
    default: 
     Serial.print("NO MEASURE IMPLEMENTED"); //sikac do wiaderka albo padl czujnik
     //Serial.print("|");
     
   }    
  #endif
} 
#endif



// Main loop
void loop() {
  //long czas1 = millis();  // sending timer


//memset (&measure,0,sizeof (measure));
//memset (&rxdata,0,sizeof (rxdata));
//waiting for comands (can be extended in receiving structure)
#if !defined BATTERY_ONLY

   //waiting for comands (can be extended in receiving structure)
   //for (byte i=0;i<10;i++){   // loop for to be sure that proper package can be received  WAIT_TIME_MULTIPLY
  #if NODEID !=10
    if (rf12_recvDone() && rf12_crc == 0 && ((int) (rf12_hdr & 0x1F)) == BASEDNODE && rf12_len == sizeof rxdata) {
       

       if (((int) rf12_data[0]) != NODEID) return;
       
       memcpy(&rxdata, (void*) rf12_data, sizeof rxdata); 
        //rf12_sendWait(1);  
       if (RF12_WANTS_ACK) {
            rf12_sendStart(RF12_ACK_REPLY,0,0);
            //rf12_sendWait(1);
           }

       consumeInData ();
       delay (5);
       #if defined RELAY_AMOUNT
       coderelay = coderelayStatus();
       delay (5);
       #endif
       #if defined RELAY_AMOUNT || defined PULSE_MEASURE
       coderelayStatusConfirmation(); //coment it for basenode
       //pendingOutput = produceOutData();
       #endif
              
       
      }
     

     
     
    #endif





       
    #if defined DEBUG && NODEID != 10
    
    if (sendTimer.poll((10000 * WAIT_TIME_MULTIPLY))) {// period sending set multiply by wait_time value ~ 3 sek procesing all and send 10 sek is a default multiply 1 time betwen sending measures change WAIT_TIME_MULTIPLY for multiply time 
       
       #ifdef PULSE_MEASURE
              #ifdef DEBUG
               Serial.print("Sending phase, if pulse not 0, curently pulse is: "); delay(10);Serial.println(flag_pulse);
              #endif
         if (pulseCount > 0 && flag_pulse == 1) {
   
         
         pendingOutput = produceOutData();
         pulseCount = 0;
         flag_pulse = 0; 
   
         }
       #else 
   
       
       pendingOutput = produceOutData();
   
       #endif
    }
    
    #endif
    #if !defined DEBUG && NODEID !=10
    

    
    
// WAIT_TIME_MULTIPLY gives multiply 1 min , wheret there is combination 20s up and waiting for commands from base and 40s in sleep mode 
    if (sendTimer.poll((10000 * WAIT_TIME_MULTIPLY - delayed ))) {
    

//sleep mode without relays
     #if !defined RELAY_AMOUNT


      for (byte i = 0; i < WAIT_TIME_MULTIPLY;i++) {  // maximum time delay is 60000 milis , it is multiply by WAIT_TIME_MULTIPLY sending duration

     //send data if there are some
                //1,7 uA power consumption
    			#ifndef RELAY_AMOUNT 
        			rf12_sleep(RF12_SLEEP)
           			delay(2);
        		#endif
                #if !defined PULSE_MEASURE 
                 long sometime;
                 if (WAIT_TIME_MULTIPLY >1) sometime = 50000;
                 if (i == (WAIT_TIME_MULTIPLY -1) ) sometime = 49600;
                  byte occurence =Sleepy::loseSomeTime(sometime);  //sleep mode
                  if (occurence == 0) { 
                      break;
                  }else{
                      continue;
                  }
    
                #else
    
                //byte occurence = 0; //comment for sleep mode
                Sleepy::powerDown(); 	// sometimes something break WDT Sleepy::loseSomeTime(60000);
                //if (occurence == 0) break;
                delay(1);
                if (pulseCount > 0 && flag_pulse != 0) break;      // exit the for(), we're done receiving pulse according INT 1 interput pulse received
                if (pulseCount == 0 && flag_pulse == 0 ) continue;  // if nothing hapened  // for sleep mode || occurence == 1
                             
               //check if comething is to process through radio INT 0 interput received
        #ifndef RELAY_AMOUNT 
        rf12_sleep(RF12_WAKEUP);
        
        delay(2);
        #endif
                     if (rf12_recvDone() && rf12_crc == 0 && ((int) (rf12_hdr & 0x1F)) == BASEDNODE && rf12_len == sizeof rxdata) { //&& rf12_len == sizeof rxdata
                          
                        
                          if (((int) rf12_data[0]) != NODEID) return;
                          
                          memcpy(&rxdata, (void*) rf12_data, sizeof rxdata ); 
                          //rf12_sendWait(1);  
                              if (RF12_WANTS_ACK) {
                                  rf12_sendStart(RF12_ACK_REPLY,0,0);
                                  //rf12_sendWait(1);
                              }
                       
                       
                       consumeInData ();
                       delay (5);
                       #if defined RELAY_AMOUNT
                       coderelay = coderelayStatus();
                       delay (5);
                       #endif
                       #if defined RELAY_AMOUNT || defined PULSE_MEASURE
                       coderelayStatusConfirmation(); //coment it for basenode
                       pendingOutput = produceOutData();
                       #endif
                       
                     }
                       

                       
                       #ifdef DEBUG
                        Serial.print("pulseCount: "); delay(10);Serial.println(pulseCount);
                        delay(10);
                        Serial.print("flag_pulse: "); delay(10);Serial.println(flag_pulse);
                        delay(10); 
                       #endif
    
 
               #endif 
   
       }
     #endif
//end sleep mode without relays

//for pulse measure produce data
      #if defined PULSE_MEASURE
        #ifdef DEBUG
          Serial.print("Sleep break after get some interput, pulse is: "); delay(10);Serial.println(flag_pulse);
        #endif
       if (pulseCount > 0 && flag_pulse == 1) {
        #ifndef RELAY_AMOUNT 
        rf12_sleep(RF12_WAKEUP);
        
        delay(2);
        #endif
         pendingOutput = produceOutData();
         pulseCount = 0;
         flag_pulse = 0; 
       }
                       
      #else 
   
        #ifndef RELAY_AMOUNT 
        rf12_sleep(RF12_WAKEUP);
        
        delay(2);
        #endif

       pendingOutput = produceOutData();
   
      #endif
   
    }
  
   #endif 


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//for nodes without node 10
  #if NODEID == 10
 
  #if defined DEBUG
   
  for (byte i = 0; i < WAIT_TIME_MULTIPLY;i++){  // maximum time delay is 60000 milis , it is multiply by WAIT_TIME_MULTIPLY sending duration
         
         //send data if there are some
           /*
           long sometime;
           if (WAIT_TIME_MULTIPLY >1) sometime = 60000;
           if (i == (WAIT_TIME_MULTIPLY -1) ) sometime = 47645;
           rf12_sleep(RF12_SLEEP); //1,7 uA power consumption
                 byte occurence =Sleepy::loseSomeTime(47110);  //sleep mode
                  if (occurence == 0) { 
                      break;
                  }else{
                      continue;
                  }
           */
       }
   
   #else
   
   for (byte i = 0; i < WAIT_TIME_MULTIPLY;i++){  // maximum time delay is 60000 milis , it is multiply by WAIT_TIME_MULTIPLY sending duration
         //send data if there are some
           
                 rf12_sleep(RF12_SLEEP); //1,7 uA power consumption
                 long sometime;
                 if (WAIT_TIME_MULTIPLY >=1) sometime = 60000;
                 //if (i == (WAIT_TIME_MULTIPLY -1) ) sometime = 47645;
                 byte occurence =Sleepy::loseSomeTime(sometime);  //sleep mode 1 min with measure
                  if (occurence == 0) { 
                      break;
                  }else{
                      continue;
                  }
       }
   #endif
 
        
        #ifdef DEBUG
        Serial.println("Sleep break ");
        Serial.println();
        #endif
        rf12_sleep(RF12_WAKEUP);
        delay(2);
        produceOutData();
         rf12_recvDone;
        #ifndef DEV_MODE
        doReport();
        #endif
       
       #ifdef LED_ON
        activityLed(0);
       #endif

  #endif

//Sending data periodicaly - for 0.3 sek not be able to get commands 


   #ifndef DEV_MODE   //STOP SENDING DATA 
    //rf12_recvDone;
    if (pendingOutput && rf12_canSend()) {
       
      pendingOutput = 0;
       
  
   //while (!rf12_canSend())
   rf12_sendStart(BASEDNODE, &measure, sizeof measure);  //sync mode parameter 1 last parameter set
   delay (2);
          if (RF12_WANTS_ACK) {
            rf12_sendStart(RF12_ACK_REPLY,0,0);
            delay (1);
          } else {
            if (rf12_canSend()){
            rf12_sendStart (BASEDNODE, &measure, sizeof measure); //if not requested ACK, the package retransmited for to be sure that data can be received
            delay (1);
            }
          #ifdef SECONDNODE
            if (rf12_canSend()){
            rf12_sendStart(SECONDNODE, &measure, sizeof measure);  //second base sending
            delay (1);
            }
          #endif
          }
      
      
       #ifdef LED_ON
        activityLed(0);
       #endif
       
       //long czas2 = millis(); //sending timer

       //Serial.println();Serial.print((float)(czas2 - czas1)*0.001,2);Serial.println(); //sending timer
    }
  #endif


#endif 
//end standard mode not for BATTERY

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>     
//THERE IS FOR BATTERY ONLY - save energy
#if defined BATTERY_ONLY && !defined RELAY_AMOUNT


  //#ifdef PULSE_MEASURE 
  //rf12_control(0x8280);
  if (rf12_recvDone() && rf12_crc == 0 && ((int) (rf12_hdr & 0x1F)) == BASEDNODE && rf12_len == sizeof rxdata) { //&& rf12_len == sizeof rxdata
        

        if (((int) rf12_data[0]) != NODEID) return;
        
        memcpy(&rxdata, (void*) rf12_data, sizeof rxdata ); 
        delay (1);
        //rf12_sendWait(1);  
          if (RF12_WANTS_ACK) {
            rf12_sendStart(RF12_ACK_REPLY,0,0);
            //rf12_sendWait(1);
          }

       
       consumeInData (); 
       delay (10);
       #if defined RELAY_AMOUNT
       coderelayStatus();
       //coderelay = 0;
       delay (10);
       #endif
       #if defined RELAY_AMOUNT || defined PULSE_MEASURE
       coderelayStatusConfirmation(); //coment it for basenode
       #endif
       pendingOutput = produceOutData();
       
   }
   

   
  //#endif    


  if (sendTimer.poll(20000)) {
    

    
    for (byte i = 0; i < WAIT_TIME_MULTIPLY;i++) {  // maximum time delay is 60000 milis , it is multiply by WAIT_TIME_MULTIPLY sending duration
     //send data if there are some
    rf12_sleep(RF12_SLEEP); //1,7 uA power consumption
    //send_when_battery ;
    
    #ifndef PULSE_MEASURE 
    long sometime;
    if (WAIT_TIME_MULTIPLY >=1) sometime = 40000;
    //if (i == (WAIT_TIME_MULTIPLY -1) ) sometime = 39600;
    byte occurence =Sleepy::loseSomeTime(sometime);  //sleep mode
    if (occurence == 0) { 
      break;
    }else{
      continue;
    }
    
    #else
    
    //byte occurence = 0; //comment for sleep mode
    Sleepy::powerDown(); 	// sometimes something break WDT Sleepy::loseSomeTime(60000);
    //if (occurence == 0) break;
    delay(1);
    if (pulseCount > 0 || flag_pulse != 0) break;      // exit the for(), we're done receiving pulse according INT 1 interput pulse received
    if (pulseCount == 0 || flag_pulse == 0 ) continue;  // if nothing hapened  // for sleep mode || occurence == 1
     //check if comething is to process through radio INT 0 interput received
     rf12_sleep(RF12_WAKEUP);
     delay(2);
     if (rf12_recvDone() && rf12_crc == 0 && ((int) (rf12_hdr & 0x1F)) == BASEDNODE && rf12_len == sizeof rxdata) { //&& rf12_len == sizeof rxdata
        
       
       
        
        if (((int) rf12_data[0]) != NODEID) return;
        
        memcpy(&rxdata, (void*) rf12_data, sizeof rxdata ); 
        delay (1);
        //rf12_sendWait(1);  
          if (RF12_WANTS_ACK) {
            rf12_sendStart(RF12_ACK_REPLY,0,0);
            //rf12_sendWait(1);
           }
       
       
       consumeInData (); 
       delay (10);
       #if defined RELAY_AMOUNT
       coderelayStatus();
       //coderelay = 0;
       delay (10);
       #endif
       #if defined RELAY_AMOUNT || defined PULSE_MEASURE
       coderelayStatusConfirmation(); //coment it for basenode
       
       #endif
       #if !defined RELAY_AMOUNT
       rf12_sleep(RF12_SLEEP); //1,7 uA power consumption
       #endif
   }   
    
    #ifdef DEBUG
    Serial.print("pulseCount: "); delay(10);Serial.println(pulseCount);
    delay(10);
    Serial.print("flag_pulse: "); delay(10);Serial.println(flag_pulse);
    delay(10); 
    
    #endif
    
  #endif  
    
   
   }


//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
   #ifdef PULSE_MEASURE
    #ifdef DEBUG
   Serial.print("Sleep break after get some interput, pulse is: "); delay(10);Serial.println(flag_pulse);
    #endif
   if (pulseCount > 0 && flag_pulse == 1) {
   
   rf12_sleep(RF12_WAKEUP);
   delay(2);
   pendingOutput = produceOutData();
   delay (10);
   pulseCount = 0;
   flag_pulse = 0; 
   
   }
   #else 
   
   
   rf12_sleep(RF12_WAKEUP);
   delay(2);
   pendingOutput = produceOutData();
   delay (10);
   #endif
  }





 //send data if there are some

if (pendingOutput && rf12_canSend() ) {
  
      pendingOutput = 0;
      //rf12_control(0x82FE);
      delay(5);
      doReport();   
      //long czas2 = millis(); //sending timer
      //Serial.println();Serial.print((float)(czas2 - czas1)*0.001,2);Serial.println(); //sending timer
}

#endif


//additional  info
  // see http://lowpowerlab.com/blog/2012/12/28/rfm12b-arduino-library/
  // http://openenergymonitor.org/emon/buildingblocks/rfm12b2
  // http://forum.arduino.cc/index.php?topic=192332.5;wap2
  // http://jeelabs.org/?s=wireless
  // Parameters calculator for setting radio http://tools.jeelabs.org/rfm12b.html
  // air upgrade see http://jeelabs.net/projects/jeeboot/wiki
  // send receive mode http://jeelabs.org/2011/05/07/rf12-skeleton-sketch/#comments
  
  //end sending in main loop


} //end main loop

//this function is simillar to loop and breaks as interput all operations when som data are typed in serial port

void serialEvent() {



      // kod do wykonania podczas odbioru danych
if (Serial.available() >=5  ) {  // should  be >5 characters - digits first is node id  and \n as last or node id xx and 0000 space and max value  4294967295 and \n 
     
#ifdef RELAY_AMOUNT
  
      
    rxdata.destnode = 0;
    rxdata.cmd = 0;
    rxdata.state = 0;
         
       /* RX 03121 | NID CMD STATE */
        //needToSend = 1; //uncoment it for basenode
     
    static char ramka[5]; //comment it for basenode
 
    int rxb = Serial.available();  //read values
     for (int i = 0; i<rxb; i++) {
       char test = Serial.read();
       
      #ifndef PULSE_MEASURE
      
      if (test == 13) break;   // exit the while(1), we're done receiving
       if (test == -1) continue;
      //Serial.flush();
      #endif
      
      ramka[i] = (test -48);
      // ramka[i] = (Serial.parseInt()-48); 
      //delay(5);
      
      }
      #ifndef PULSE_MEASURE
      Serial.flush();
      #endif
      //proccess relays data
    rxdata.destnode = (ramka[0] * 10) + ramka[1];
    rxdata.cmd = (ramka[2] * 10) + ramka[3];
    rxdata.state = ramka[4];
#endif

// data to send use node ID ex. 02 and 3 times zeros from the begining and space, after that type value to transmit to node (currently used for pulse)max value  4294967295 
 
#ifdef PULSE_MEASURE    
     char incomingByte;  // for data after getting data from serial 
     //unsigned long values = 0;
     measure.pulse = 0;
     while(1) {
        
         //incomingByte = Serial.read();
          incomingByte = Serial.read();
          if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
          if (incomingByte == 32) continue;  // if space read() 
          if (incomingByte == -1) continue; // if no characters are in the buffer read() returns -1
         measure.pulse *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
         measure.pulse = ((incomingByte -48) + measure.pulse);
         //values = ((incomingByte) + values);
    }
   
         //Serial.flush(); //clear serial data 
         rxdata.receivedvalue = measure.pulse;
          
delay (10);
#endif

 
 
    #ifdef RELAY_AMOUNT 
       Serial.println();
       Serial.print("SEND NODE ");
       Serial.print(rxdata.destnode);
       Serial.print(" RELAY ");
       Serial.print(rxdata.cmd);
       Serial.print(" STATE ");
       Serial.print(rxdata.state);
       Serial.print(' ');
   #endif  

   #ifdef PULSE_MEASURE   
       if (measure.pulse!= 0) { 
          Serial.println();
          delay (10);
          Serial.print("RECEIVED PULSE VALUE: ");
          delay (10);
          //Serial.print(values);
          Serial.print(' ');
          delay (10);
          Serial.print(rxdata.receivedvalue);
          delay (10);
          Serial.print(' ');
          delay (10);
       }
   #endif  
 
#if defined RELAY_AMOUNT
       receiveRXrelayPayload();
       coderelayStatus();
       //coderelay = 0;
       delay (10);
#endif

#if defined PULSE_MEASURE
doMeasure(); //coment it for basenode
#endif

#if defined RELAY_AMOUNT || defined PULSE_MEASURE
coderelayStatusConfirmation(); //coment it for basenode
#endif
    }   
 
}


