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









// lib for RFM12B from https://github.com/jcw/jeelib
#if RFM69 == 1
   #define RF69_COMPAT 1
#else
   #define RF69_COMPAT 0
#endif

#include <JeeLib.h>
// avr sleep instructions
//#include <avr/sleep.h>

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

#include "SENSSTRUCTPAYLOAD.h"
//txpower RF12b control variable
//set mili timer for sent package periodicaly and switch to receiving mode  //sending timer
MilliTimer sendTimer;   // see https://github.com/jcw/jeelib/blob/master/examples/RF12/timedSend/timedSend.ino

//add additional control timer variables
//static
volatile byte pendingOutput=0;

volatile word repeatwait = WAIT_TIME_MULTIPLY;  // multilpy control for extending wait for sending time



//bit control for sending post fixed data and decoding it in receiver side 
#ifdef STRUCT_CONTROL_BYTE

 byte dsSensoramount;
 byte phaseAmount;

#endif



//set debug for RS printing from node side
#ifdef DEV_MODE
#define DEBUG
#endif

#ifdef DEBUG
#define DEBUG_BAUD 9600
#endif

// settings, pin choice, voltage and curent sensors calibration, measurment delay
#if defined NSENSORS  
// Undefine sensors when 3 phase is set


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
If you are using the voltage input, with the load connected adjust the phase angle calibration 1.7 in lines like this const float phase_shift=                  1.7 – default so that real power and apparent power read the same value (and power factor is as close to 1.00 as possible). –resistance load need to be measured for that calibration. Your meter is not needed for this, there is small error to carry out of that. The phase calibration coefficient should not normally go outside the range 0.0 - 2.0 degr.  [my test gives 1.01 value].By modifying the software to report the time it takes to complete the inner measurement loop and the number of samples recorded, the time between samples was measured as 377 micros. This equates to 6.79 deg. (360deg. takes 20 ms). Therefore a value of 1 applies no correction, zero and 2 apply approximately 7 percent of correction in opposite directions. A value of 1.28 will correct the inherent software error of 2 percent that arises from the delay between sampling voltage and current.
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
#define LDR_PIN 6 //standard 2 nano 6 new 7/new 7
#define BAT_VOL 7 //standard 3 nano 7 /new 6
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

#ifdef TANK_LEVEL 

byte low_level_digi_pin = 5;
byte medium_level_digi_pin = 6;
byte high_level_digi_pin = 7;


#endif

//measure variables for sensmon




volatile unsigned int adcreading;  //for bang up voltage
int volts;

// ***********set parameters for DS18B20****************


// emon variables 
// Time (ms) to allow the filters to settle before sending data
#if defined NSENSORS && (NSENSORS == 1 || NSENSORS == 3)
#define FILTERSETTLETIME 10000 //used for complete sample getting for emon  
boolean settled = false;  //used for complete sample getting for emon
unsigned long lastTick = 0; //used for complete sample getting for emon
#ifdef DEBUG
unsigned long secTick;
int lastSampleTime_kwh, sampleDuration_kwh;
long lastmillis_kwh;   //time
#endif
float realPowersumcollect; //temp variable for summarizing power
//float realPowersum; //local variable for summarizing power
#endif 

//WDT set for uno and specyfic bootloaders - ned to be werified whether Watchdog working on
//ISR(WDT_vect) { Sleepy::watchdogEvent(); }


#ifdef NEW_REV
Port p1 (1); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p2 (2); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p3 (3); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p4 (4); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20

#endif



//***************************SETUP BEGIN*********************//


void setup() {

#ifdef LED_ON
pinMode(ACT_LED, OUTPUT);
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
rf12_control(0x985|(TXPOWER > 7 ? 7 : TXPOWER)); //power control 0 means max and maximum power consumption

#else

rf12_control(0x985|(TXPOWER > 7 ? 7 : TXPOWER)); //power control 0 means max and maximum power consumption
RF69::control(0x91, 0x9F); // Maximum TX power

#endif

//rf12_sleep(RF12_SLEEP);   //sleep RF module
//rf12_control(0x8280);
      rf12_recvDone();
      delay(20);
      rf12_sleep(RF12_SLEEP); 
      delay(100);
      
#ifdef TANK_LEVEL

pinMode(low_level_digi_pin, INPUT); 
pinMode(medium_level_digi_pin, INPUT);  
pinMode(high_level_digi_pin, INPUT);

#endif  

#ifdef DEBUG
  Serial.begin(DEBUG_BAUD);
#endif




//emon set parameters - calibration
#if defined NSENSORS && (NSENSORS == 1 || NSENSORS == 3)
  //set reference voltage 3.3 V
  //analogReference(DEFAULT);
  //emon parameters   - voltage is for phase 1 and is set for all phases       
  emon[0].voltage(VOLTAGE_PIN,VCAL, VCAL_CURENT_PHASE_SHIFT,1); //Voltage: input pin, calibration, phase_shift for phase 1
  emon[0].current(CURRENT_SENSOR1_PIN, CURRENT_SENSOR1_CALIBRATION);       // Current: input pin, calibration.
  #if NSENSORS == 3
  emon[1].voltage(VOLTAGE_PIN,VCAL,2.0,2); //Voltage: input pin, calibration, phase_shift for phase 2
  emon[1].current(CURRENT_SENSOR2_PIN, CURRENT_SENSOR2_CALIBRATION);       // Current: input pin, calibration.
  emon[2].voltage(VOLTAGE_PIN,VCAL,3.0,3); // Voltage: input pin, calibration, phase_shift for phase 3
  emon[2].current(CURRENT_SENSOR3_PIN, CURRENT_SENSOR3_CALIBRATION);       // Current: input pin, calibration.
  #endif 
#endif // closing ifdef POWERMON 



} //end setup



//Functions declaration

#if defined STRUCT_CONTROL_BYTE 
//set control bits function 
static void setControlbits() {

//coder for controlbits


//phaseAmount=0;

if (NSENSORS > 3) {
  phaseAmount=0; 
} else {
  phaseAmount = NSENSORS;
}
#endif

  dsSensoramount=0; 



measure.controlbits=(dsSensoramount * 16) + phaseAmount;  //sumaraise 2 digits to HEX 16 word wher 10ns are as a first 4 bits and phase as second 4 bits 

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



}
//#endif



static void doReport() {
  #ifdef LED_ON
   activityLed(1);
  #endif
 while (!rf12_canSend())    // wait until sending is allowed
       rf12_recvDone();      // process any incoming data (in the background)
      //if (rf12_canSend()) {
       delay(50);
               rf12_sendStart(BASEDNODE, &measure, sizeof measure);  //sync mode parameter 1 last parameter set
               //rf12_sendWait(RADIO_SYNC_MODE);
               rf12_recvDone(); 
               
              if (RF12_WANTS_ACK) {
              rf12_sendStart(RF12_ACK_REPLY,0,0);
              delay (2);
               //rf12_recvDone(); 
              
             }


    
   
  #ifdef SECONDNODE
       if (rf12_canSend()){
       rf12_sendStart(SECONDNODE, &measure, sizeof measure);
       //rf12_sendWait(RADIO_SYNC_MODE);
       rf12_recvDone(); 
            if (RF12_WANTS_ACK) {
                rf12_sendStart(RF12_ACK_REPLY,0,0);
                 //rf12_recvDone(); 
            }  
       

       }
  #endif

  
  #ifdef LED_ON
   activityLed(0);
  #endif
}
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 /* 
    static void doReport() {
    rf12_recvDone();
  delay(10);

  //if(rf12_canSend()){
    //rf12_recvDone();delay(10);};
  
  rf12_sendNow(BASEDNODE, &measure, sizeof measure); //send in sync mode last partametr 1
  rf12_sendWait(RADIO_SYNC_MODE);
  rf12_recvDone();
  delay (2);
  activityLed(1);
  //delay (2);
      if (RF12_WANTS_ACK) {
         if (rf12_canSend()){ 
         rf12_sendStart(RF12_ACK_REPLY,0,0);
         rf12_sendWait(RADIO_SYNC_MODE);
         rf12_recvDone();
         delay (2);
         }

     }


    //}
       #ifdef SECONDNODE
    
       //while (!rf12_canSend()){};
       rf12_sendNow(SECONDNODE, &measure, sizeof measure);
       rf12_sendWait(RADIO_SYNC_MODE);
       rf12_recvDone();
            if (RF12_WANTS_ACK) {
             if (rf12_canSend() ){ 
              rf12_sendStart(RF12_ACK_REPLY,0,0);
              rf12_sendWait(RADIO_SYNC_MODE);
              rf12_recvDone();
              delay (2);
             }
            }
            

          } 

     #endif
   activityLed(0);
  
//rf12_recvDone();





}
*/


//reference voltage bangup

long readVcc() {
  long result;
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5);   // Without this the function always returns -1 on the ATmega2560 http://openenergymonitor.org/emon/node/2253#comment-11432
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
  #endif
  #if defined(__AVR__) 
  delay(2); 
  ADCSRA |= _BV(ADSC);                             // Convert
  while (bit_is_set(ADCSRA,ADSC));
  //delay(2); 
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result;                     //1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
  return result ;
  #elif defined(__arm__)
  return (3300);                                  //Arduino Due
  #else 
  return (3300);                                  //Guess that other un-supported architectures will be running a 3.3V!
  #endif
  
  //-49
}
//battery control 
int battVolts(void) {

double vccref = 0.0;
unsigned int adcreading = 0;
 //analogReference(INTERNAL);

  
    readVcc();
    delay(10);
  
  vccref = readVcc()/1000.0;
 
  adcreading = analogRead(BAT_VOL) * 2;
  //vccref = readVcc()/1000.0;
  //adcreading = analogRead(BAT_VOL) * 2; 
  delay(10);
  double battvol = (adcreading / 1024.0) * vccref;
  return (battvol * 1000) + 72;
}




//LED activation
void activityLed (byte on) {
  digitalWrite(ACT_LED, on);
  delay(20);
}


//get values and set for transmission, DEBUG print to serial
static void transmissionRS() {
#ifdef DEV_MODE
  Serial.println("==DEV MODE==");
  delay(2);
#endif

  #ifdef DEBUG
  //activityLed(1);
  Serial.println(' ');
  delay(2);




  Serial.println();
  #if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING 
  Serial.print("TEMP ");
  #endif
  //internal temp *
  #if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING && !defined  DHT_SENSOR && !defined SHT21_SENSOR  
  Serial.print("INTERNAL ");
  #endif
  Serial.println((float)measure.temp*0.1,2);
  delay(2);
  
  Serial.print("LOBAT " );
  Serial.println(measure.lobat, DEC);
  delay(2);
  Serial.print("BATVOL ");
  Serial.println((float)measure.battvol/1000,3);
  delay(2);
  Serial.println(' ');
  delay(2);
         

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
  
  
   Serial.println("  Sample duration in seconds for counting kWh="); Serial.print(sampleDuration_kwh);; Serial.print(" sec. ");
   delay (2);
   Serial.println("  Power consumption="); Serial.print(float ((((realPowersumcollect) / 1000)/3600)* sampleDuration_kwh),4); Serial.print(" kWh ");  //aktual consumption kWh
   Serial.println();
   delay (2);
   Serial.println("  Stored count of seconds: "); Serial.print(sampleDuration_kwh);
   Serial.println(" sec.");
   delay (2);

   #endif

#endif
}

//get measure for sending
static void doMeasure() {       //main measure
  
  //count++;
  adcreading = 0;
  //measure.lobat = rf12_lowbat();  //probably if RF 69 it should be commented and measure should be changed 
  measure.lobat = 0;
  measure.battvol = battVolts();
  setControlbits();


#if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING && !defined  DHT_SENSOR && !defined SHT21_SENSOR
 
 measure.temp = readTemp()/1000;
 
#endif

}


#if defined NSENSORS && (NSENSORS == 1 || NSENSORS == 3)
// powermon colection values main function
static void doPowermon () {
  #ifdef DEBUG
  lastSampleTime_kwh = millis();     // Store time for kwh calc
  #endif
  realPowersumcollect = 0;
  //analogReference(DEFAULT); //set proper sample reference voltage
  
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
  #ifdef DEBUG
  // store count of seconds
   secTick = millis() / 1000;
  #endif
  //lastTick = millis();
  // start when emon defined
 

}
#endif







//sending/printing  process function 
static void sendPayload() {

    #ifdef LED_ON
    activityLed(1);
    #endif
   
    
    #ifdef TANK_LEVEL
    TankLevel ();
    #endif
    
    #if defined NSENSORS && (NSENSORS == 1 || NSENSORS == 3)
    doPowermon (); //do power measure
    #endif


    doMeasure(); // measure other sensors

    #ifdef DEBUG
    transmissionRS();  //printout data to serial
    #endif



    #ifdef LED_ON
    activityLed(0);
    #endif




}







//function for sending data
// data send for some period time
static byte produceOutData () {
  // Transmit data out
   sendPayload();
   // because millis() returns to zero after 50 days ! 
  if (!settled && millis() > FILTERSETTLETIME) settled = true;
    #ifdef DEBUG
    Serial.println("Millis correction for 50 days reset for sattled: "); Serial.print(settled);
    #endif
    
    if (settled) {
    //digitalWrite(LED, 1);
    
    // normally, loop time is about 1.3 sec,
    // so, let's wait for 2 sec interval to elapse before transmitting.
    // LED is a good indicator of wasted time here!
    #ifdef DEBUG

    while ( (((millis() -(10000 * WAIT_TIME_MULTIPLY))  -  lastTick )%4096) < 2000 ) {};
    #else
     while ( (((millis() -(60000 * WAIT_TIME_MULTIPLY))  -  lastTick )%4096) < 2000 ) {}; 
    #endif
    // remember this tick
     lastTick = millis();
    //digitalWrite(LED, 0);
    // start transmitting
    
    #ifdef DEBUG
    Serial.println("  Payload send tick >>>>>>>>: "); Serial.print(lastTick);
    Serial.print(" milisec");
    delay (50);
    #endif
    return 1;
    } else { 
    lastTick = millis();
    #ifdef DEBUG
    Serial.println("  Not ready: "); Serial.print(lastTick);
    Serial.print(" milisec");
    delay (50);
    #endif
    return 0;
    
  }  





}



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
 
#if defined DEBUG 
    
    if (sendTimer.poll((10000) * WAIT_TIME_MULTIPLY)) {// period sending set multiply by wait_time value ~ 3 sek procesing all and send 10 sek is a default multiply 1 time betwen sending measures change WAIT_TIME_MULTIPLY for multiply time 
       




       //while ( produceOutData() == 0) {};
             pendingOutput = produceOutData();
             //rf12_recvDone();
             //pendingOutput = 1;
             //delay(5);

       
    }


#else
   
    
// WAIT_TIME_MULTIPLY gives multiply 1 min , 
   
     
   
    if (sendTimer.poll(60000)) {
        
//for measure produce data

      if (repeatwait <= 1 ) {
       
       repeatwait = WAIT_TIME_MULTIPLY;
      
      //delay(20);//while (  produceOutData() == 0) {};
       pendingOutput = produceOutData();
       
       //pendingOutput = 1;
      }
     --repeatwait;
    }

#endif



 //send data if there are some


if (pendingOutput) {
      rf12_sleep(RF12_WAKEUP);
      delay(50);
      //rf12_recvDone();
      //delay(20);
      pendingOutput = 0;
      #if defined DEV_MODE
      
      //interrupts();
      Serial.println("sending sample to basenode"); 
      delay(10);
      rf12_sleep(RF12_SLEEP); 
      delay(50);
      #else
      doReport(); 
      delay(50);
      rf12_sleep(RF12_SLEEP); 
      //delay(100);  
      #endif
      
      pendingOutput = 0;

}






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


