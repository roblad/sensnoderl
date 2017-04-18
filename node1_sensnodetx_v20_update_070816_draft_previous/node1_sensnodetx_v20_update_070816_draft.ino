//DHT// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3
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

// libs for I2C and DS18B20
#include <Wire.h>
//#include <OneButton.h> // one buton functions to operate in functions (sensbase GOLD) http://www.mathertel.de/Arduino/OneButtonLibrary.aspx
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
#include <avr/power.h>
#include <avr/wdt.h>

// DHT11 and DHT22 from https://github.com/adafruit/DHT-sensor-library
#include "DHT.h"

//**************************************************************

#include  <PCF8574.h>  // expander I2C - http://akademia.nettigo.pl/starter_kit_030/programowanie_cd.html
// try to modyfy to lib http://playground.arduino.cc/Main/PCF8574Class
//*************************************************************

#include "SENSSTRUCTPAYLOAD.h"

//send pool timer

MilliTimer sendTimer;   // see https://github.com/jcw/jeelib/blob/master/examples/RF12/timedSend/timedSend.ino

//add additional control timer variables
//static


/**************RELAYS SETTINGS***********************************/
// controlbits 12 byte after fixing 11 bytes in structure - if you want to extend fixed data block above 11 change sequence of control bitsvariable

volatile byte pendingOutput = 0;
volatile unsigned int count_relaystatus = 0; //relaystatus
volatile byte coderelay = 0; //coderelayctatus execution check
volatile static byte RelayValue[RELAY_AMOUNT];  //sum of relays from all expanders



#ifdef LDR_SENSOR && NODEID = 1
volatile float ratio = 0;
//experymentaly set for full cycle of measure in WAIT_TIME_MULTIPLY in 60 sec - time for performing measuring. maximum value for timerpool is 60s for more wait state the for loop is needed with counter for timerpool
//int delayed = 1000;
#endif


//bit control for sending post fixed data and decoding it in receiver side

volatile byte dsSensoramount = 0;
//controll byte send for structure 
byte  phaseAmount = 0;


//DHT requirements
#ifdef DHT_SENSOR
//float h = 0; // define humidity DTH variable
//float t = 0; //define temperature DTH variable
#define DHTTYPE DHT_SENSOR_TYPE
#define DHTPIN 4 // port P1 = digital 4 on that port
#endif

//i2c condition
#if defined SHT21_SENSOR || defined BMP_SENSOR || defined RELAY_AMOUNT

 //set i2c
   #define I2C
   

//set expanders instances  
  
  #define LIST_OF_VARIABLES  X(EXPANDER0) X(EXPANDER1)  X(EXPANDER2) X(EXPANDER3)
  #define X(list) PCF8574 list;
  LIST_OF_VARIABLES
  #undef X
 

#endif

//set debug for RS printing from node side
#ifdef DEV_MODE
#define DEBUG
#endif

#ifndef DEBUG_BAUD
#define DEBUG_BAUD 9600
#endif


// Input/Output definition for sensmon
// Analog
#ifdef NEW_REV //3.4/4
#define LDR_PIN 2 //standard 2 nano 6 /new 7
#define BAT_VOL 3 //standard 3 nano 7 /new 6
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


// ***********set parameters for DS18B20****************
#if defined DS_COUNT && DS_COUNT > 0
DeviceAddress tempDeviceAddress; //for automatic getting DS device parameters in loops

// define precision of DS temperature displaying 9,10,11,12  - 12 is the best but longer reading is needed
#define TEMPERATURE_PRECISION 11 

//byte numberOfDevices_DS;  // number of DS devices found variable used for measure

OneWire oneWire(ONEWIRE_DATA);
DallasTemperature sensors(&oneWire);

// arrays to hold DS device results for transformation to integer
int ds_array[DS_COUNT];

// Set device addresses if you want to get temperature by address of DS device (check printed DS addresses in DEBUG mode) for each HEX add 0x...) and put here accordingly
  #ifdef DS_BY_ADDRESS_GETTING

 static byte Address_DS[DS_COUNT][8];  //variable for DS addresses
 
const byte Address_DS1[8] = { 0x28, 0xC4, 0xCE, 0x4C, 0x05, 0x00, 0x00, 0x1A }; 
const byte Address_DS2[8] = { 0x28, 0x54, 0x38, 0x4C, 0x05, 0x00, 0x00, 0x91 };
const byte Address_DS3[8] = { 0x28, 0x92, 0x64, 0x4D, 0x05, 0x00, 0x00, 0x87 };
const byte Address_DS4[8] = { 0x28, 0xF6, 0xDD, 0x4B, 0x05, 0x00, 0x00, 0x2D };
const byte Address_DS5[8] = { 0x28, 0xC1, 0x08, 0x38, 0x05, 0x00, 0x00, 0xE9 };
/*
 Address for DS [0] = 0x28, 0xC4, 0xCE, 0x4C, 0x05, 0x00, 0x00, 0x1A TEMP: 46.50 *C Resolution setting is: 11
 Address for DS [1] = 0x28, 0x54, 0x38, 0x4C, 0x05, 0x00, 0x00, 0x91 TEMP: 36.87 *C Resolution setting is: 11
 Address for DS [2] = 0x28, 0x92, 0x64, 0x4D, 0x05, 0x00, 0x00, 0x87 TEMP: 18.87 *C Resolution setting is: 11
 Address for DS [3] = 0x28, 0xF6, 0xDD, 0x4B, 0x05, 0x00, 0x00, 0x2D TEMP: 18.25 *C Resolution setting is: 11
 Address for DS [4] = 0x28, 0xC1, 0x08, 0x38, 0x05, 0x00, 0x00, 0xE9 TEMP: 36.25 *C Resolution setting is: 11
*/

//Address_DS[n][8] = { 0x28, 0x59, 0xBE, 0xDF, 0x02, 0x00, 0x00, 0x9F }; 
//Address_DS[n][8] set according to DS_COUNT [n]=[DS_COUNT] - address must be found in DEBUG mode DS device
  #endif
#endif 


//WDT set for uno and specyfic bootloaders - ned to be werified whether Watchdog working on
//ISR(WDT_vect) { Sleepy::watchdogEvent(); }


//port for jeenode compatibility
#ifdef NEW_REV
Port p1 (1); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p2 (2); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p3 (3); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p4 (4); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20

#endif

#ifdef DHT_SENSOR
DHT dht(DHTPIN, DHTTYPE);
#endif





void setup() {


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


//DHT initialize
#ifdef DHT_SENSOR
  dht.begin();
  delay (5);
#endif

#ifdef I2C
  //switch emon to 1 phase when DHT is set
  Wire.begin();

#endif

//serial setting

Serial.begin(DEBUG_BAUD);





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
/*
    EXPANDER0.enableInterrupt(5, onInterrupt_EXPANDER0);
    EXPANDER0.attachInterrupt(4, P1_onPin4_EXPANDER0, FALLING);
    EXPANDER0.attachInterrupt(5, P2_onPin5_EXPANDER0, FALLING);
    EXPANDER0.attachInterrupt(6, P3_onPin6_EXPANDER0, FALLING);    
*/ 
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
   
     #endif

    EXPANDER0.enableInterrupt(5, onInterrupt_EXPANDER0);
    EXPANDER0.attachInterrupt(4, P1_onPin4_EXPANDER0, FALLING);
    EXPANDER0.attachInterrupt(5, P2_onPin5_EXPANDER0, FALLING);
    EXPANDER0.attachInterrupt(6, P3_onPin6_EXPANDER0, FALLING);    
     
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


//set control bits function
 void setControlbits() {


  #if defined DS_COUNT

//dsSensoramount=0;

   if (DS_COUNT > 10 ) {

  dsSensoramount=0;

   } else {
   dsSensoramount=DS_COUNT;
   }

  #endif

measure.controlbits=(dsSensoramount * 16) + phaseAmount;  //sumaraise 2 digits to HEX 16 word wher 10ns are as a first 4 bits and phase as second 4 bits
    delay (5);





  #if defined DS_AS_MAIN_TEMP && defined DS_COUNT  && defined DS_BY_ADDRESS_GETTING && defined  DHT_SENSOR && defined SHT21_SENSOR

//decoder used also in base side
    phaseAmount=measure.controlbits & 15;
    dsSensoramount= measure.controlbits  >> 4;
    delay (2);
    #if defined  DEBUG
    Serial.println();
    delay (2);
    Serial.print("Control byte set: |");
    delay (2);
    Serial.print(measure.controlbits);
    delay (2);
    Serial.print("| Amount of DS sensors set: |");
    delay (2);
    Serial.print(dsSensoramount);
    delay (2);
    Serial.print("| Amount of power measure phases send: ");
    delay (2);
    Serial.println();
    delay (2);
    #endif

  #endif

}


//sending packages through radio for battery only -removed ArtekW conception





void doReport() {

  rf12_recvDone();
  delay(10);
  if (rf12_canSend()){
  rf12_sendStart(BASEDNODE, &measure, sizeof measure); //send in sync mode last partametr 1
  rf12_sendWait(RADIO_SYNC_MODE);
  activityLed(1);
  delay (2);
      if (RF12_WANTS_ACK) {
        rf12_sendStart(RF12_ACK_REPLY,0,0);
        rf12_sendWait(RADIO_SYNC_MODE);
        delay (2);
     }
       #ifdef SECONDNODE
    
       if (rf12_canSend()){
       rf12_sendStart(SECONDNODE, &measure, sizeof measure);
       rf12_sendWait(RADIO_SYNC_MODE);
       delay (2);
            if (RF12_WANTS_ACK) {
            rf12_sendStart(RF12_ACK_REPLY,0,0);
            rf12_sendWait(RADIO_SYNC_MODE);
            delay (2);   
            } 
       }
            
     #endif
   activityLed(0);
  }





 
}



//reference voltage bangup

long readVcc() {
  long resultVcc; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(30); // Wait for Vref to settle - 2 was inadequate
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  resultVcc = ADCL;
  resultVcc |= ADCH<<8;
  //resultVcc = 1126400L / resultVcc; // Back-calculate AVcc in mV
  resultVcc = (1125300L) / resultVcc;
  return resultVcc;
  //-49
}
//battery control
int battVolts(void) {

double vccref2 = 0.0;
unsigned int adcreading2 = 0;
 //analogReference(INTERNAL);

  
    readVcc();
    delay(10);
  
  vccref2 = readVcc()/1000.0;
 
  adcreading2 = analogRead(BAT_VOL) * 2;
  //vccref = readVcc()/1000.0;
  //adcreading = analogRead(BAT_VOL) * 2; 
  delay(10);
  double battvol = ((adcreading2 / 1024.0)) * vccref2;
  delay(10); 
  return battvol* 1000;
  
}
//LED activation
void activityLed (byte on) {
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, on);
  delay(20);
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
void printAddress(DeviceAddress deviceAddress) {
   #ifdef DEBUG
    
    for (uint8_t i = 0; i < 8; i++)   {
    Serial.print("0x");
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) {
    Serial.print(", ");

    }
   }
    #endif
}

//get values and set for transmission, DEBUG print to serial
void transmissionRS() {
#ifdef DEV_MODE
  Serial.println("==DEV MODE==");
  delay(2);
#endif

 #ifdef DEBUG
  //activityLed(1);
  Serial.println(' ');
  delay(2);
 #endif

#if defined DS_COUNT && DS_COUNT > 0

  #ifdef DEBUG
  Serial.print("DS18B20 found: ");
  delay(2);
  #endif
   byte numberOfDevices_DS = 0;
   numberOfDevices_DS=sensors.getDeviceCount();
  #ifdef DEBUG
   Serial.println(numberOfDevices_DS, DEC);
   delay(2);
  #endif
 
  for (byte i=0; i<numberOfDevices_DS; i++)
   {

     
      if(!sensors.getAddress(tempDeviceAddress, i))
      {
      #ifdef DEBUG
      Serial.println();
      delay (2);
      Serial.print("Unable to find DS address check connection of DS device");
      delay (2);
      Serial.println();
      delay (2);
      #endif
      break;
             
      } else {
      #ifdef DEBUG
      Serial.print(" Address for DS [");
      delay (2);
      Serial.print(i);
      delay (2);
      Serial.print("] = ");
      delay (2);
      #endif
      printAddress(tempDeviceAddress);
      #ifdef DEBUG
      Serial.print(" TEMP: ");
      delay (2);
      Serial.print((float) (ds_array[i]*0.01),2);
      delay (2);
      Serial.print(" *C");
      delay (2);
      Serial.print (" Resolution setting is: ");
      delay (2);
      Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
      delay (2);
      Serial.println(' ');
      delay(2); 
      #endif
       
      }
  }

#endif

#ifdef DEBUG
  Serial.println();
  delay (2);
  #ifdef LDR_SENSOR
  Serial.print("GAS ");
  delay (2);
  Serial.println((float)measure.light/100,2);
  delay(2);
  #endif
  #if defined  SHT21_SENSOR || defined DHT_SENSOR
  Serial.print("HUMI ");
  delay (2);
  Serial.println((float) measure.humi/10,2);
  delay(2);
  #endif
  //#if !defined DS_AS_MAIN_TEMP
  Serial.print("TEMP ");
  delay (2);
  //#endif
  //internal temp *
  #if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING && !defined  DHT_SENSOR && !defined SHT21_SENSOR 
  Serial.print(" INTERNAL ");
  delay (2);
  #endif
  #if defined DS_AS_MAIN_TEMP && defined DS_COUNT  && DS_COUNT > 0
  Serial.print(" DS0 ");
  delay (2);
  #endif
  Serial.println((float)measure.temp*0.1,2);
  delay(2);
 
  #if defined BMP_SENSOR
  Serial.print("PRES ");
  delay (2);
  Serial.println((float)measure.pressure/10,2);
  delay(2);
  #endif
  Serial.print("LOBAT " );
  delay (2);
  Serial.println(measure.lobat, DEC);
  delay(2);
  Serial.print("BATVOL ");
  delay (2);
  Serial.println((float)measure.battvol/1000,3);
  delay(2);
  Serial.println(' ');
  delay(2);

#endif

}

//for node 1 where LDR is set for ppm

#ifdef LDR_SENSOR

 int getppm (float ratio){
    //float sensor_volt;
  //float RS_air; //  Get the value of RS via in a clear air
  //float R0;  // Get the value of R0 via in H2

/*--- Get a average data by testing 100 times ---*/  
    //for(int x = 0 ; x < 100 ; x++)
  //{

  
 
   float ppm = 0.0;
   int getvalue;
     //dym
     if  (ratio < 5.6 && ratio >= 4.0 ) {
        ppm = 2000000000L * pow (ratio, -9.064);
        delay(10); 
      getvalue =  (int)( ppm / 100.0) +10000;

      #ifdef DEBUG
     
      delay(2);
      Serial.print("Dym   ");
      delay(2);
      //Serial.println(ppm,0);
     
      #endif
     
      return constrain (getvalue,10002,10100);
     
    } else if  //CO
 
    (ratio < 4.0 &&  ratio >= 1.8) {
    ppm = 0.0;
    getvalue = 0;
    ppm = 2000000 * pow (ratio, -6.500);
    delay(10); 
    getvalue =  (int)( ppm / 100.0) +20000;
    
    #ifdef DEBUG
   
    delay(2);
    Serial.print("UWAGA CO   ");
    delay(2);
    //Serial.println(ppm,0);
    #endif
   
    return constrain (getvalue,20002,20100);

     
    } else if  // CH4 and LPG
    
     (ratio  < 1.8 &&  ratio >= 0.12) {
        ppm = 0.0;
       getvalue = 0;   
        ppm = 186.73 * pow (ratio, -2.503);
        delay(10);  
        getvalue =  (int)( ppm / 100.0) +30000;

    #ifdef DEBUG
   
    delay(2);
    Serial.print("CH4 or LPG   ");
    delay(2);
            //Serial.println(ppm,0);
    #endif
   
    return constrain (getvalue,30002,30100);

     
   } else if (ratio  < 0.12){

     #ifdef DEBUG
    
     Serial.println("explosion");
    
     #endif
    
     return 9999;

  } else {

    #ifdef DEBUG
   
    delay(2);
    Serial.print("Brak gazow - czyste powietrze ");
    delay(2);
   
    #endif
   
    return 0;
 
  }  
}

void getValueLDR (){

float RS_air = 0;
float sensor_volt = 0; 
int sensorValue = 0;

   for(int x = 0 ; x < 20 ; x++)
  {
   sensorValue += analogRead(LDR_PIN);
   delay(10); 
  }

   sensorValue = sensorValue / 20;
   float vccref=0.0;
   for(int x = 0 ; x < 10 ; x++)
  {
   vccref += readVcc()/1000.0;

    delay(2);
  }
  vccref =vccref / 10.0;

   sensor_volt = ((float)sensorValue/1024.0 * (vccref) ) * 2 ;
   delay(2);

   RS_air = (5.10-sensor_volt)/sensor_volt; // omit *RL
  //R0 = RS_air/7.0; // The ratio of RS/R0 is 6.5 in a clear air from Graph (Found using WebPlotDigitizer)
  /*-Replace the name "R0" with the value of R0 in the demo of First Test -*/
 
   ratio = RS_air/1.005;//0.95;//0,751.20;//8.60;//1.21/8.30,10.91;//7.90;//10.91;  // ratio = RS/R0

   #ifdef DEBUG
   Serial.print("Ref volt = ");
    delay(2);
   Serial.print((float)(readVcc()/1000.0),4);
    delay(2);
   Serial.println(" V");
    delay(2);
   Serial.print("Sensor volt = ");
    delay(2);
   Serial.print(sensor_volt,4);
    delay(2);
   Serial.println("V");
    delay(2);
   Serial.print("Rs/R0 - ratio = ");
    delay(2);
   Serial.println(ratio,4);
    delay(2);
   Serial.print("Sensor value (multiply by 2) = ");
    delay(2);
   Serial.println(sensorValue);
    delay(2);
   #endif
   delay(10); 
}

//end read gas sensor LDR
#endif

//get measure for sending
void doMeasure() {       //main measure
 
  //count++;
  //unsigned int adcreading = 0;
  //measure.lobat = rf12_lowbat();  //probably if RF 69 it should be commented and measure should be changed
  measure.lobat = 0;
  measure.battvol = battVolts();
  delay (20);
 #ifdef LDR_SENSOR
  //if ((count % 2) == 0) {  // measuring once per two cycles only for photoresistor usage - uncoment if declaration and closing bracket

    //analogReference(INTERNAL);

    //measure.light = (analogRead(LDR_PIN));
    // use for photoresistor
    //double vccref_ldr = readVcc()/1000.0;
    //delay (20);
    //unsigned int adcreading_ldr;
   
  
     
    //adcreading_ldr = 1023 - (analogRead(LDR_PIN));

   
    //delay (20);
    //double light_ldr = ((adcreading_ldr * (vccref_ldr/ 1023.0)));
    //double measure_ldr = light_ldr * 1000;
          
    //measure.light = map(measure_ldr,0,((vccref_ldr*1000)-((vccref_ldr*1000)*0.005)),0,1000);
    // it is for sensors 5V/3.3V-0 - ((vccref_ldr*1000)-1700) - for 5V (vccref_ldr*1000) for 3.3V
    // use for any Chinese arduino sensors with LM386 chip,
    // plug to 3.3V and A3 to LDR  - check polarity,
    // remove R3 resistor from PCB. type http://botland.com.pl/373-czujniki-do-arduino */
 // }



  
   #ifdef DEBUG

   Serial.println("Current value where prefix 0 - clean air,  1 - Dym, 2 - CO, 3 - CH4/LPG:  ");
   delay(2);
 
   // Serial.print ((float)getppm (ratio)/100);
   //Serial.print("Vccref = ");
   //Serial.println((readVcc()/1000.0),2);
   #endif
   getValueLDR ();
   delay(10);  
   measure.light = (getppm (ratio) );
   delay(10); 
   #ifdef DEBUG
   Serial.print((float)measure.light/100,2);
   delay(2);
   Serial.println (" %/m3 - ppm( x 10000)");
   delay(2);
   #endif
  
     
//sensors initialise and getting values for measure

 #endif

 #ifdef I2C
 
     #ifdef SHT21_SENSOR
      float shthumi = SHT2x.GetHumidity();
      delay(20);
      measure.humi = shthumi * 10;
      float shttemp = SHT2x.GetTemperature();
      delay(20);
      measure.temp = shttemp * 10;
     #endif

     #ifdef BMP_SENSOR
      BMP085.getCalData();
      delay (50);
      BMP085.readSensor();
      delay (50);
      measure.pressure = ((BMP085.press*10*10) + 16);
     #endif

 #endif


 #ifdef DHT_SENSOR
   #ifndef SHT21
    float h = dht.readHumidity();
    delay (50);
    float t = dht.readTemperature();
    delay (50);
    if (isnan(h) || isnan(t)) {
     measure.humi = 999;
     measure.temp = 999;
    return;
    } else {
     measure.humi = h*10;
     measure.temp = t*10;
    }
  #endif

 #endif



#if defined DS_COUNT && DS_COUNT > 0
   byte numberOfDevices_DS = 0;
   sensors.begin();
   delay (50);
   oneWire.reset_search();
   delay (50);
    //measure.controlbits=0;numberOfDevices_DS=0;
    numberOfDevices_DS = sensors.getDeviceCount();
    delay (50);
   if (numberOfDevices_DS > DS_COUNT) numberOfDevices_DS=DS_COUNT;   //set proper values for DS amount per found real attached DS - only declared value in configuration will be used
   if (numberOfDevices_DS > 0) {// check number of DS devices found
    dsSensoramount=numberOfDevices_DS;
    //set precision for all found devices
    for(byte i=0; i < numberOfDevices_DS; i++) {
    sensors.getAddress(tempDeviceAddress, (i));
     delay (50);
    sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
    delay (20); //wait for seting - can be increased for many devices
    //Sleepy::loseSomeTime(100);  //it causes hang of processor it is replaced in baterry only set WDT is used there in this place it should be not nesscessary
    }
    sensors.setWaitForConversion(true); //being tested asynhrounous reading -  needed to be checked for faster reading temperature test required
    //delay (50);
    sensors.requestTemperatures();
    delay(700);    
   
    //Sleepy::loseSomeTime(750);  //can be increased when amount of devices ic huge - it causes hang of processor it is replaced in baterry only set WDT is used there in this place it should be not nesscessary
 
  
    #ifndef DS_BY_ADDRESS_GETTING  
       oneWire.reset_search();
       delay (10);
       for(byte i=0; i < numberOfDevices_DS; i++) {
        float temp = sensors.getTempCByIndex(i);
        delay (20);
        ds_array[i] = temp * 100;
        measure.temp_DS[i] = ds_array[i];
     
      }
    #endif

    #if defined DS_BY_ADDRESS_GETTING  //set DeviceAddress Address_DS[n] if you want to use that, uncoment  addresses in the "set parameters for DS18B20" and DS_BY_ADDRESS_GETTING in config
       oneWire.reset_search();
       delay (10);
        for(byte i=0; i < numberOfDevices_DS; i++) {
        oneWire.search(Address_DS[i]);
        delay (20);
        float temp = sensors.getTempC(Address_DS[i]);
        delay (20);
        //oneWire.reset_search();
        //delay (10);
        //Serial.println(sensors.getTempC(Address_DS[i]));
        //Serial.println(oneWire.search(Address_DS[i]));
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


//code relay function
#if defined RELAY_AMOUNT && defined STRUCT_RELAY_STATUS  && EXPANDERS_AMOUNT && NSENSORS < 2

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
    delay (2);
    Serial.print(RELAY_AMOUNT);
    delay (2);
    Serial.print("| Expanders ammount: |");Serial.print(EXPANDERS_AMOUNT);
    delay (2);
    Serial.print("| Coded amount of relays: ");Serial.println(measure.relaystatus);
    delay (2);
    Serial.println("PCF8574 real value is opposite logic value I/O");
    delay(2);
    #endif    
  for (byte i=0; i< RELAY_AMOUNT; i++) {
    delay(10);
// it can be changed when 1 expander has different amount of relays - each case in switch should be  set accordingly
// read each pin for particular expander and get code for sending       


  //expander 1
  #if defined ADDRESS_0 && EXPANDERS_AMOUNT > 0   
   switch (i) {     
         case 0:
            RelayValue[i] = ((EXPANDER0.digitalRead(0) == 0? 1:0));   //check for proper expander I/O pins
            #ifdef DEBUG
            Serial.print(" Relay |");Serial.print(i+1);delay(2);Serial.print("| Relay value: ");delay(2);Serial.println(RelayValue[i]);delay(2);
            #endif
            delay(10);   
//== 1? 0:1);          
          break;
           
          case 1:
            RelayValue[i]= ((EXPANDER0.digitalRead(1) == 0? 1:0));   //check for proper expander I/O pins
            #ifdef DEBUG
            Serial.print(" Relay |");
            delay(2);
            Serial.print(i+1);
            delay(2);
            Serial.print("| Relay value: ");Serial.println(RelayValue[i]);
            delay(2);
            #endif 
            delay(10); 
//== 1? 0:1);
          break;
          case 2:
            RelayValue[i] = ((EXPANDER0.digitalRead(2)== 0? 1:0));   //check for proper expander I/O pins
            #ifdef DEBUG
            Serial.print(" Relay |");
            delay(2);
            Serial.print(i+1);
            delay(2);
            Serial.print("| Relay value: ");Serial.println(RelayValue[i]);
            delay(2);
            #endif 
            delay(10); 
//== 1? 0:1);
          break;
         }
  #endif

//code of relays status
  
    //count_relaystatus will code status of  relays
    measure.relaystatus+= (RelayValue[i]==0) ? (pow(2,(i))):0;
    delay(10);
   #ifdef DEBUG
   Serial.print(" Binary value sum collected: ");Serial.println(count_relaystatus,BIN);
   #endif
  }


  #ifdef DEBUG
  Serial.print(" Coded relays amount + statuses: ");
  delay (2);
  Serial.println(measure.relaystatus);
  delay (2);
  #endif

  return 1;
}
 
 
#endif


#if defined ADDRESS_0 && EXPANDERS_AMOUNT > 0     //for EXPANDER_0 switch setting functions
// relays interput functions for switches (add similar section for more expanders)

// PCF interput function
  void onInterrupt_EXPANDER0() {
  #ifdef DEBUG
  Serial.println("PCF interput IRQ PIN5");
  delay (2);
  #endif
//set expander number set - EXPANDER0
  EXPANDER0.checkForInterrupt();
  delay (50);
}


void P1_onPin4_EXPANDER0() {
//EXPANDER0 

    #ifdef DEBUG 
    Serial.println("Switch for P1 - Pin 4 received");
    delay (2);
   #endif  

   // static unsigned long lastMillis = 0;   //function for switch unstable status - usualy used for atmega for direct attaching
   // unsigned long newMillis = millis();
   //if (newMillis - lastMillis <50) {
   //Serial.println("migotanie");
   //} else {
      if (EXPANDER0.digitalRead(0) == 0) {
      // jesli tak to zapala diode LED
      // expander.digitalWrite(0, 1);
      EXPANDER0.toggle(0);
      } else  {
      // jesli nie to wylacza
      //expander.digitalWrite(0, 0);
      delay(10);
      EXPANDER0.toggle(0);
      }
      // lastMillis=newMillis;
      // }
      coderelay = coderelayStatus();
      delay (10);
      coderelayStatusConfirmation();
      delay(10);

}

void P2_onPin5_EXPANDER0() {
//EXPANDER0

  #ifdef DEBUG
  Serial.println("Switch for P2 - Pin 5 received");
  delay (2);
  #endif
 // static unsigned long lastMillis = 0;
 // unsigned long newMillis = millis();
 // if (newMillis - lastMillis <50) {
   //Serial.println("migotanie");
   //} else {
      if (EXPANDER0.digitalRead(1) == 0) {
      // jesli tak to zapala diode LED
      //expander.digitalWrite(1, 1);
      EXPANDER0.toggle(1);
      } else  {
      delay(10);
        // jesli nie to wylacza
      //expander.digitalWrite(1, 0);
      EXPANDER0.toggle(1);
      }
      //lastMillis=newMillis;
      // }
      coderelay = coderelayStatus();
      delay (10);
      coderelayStatusConfirmation();
      delay(10);

}

void P3_onPin6_EXPANDER0(){
//EXPANDER0

    #ifdef DEBUG
    Serial.println("Switch for P3 - Pin 6 received");
    delay (2);
    #endif
   // static unsigned long lastMillis = 0;
   // unsigned long newMillis = millis();
   //if (newMillis - lastMillis <50) {
   //Serial.println("migotanie");
   //} else {
      if (EXPANDER0.digitalRead(2) == 0) {
      // jesli tak to zapala diode LED
      //expander.digitalWrite(2, 1);
      EXPANDER0.toggle(2);
      } else  {
      delay(10);
        // jesli nie to wylacza
      //expander.digitalWrite(2, 0);
      EXPANDER0.toggle(2);
      }
     // lastMillis=newMillis;
     // }
     coderelay = coderelayStatus();
     delay (10);
     coderelayStatusConfirmation();
     delay(10);
}

#endif




void receiveRXrelayPayload () {


#ifdef DEBUG 
 
    if (rxdata.destnode == NODEID && rxdata.state == 1 || rxdata.state ==0 ) {
    Serial.println("Proccessing commands received for relays. ");
    delay (2);
    } else {
     Serial.println("Nothing to process after data receiving. ");
    delay (2);
    }
 
#endif

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
     #ifdef DEBUG 
     Serial.println("EXECUTE RELAY 1");
     delay (2);
     #endif
     break;
     case 1:
     EXPANDER0.digitalWrite(0, LOW);
      //P1_onPin4_EXPANDER0();  //test
     delay(10);
     #ifdef DEBUG 
     Serial.println("EXECUTE RELAY 1");
     delay (2);
     #endif
     break;

    }
   } else if (rxdata.cmd == 2) {
// process cmd
    switch (rxdata.state) {
     case 0:
     EXPANDER0.digitalWrite(1, HIGH);
     delay(10);
     #ifdef DEBUG 
     Serial.println("EXECUTE RELAY 2");
     delay (2);
     #endif
     break;
     case 1:
     EXPANDER0.digitalWrite(1, LOW);
     delay(10);
     #ifdef DEBUG 
     Serial.println("EXECUTE RELAY 2");
     delay (2);
     #endif
     break;
    }
  
   } else if (rxdata.cmd == 3) {
  // process cmd
    switch (rxdata.state) {
     case 0:
     EXPANDER0.digitalWrite(2, HIGH);
     delay(10);
     #ifdef DEBUG 
     Serial.println("EXECUTE RELAY 3");
     delay (2);
     #endif
     break;
     case 1:
     EXPANDER0.digitalWrite(2, LOW);
     delay(10);
     #ifdef DEBUG 
     Serial.println("EXECUTE RELAY 3");
     delay (2);
     #endif
     break;
    }

    } else {
    
     #ifdef DEBUG 
     Serial.println("Nothing to do");
     delay (2);
     #endif
   }
  }


  #endif



}

//#endif   //end if RELAY_AMOUNT



void CheckIfdataCome () {
 
    #ifdef DEBUG
    Serial.println();
    delay(2);
    Serial.print("RX:");
    delay(2);
    Serial.print("|");
    Serial.print((int) rf12_hdr);
    delay(2);
    for (byte i = 0; i < rf12_len; ++i) {
    Serial.print("|");
    delay(2);
    Serial.print((int) rf12_data[i]); //data print binary
    delay(2);
    }
   
    Serial.println("|  Receiving data from basenode to process for node: ");
    delay(2);
    Serial.println((int) rf12_data[0]); //data print binary
    delay(2);
    //Serial.println(rf12_len);
    #endif
}




//#if defined RELAY_AMOUNT  //curently for relays and pulse - add next if you want get data from basenode
//report status of relays or pylse values received  after receiving command for change state
void coderelayStatusConfirmation(){
    #ifdef LED_ON
    activityLed(1);
    #endif
    rf12_recvDone();
    delay(10);
    if (rf12_canSend()) {
    rf12_sendStart(BASEDNODE, &measure, sizeof measure);  //sync mode parameter 1 last parameter set
    rf12_sendWait(RADIO_SYNC_MODE);
    delay (2);
     if (RF12_WANTS_ACK) {
     rf12_sendStart(RF12_ACK_REPLY,0,0);
     rf12_sendWait(RADIO_SYNC_MODE);
     delay (2);
     //rf12_sendWait(1);
     }
    }
 

    #ifdef LED_ON
    activityLed(0);
    #endif

}

//#endif



//sending/printing  process function
void sendPayload() {


#ifdef LED_ON
  activityLed(1);
#endif
 

#if defined RELAY_AMOUNT && defined STRUCT_RELAY_STATUS && defined EXPANDERS_AMOUNT
 coderelay = coderelayStatus();
 delay (10);
#endif


#if defined STRUCT_CONTROL_BYTE
  setControlbits(); //set control bits function run
#endif

doMeasure(); // measure other sensors

#ifdef DEBUG
  delay (2);
  transmissionRS();  //printout data to serial
#endif

#ifdef LED_ON
  activityLed(0);
#endif

}

//function for received data
// setd and receive for node see http://jeelabs.org/2011/05/07/rf12-skeleton-sketch/#comments
// when something will be received for node
void consumeInData () {

#ifdef DEBUG
  CheckIfdataCome ();
#endif

receiveRXrelayPayload ();
}

//function for sending data
// data send for some period time
byte produceOutData () {

  sendPayload();
  delay(10);
  return 1;

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
  result = ((result - 125) * 1075); //Don't forget to change "x" with multiplication sign
  return result;
}

#endif


// Main loop
void loop() {
  //long czas1 = millis();  // sending timer



//sending time for RELAYS - check sending timer before set
   

 


   if (rf12_recvDone() && rf12_crc == 0 && ((int) (rf12_hdr & 0x1F)) == BASEDNODE && rf12_len == sizeof rxdata && ((int) rf12_data[0]) == NODEID) {

       pendingOutput = 0;

       //if (((int) rf12_data[0]) != NODEID) return;
       
       memcpy(&rxdata, (void*) rf12_data, sizeof rxdata); 
       delay(2);  
       consumeInData ();
       delay (10);
       coderelay = coderelayStatus();
       delay (5);
       coderelayStatusConfirmation(); //coment it for basenode
       delay (100);
       coderelay = coderelayStatus();
       delay (5);
       coderelayStatusConfirmation(); //coment it for basenode
       
       //delay (100);
              
       
   }

     
#if defined DEBUG 
    
    if (sendTimer.poll((10000))) {// period sending set multiply by wait_time value ~ 3 sek procesing all and send 10 sek is a default multiply 1 time betwen sending measures change WAIT_TIME_MULTIPLY for multiply time 
       

       pendingOutput = produceOutData();

             delay(5);

       
    }

#else
   
    
// WAIT_TIME_MULTIPLY gives multiply 1 min , wheret there is combination 20s up and waiting for commands from base and 40s in sleep mode 

    
   
    if (sendTimer.poll(((60000 * WAIT_TIME_MULTIPLY) - 3500 ))) {
       
//for pulse measure produce data

          
       
       pendingOutput = produceOutData();
       
       
       delay(5);
   
    }

#endif



 //send data if there are some

if (pendingOutput && rf12_canSend() ) {
      
      pendingOutput = 0;
      //interrupts();
      doReport();   
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

void serialEvent() {



      // kod do wykonania podczas odbioru danych
   if (Serial.available() >=5  ) {  // should  be >5 characters - digits first is node id  and \n as last or node id xx and 0000 space and max value  4294967295 and \n 
     
      
    rxdata.destnode = 0;
    rxdata.cmd = 0;
    rxdata.state = 0;
         
       /* RX 03121 | NID CMD STATE */
        //needToSend = 1; //uncoment it for basenode
     
        static char ramka[5]; //comment it for basenode
 
        int rxb = Serial.available();  //read values
        for (int i = 0; i<rxb; i++) {
       char test = Serial.read();
       
            
      ramka[i] = (test -48);
      // ramka[i] = (Serial.parseInt()-48); 
      //delay(5);
      
      }

      Serial.flush();

      //proccess relays data
    rxdata.destnode = (ramka[0] * 10) + ramka[1];
    rxdata.cmd = (ramka[2] * 10) + ramka[3];
    rxdata.state = ramka[4];


// data to send use node ID ex. 02 and 3 times zeros from the begining and space, after that type value to transmit to node (currently used for pulse)max value  4294967295 
 

       #ifdef DEBUG
   
       Serial.println();
       delay (2);
       Serial.print("SEND NODE ");
       delay (2);
       Serial.print(rxdata.destnode);
       delay (2);
       Serial.print(" RELAY ");
       delay (2);
       Serial.print(rxdata.cmd);
       delay (2);
       Serial.print(" STATE ");
       delay (2);
       Serial.print(rxdata.state);
       delay (2);
       Serial.print(' ');
       delay (2);

       #endif

   

       receiveRXrelayPayload();
       delay (10);
       coderelayStatus();
       //coderelay = 0;
       delay (10);

      coderelayStatusConfirmation(); //coment it for basenode

    }   
 
}


