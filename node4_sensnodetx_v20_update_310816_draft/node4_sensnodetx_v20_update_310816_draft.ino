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

//*********** BATTERY SAVING ********
#ifdef BATTERY_ONLY

  #define VCC_OK 3400 // enough power for normal 1-minute sends
  #define VCC_LOW 3200 // sleep for 1 minute, then try again
  #define VCC_DOZE 3000 // sleep for 5 minutes, then try again
// sleep for 60 minutes, then try again
  #define VCC_SLEEP_MINS(x) ((x) >= VCC_LOW ? 1 : (x) >= VCC_DOZE ? 5 : 60)
  #define VCC_FINAL 2900 // send anyway, might be our last swan song

#endif

// libs for I2C and DS18B20
#include <Wire.h>
//#include <OneButton.h> // one buton functions to operate in functions (sensbase GOLD) http://www.mathertel.de/Arduino/OneButtonLibrary.aspx
//http://majsterkowo.pl/zegar-ds1307/ - zegar
#include <OneWire.h>
#include <DallasTemperature.h>
// lisb for SHT21 and BMP085
#include <BMP085.h>
#include <SHT2x.h>

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
#include "SENSSTRUCTPAYLOAD.h"
//txpower RF12b control variable
byte txPower = TXPOWER;   //0 -max power, 7 -minimum there is last byte of comand control rf12_control(0x9850) -means max, rf12_control(0x9857); -means min -21dbm
//set mili timer for sent package periodicaly and switch to receiving mode  //sending timer
MilliTimer sendTimer;   // see https://github.com/jcw/jeelib/blob/master/examples/RF12/timedSend/timedSend.ino

//add additional control timer variables
//static
byte pendingOutput;

//sending time for RELAYS - check sending timer before set
// Pulse counting settings

/**************RELAYS SETTINGS***********************************/ 
// controlbits 12 byte after fixing 11 bytes in structure - if you want to extend fixed data block above 11 change sequence of control bitsvariable

//bit control for sending post fixed data and decoding it in receiver side 
#ifdef STRUCT_CONTROL_BYTE
byte dsSensoramount;
#endif
//DHT requirements

#ifdef DHT_SENSOR
float h; // define humidity DTH variable
float t; //define temperature DTH variable
#define DHTTYPE DHT_SENSOR_TYPE

#define DHTPIN 4 // port P1 = digital 4 on that port

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
#define DS_PWR 7  //(can be used for other reason when mosfet is not avaliable)
#define ONEWIRE_DATA 8
#define ACT_LED 9
#define SOLAR 0  //AIO0
#define GROUND_PWR 3 //DIO3
#define DHT22_PWR 5
#else //3.0
#define CustomD3 3
#define CustomD4 4
#define MOSFET 5
#define ONEWIRE_DATA 8
#define ACT_LED 9 //standard 9 uno 13 
#endif


//measure variables for sensmon
byte count = 0;
unsigned int adcreading;  //for bang up voltage
unsigned int adcreading_solar;  //for bang up voltage solar
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

const byte Address_DS0[8] = { 0x28, 0x63, 0x01, 0xFB, 0x05, 0x00, 0x00, 0x99 }; 
const byte Address_DS1[8] = { 0x28, 0x63, 0x01, 0xFB, 0x05, 0x00, 0x00, 0x99 };
//const byte Address_DS2[8] = { 0x28, 0x63, 0x01, 0xFB, 0x05, 0x00, 0x00, 0x99 };
//const byte Address_DS3[8] = { 0x28, 0x63, 0x01, 0xFB, 0x05, 0x00, 0x00, 0x99 };
//const byte Address_DS4[8] = { 0x28, 0x63, 0x01, 0xFB, 0x05, 0x00, 0x00, 0x99 };
//const byte Address_DS5[8] = { 0x28, 0x63, 0x01, 0xFB, 0x05, 0x00, 0x00, 0x99 };


//Address_DS[n][8] = { 0x28, 0x59, 0xBE, 0xDF, 0x02, 0x00, 0x00, 0x9F };  
//Address_DS[n][8] set according to DS_COUNT [n]=[DS_COUNT] - address must be found in DEBUG mode DS device
  
  #endif


#endif


//WDT set for uno and specyfic bootloaders - ned to be werified whether Watchdog working on
ISR(WDT_vect) { Sleepy::watchdogEvent(); }


//port for jeenode compatibility
#ifdef NEW_REV
//Port p1 (1); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
//Port p2 (2); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
//Port p3 (3); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
//Port p4 (4); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20

#endif

#ifdef DHT_SENSOR
DHT dht(DHTPIN, DHTTYPE);
#endif

//***************************SETUP BEGIN*********************//


void setup() {
#ifdef DS_PWR && DS_COUNT

pinMode(DS_PWR,OUTPUT);
digitalWrite(DS_PWR,LOW);  
#endif

#ifdef LDR_SENSOR && GROUND_PWR

  pinMode(GROUND_PWR,OUTPUT);
  digitalWrite(GROUND_PWR,LOW);

#endif 

#ifdef DHT22_PWR && DHT_SENSOR

 pinMode(DHT22_PWR,OUTPUT);
digitalWrite(DHT22_PWR,LOW);

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


//DHT initialize
#ifdef DHT_SENSOR
  dht.begin();
 delay(50);
#endif




#ifdef DEBUG
  Serial.begin(DEBUG_BAUD);
#endif


	rf12_recvDone();
	delay(10);
	
	if (RF12_WANTS_ACK) {
		rf12_sendStart(RF12_ACK_REPLY,0,0);
		delay (5);
		//rf12_recvDone(); 
		
	}
} //end setup



//Functions declaration
#if defined STRUCT_CONTROL_BYTE 
//set control bits function 
static void setControlbits() {

//coder for controlbits

#if defined DS_COUNT

//dsSensoramount=0;

if (DS_COUNT > 10 ) {

  dsSensoramount=0; 

   } else {
   dsSensoramount=DS_COUNT;
}

#endif

measure.controlbits=(dsSensoramount * 16) + 1;  //sumaraise 2 digits to HEX 16 word wher 10ns are as a first 4 bits and phase as second 4 bits



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
void doReport() {

activityLed(1);
 
       while (!rf12_canSend())    // wait until sending is allowed
               rf12_recvDone();      // process any incoming data (in the background)
               rf12_sendStart(BASEDNODE, &measure, sizeof measure);  //sync mode parameter 1 last parameter set
               rf12_sendWait(1);
               delay(10);
               rf12_recvDone();
               delay(10);


              if (RF12_WANTS_ACK) {
              rf12_sendStart(RF12_ACK_REPLY,0,0);
              delay (5);
              }

     
    
   
  #ifdef SECONDNODE
       if (rf12_canSend()){
       rf12_sendStart(SECONDNODE, &measure, sizeof measure);
       rf12_sendWait(1);
       delay(10);  
       rf12_recvDone();
       delay(10);
       
       if (RF12_WANTS_ACK) {
       rf12_sendStart(RF12_ACK_REPLY,0,0);
       delay (5);
               //rf12_recvDone(); 
              
             }

       

       }
  #endif
  
   activityLed(0);
}
//reference voltage bangup

/*long readVcc() {
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
*/

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
/*int battVolts(void) {


 analogReference(INTERNAL);

   for (int i=0; i<5;i++ ) {
    readVcc();
    dodelay(10);
   }
  double vccref = readVcc()/1000.0;
  delay(10); 
  adcreading = analogRead(BAT_VOL) * 2;
  //vccref = readVcc()/1000.0;
  //adcreading = analogRead(BAT_VOL) * 2;  
  delay(10); 
  double battvol = ((adcreading / 1023.0)) * vccref;
  return battvol* 1000;
}
*/
int battVolts(void) {
    readVcc();
    delay(10);
  
  double vccref = readVcc()/1000.0;
 
  adcreading = analogRead(BAT_VOL) * 2;
  //vccref = readVcc()/1000.0;
  //adcreading = analogRead(BAT_VOL) * 2; 
  delay(10);
  double battvol = (adcreading / 1024.0) * vccref;
  return (battvol * 1000)+38;
}


//Solar voltage

#ifdef SOLAR 
//battery control 
int SolarVolts(void) {


 //analogReference(INTERNAL);

   for (int i=0; i<5;i++ ) {
    readVcc();
    dodelay(10);
   }
  double vccref_solar = readVcc()/1000.0;
  dodelay(20); 
  for (int i=0; i<3;i++ ) {
  adcreading_solar = ((analogRead(SOLAR)) * 2);
  }
  //vccref = readVcc()/1000.0;
  //adcreading = analogRead(BAT_VOL) * 2;  
  dodelay(20);
  //double solar = ((adcreading_solar / 1023.0)) * vccref_solar;
  //return solar * 1000;
  
    double solar = ((adcreading_solar ) * vccref_solar / 1024.0);
    solar = solar * 1000;
    int map_ = map(solar,0,6670,0,1000);
    //delay(20);
    //float value = max(map_, 50);
    if (map_ < 400) {
    return 0;
    } else if (map_ > 350 && map_ < 967){
    return map_;
    } else{
      return 1000;
      //return map_;
    }
    //return solar;
}
# endif

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
void printAddress(DeviceAddress deviceAddress) {
   for (uint8_t i = 0; i < 8; i++) {
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
  Serial.print("GROUND HUMI ");
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
  
  //#if defined BMP_SENSOR  //used for solar
  Serial.print("SUN INTESIVITY ");
  Serial.println((float)measure.pressure/10,1);
  delay(2);

  //#endif
  //Serial.print("LOBAT " );
  //Serial.println(measure.lobat, DEC);
  //elay(2);
  Serial.print("BATVOL ");
  Serial.println((float)measure.battvol/1000,3);
  delay(20);
  Serial.println(' ');
  delay(20);
}

//get measure for sending
static void doMeasure() {       //main measure
  
  count++;
  adcreading = 0;
  measure.lobat = 0;//rf12_lowbat();  //probably if RF 69 it should be commented and measure should be changed 
 
  measure.battvol = battVolts();
  measure.pressure= SolarVolts();

  #ifdef LDR_SENSOR
  //if ((count % 2) == 0) {  // measuring once per two cycles only for photoresistor usage - uncoment if declaration and closing bracket

  #ifdef GROUND_PWR

    digitalWrite(GROUND_PWR,HIGH);
    dodelay(100); 

  #endif
  //  analogReference(INTERNAL);

    //measure.light = (analogRead(LDR_PIN));
    // use for photoresistor
    double vccref_ldr = readVcc()/1000.0;
    delay (20);
    unsigned int adcreading_ldr;
   
    for (int i=0; i<3;i++ ) {
      
    adcreading_ldr = 1024 - (analogRead(LDR_PIN));
    delay (100);
    }
    
    //delay (20);
    double light_ldr = ((adcreading_ldr * (vccref_ldr/ 1024.0)));
    delay (20);
    //double test = ((analogRead(GROUND_PWR) / 1023.0) * vccref_ldr);
    delay (20);
    double measure_ldr = (light_ldr  * 1000)   ;
    
    
    measure.light =  map(measure_ldr,0,(2800),0,1000);
    
    if (measure.light < 300) {
      
      measure.light = 0;
      
    } else if (measure.light >980) {
        
      measure.light = 1000;
      
    } else {
       measure.light=measure.light;
    }
    
     // it is for sensors 5V/3.3V-0 - ((vccref_ldr*1000)-1700) - for 5V (vccref_ldr*1000) for 3.3V
    // use for any Chinese arduino sensors with LM386 chip,
    // plug to 3.3V and A3 to LDR  - check polarity,
    // remove R3 resistor from PCB. type http://botland.com.pl/373-czujniki-do-arduino */
 // }

//sensors initialise and getting values for measure
  #ifdef GROUND_PWR

  digitalWrite(GROUND_PWR,LOW);

  #endif
#endif

#ifdef DHT_SENSOR

  #ifdef DHT22_PWR   
    digitalWrite(DHT22_PWR,HIGH);                              // Send the command to get temperatures
    dodelay(2000);                                             //sleep for 1.5 - 2's to allow sensor to warm up
  #endif 
  
  #ifndef SHT21
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    return;
  }  else {

    measure.humi = h*10;
    measure.temp = (t-0.1)*10;
  }
  #endif
    #ifdef DHT22_PWR
    digitalWrite(DHT22_PWR,LOW);
    #endif
#endif



#if defined DS_COUNT && DS_COUNT > 0 
 
#ifdef DS_PWR
digitalWrite(DS_PWR,HIGH); 
dodelay(500); 
#endif
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
    dodelay (100);
    
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


   #ifdef DS_PWR
   delay(100);
   digitalWrite(DS_PWR,LOW); 
   #endif
#endif

//set DS main temperature, default DS sensor 0, you can change it to any

#if defined DS_AS_MAIN_TEMP && defined DS_COUNT && DS_COUNT > 0
    measure.temp = measure.temp_DS[0]*0.1;
#endif

#if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING && !defined  DHT_SENSOR && !defined SHT21_SENSOR
 
 measure.temp = readTemp()/1000;
#endif
}


//sending/printing  process function 
static void sendPayload() {

//rf12_sleep(RF12_WAKEUP);   //wake up RF module  - need to be measured for current consumption by multimeter if it is required amd sending was mising 

#ifdef LED_ON
  activityLed(1);
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
  result = ((result - 125) * 1075)+60000;; //Don't forget to change "x" with multiplication sign
  return result; 
}

#endif


void dodelay(unsigned int ms)
{
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;
Sleepy::loseSomeTime(ms);
   
  ADCSRA=oldADCSRA;         // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}


// Main loop
void loop() {
  //long czas1 = millis();  // sending timer



//memset (&measure,0,sizeof (measure));
//memset (&rxdata,0,sizeof (rxdata));
//waiting for comands (can be extended in receiving structure)
#if !defined BATTERY_ONLY

   //waiting for comands (can be extended in receiving structure)
   //for (byte i=0;i<10;i++){   // loop for to be sure that proper package can be received  WAIT_TIME_MULTIPL

       
    #if defined DEBUG 
    
    if (sendTimer.poll((10000 * WAIT_TIME_MULTIPLY))) {// period sending set multiply by wait_time value ~ 3 sek procesing all and send 10 sek is a default multiply 1 time betwen sending measures change WAIT_TIME_MULTIPLY for multiply time 
       
       rf12_recvDone();
       delay(20);
       pendingOutput = produceOutData();
      
   
   
        //#ifdef DEBUG
        //Serial.println("Sleep break ");
        //Serial.println();
        //#endif
        //rf12_sleep(RF12_WAKEUP);
        //delay(2);
        //produceOutData();
        // rf12_recvDone;

       
       #ifdef LED_ON
        activityLed(0);
       #endif

    }
    
    #endif

        

  #endif

//Sending data periodicaly - for 0.3 sek not be able to get commands 

   #ifndef DEV_MODE   //STOP SENDING DATA 
    //rf12_recvDone;
    if (pendingOutput && rf12_canSend()) {
       
        
		delay(10);
        pendingOutput = 0;
        
       #ifdef LED_ON
        activityLed(1);
       #endif
            
        rf12_recvDone();
        delay(20);
        
        doReport();
            
       #ifdef LED_ON
        activityLed(0);
       #endif
       
       //long czas2 = millis(); //sending timer

       //Serial.println();Serial.print((float)(czas2 - czas1)*0.001,2);Serial.println(); //sending timer
    }
#endif
//end standard mode not for BATTERY

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>     
//THERE IS FOR BATTERY ONLY - save energy
#if defined BATTERY_ONLY 


  //#ifdef PULSE_MEASURE 
  //rf12_control(0x8280);
  
    byte count =0;
    
    for (byte i = 0; i < WAIT_TIME_MULTIPLY;i++) {  // maximum time delay is 60000 milis , it is multiply by WAIT_TIME_MULTIPLY sending duration
     //send data if there are some
    rf12_sleep(RF12_SLEEP); //1,7 uA power consumption
    //send_when_battery ;
    
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX; 
  
  
  power_adc_disable();
  power_spi_disable();
  //#ifndef DEBUG
  //power_usart0_disable();
  //#endif
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();  
  
  Sleepy::loseSomeTime(60000); // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)
    //1 minute should be 60000 but is not because of variation of internal time source
    //caution parameter cannot be more than 65000, maybe find better solution
    //due to internal time source 60000 is longer than 1 minute. so 55s is used. external 60000
  
  
  ADCSRA=oldADCSRA;         // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
  
  power_adc_enable();
  power_spi_enable();
  //#ifndef DEBUG
  //power_usart0_enable();
  //#endif
  power_timer1_enable();
  power_timer2_enable();
  power_twi_enable();



   ++count;
  
  delay(10);
     if (count == WAIT_TIME_MULTIPLY) {
      rf12_sleep(RF12_WAKEUP);
      delay(100);
      pendingOutput = produceOutData();
       
       #ifdef LED_ON
        activityLed(0);
       #endif
       
      rf12_recvDone();
      delay(20);
      count =0;
   
     }
  }





 //send data if there are some

if (pendingOutput && rf12_canSend() ) {
       
       #ifdef LED_ON
        activityLed(1);
       #endif
  
       pendingOutput = 0;
       rf12_recvDone();
  	   delay(20);
       doReport();
            
       #ifdef LED_ON
        activityLed(0);
       #endif
       
       //rf12_sleep(RF12_SLEEP); //1,7 uA power consumption
       rf12_sleep(RF12_SLEEP); 
       delay (2000);
       

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
