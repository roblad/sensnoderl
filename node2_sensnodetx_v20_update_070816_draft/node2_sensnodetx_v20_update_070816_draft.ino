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

  #define VCC_OK 3200 // enough power for normal 1-minute sends
  #define VCC_LOW 3100 // sleep for 1 minute, then try again
  #define VCC_DOZE 2900 // sleep for 5 minutes, then try again
// sleep for 60 minutes, then try again
  #define VCC_SLEEP_MINS(x) ((x) >= VCC_LOW ? 1 : (x) >= VCC_DOZE ? 5 : 60)
  #define VCC_FINAL 2800 // send anyway, might be our last swan song

//http://interface.khm.de/index.php/lab/interfaces-advanced/sleep_watchdog_battery/
//http://donalmorrissey.blogspot.com/2010/04/sleeping-arduino-part-5-wake-up-via.html
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h> 
// avr sleep instructions
#include <avr/wdt.h>



#endif

// lib for RFM12B from https://github.com/jcw/jeelib
//#define RF69_COMPAT 1 //for RF69
#include <JeeLib.h>


//EEPROM WRITE
#include <EEPROM.h>


//*************************************************************


#include "SENSSTRUCTPAYLOAD.h"
//txpower RF12b control variable
byte txPower = TXPOWER;   //0 -max power, 7 -minimum there is last byte of comand control rf12_control(0x9850) -means max, rf12_control(0x9857); -means min -21dbm


//set mili timer for sent package periodicaly and switch to receiving mode  //sending timer
MilliTimer sendTimer;   // see https://github.com/jcw/jeelib/blob/master/examples/RF12/timedSend/timedSend.ino
//MilliTimer rainTimer;   //rain check timer
//add additional control timer variables
//static
volatile byte pendingOutput;
volatile unsigned int adcreading;  //for bang up voltage
// Pulse counting settings
#ifdef PULSE_MEASURE 
 

volatile unsigned long pulseCount = 0;                                           // Number of pulses, used to measure gass 0,01 m3 for each.
volatile unsigned long lastBlink = 0;


volatile byte flag_pulse = 0;
#endif

//set debug for RS printing from node side
#ifdef DEV_MODE
#define DEBUG
#endif

#ifndef DEBUG_BAUD
#define DEBUG_BAUD 9600
#endif

#ifdef NEW_REV //3.4
#define LDR_PIN 2 //standard 2 nano 6
#define BAT_VOL 3 //standard 3 nano 7
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




//#define LOW LOW
//WDT set for uno and specyfic bootloaders - ned to be werified whether Watchdog working on
ISR(WDT_vect) { Sleepy::watchdogEvent(); }


//port for jeenode compatibility
#ifdef NEW_REV
Port p1 (1); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p2 (2); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p3 (3); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p4 (4); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20

#endif


/*
http://aeroquad.com/archive/index.php/t-1064.html?s=1569d8a6149fdf30b10ecb13926e062b

tool for find everything

http://cbxdragbike.com/arduino/arduino_exp.html

*/



//***************************SETUP BEGIN*********************//


void setup() {


//setings BAND - declared in setup
#if RFM69 == 1
   #define RF69_COMPAT 1
#else
   #define RF69_COMPAT 0
#endif


//setings BAND - declared in setup
#if BAND == 433
  rf12_initialize(NODEID, RF12_433MHZ, NETWORK);
#endif
#if BAND == 868
  rf12_initialize(NODEID, RF12_868MHZ, NETWORK);
#endif
  rf12_control(0xC040); // 2.2v low
#if LOWRATE
  rf12_control(0xC623); // ~9.6kbps
#endif

#if defined LOWRATE && defined RFM69
  rf12_control(0xC623); // ~9.6kbps
#endif

rf12_control(0x9850 | (txPower > 7 ? 7 : txPower)); //power control 0 means max and maximum power consumption
//rf12_sleep(RF12_SLEEP);   //sleep RF module
//rf12_control(0x8280);

//define INT1 in pin 3
#ifdef PULSE_MEASURE 
//digitalWrite(3, HIGH);
pinMode(3, INPUT_PULLUP);
//_PULLUP
digitalWrite(3, HIGH);

attachInterrupt(digitalPinToInterrupt(3), onPulse,FALLING); 
//FALLING FALLING
measure.pulse=0;
UpdateEEPROM();

//FALLING
#endif


#ifdef DEBUG
  Serial.begin(DEBUG_BAUD);
#endif

#if !defined DEBUG && defined PULSE_MEASURE

Serial.begin(DEBUG_BAUD);

#endif

} //end setup

//Functions declaration




//common functions
/*
void doReport() {
  //while (!rf12_canSend());
  rf12_recvDone();
  delay(10);
  
  rf12_sendNow(BASEDNODE, &measure, sizeof measure); //send in sync mode last partametr 1
  rf12_sendWait(RADIO_SYNC_MODE);
  rf12_recvDone();
  delay (2);
  activityLed(1);

       if (RF12_WANTS_ACK) {
         if (rf12_canSend()){ 
         rf12_sendStart(RF12_ACK_REPLY,0,0);
         rf12_sendWait(RADIO_SYNC_MODE);
         rf12_recvDone();
         delay (2);
         }
       }
       #ifdef SECONDNODE
    
       
       rf12_sendNow(SECONDNODE, &measure, sizeof measure);
       rf12_sendWait(RADIO_SYNC_MODE);
       rf12_recvDone();
       delay (2);

      if (RF12_WANTS_ACK) {
         if (rf12_canSend()){ 
         rf12_sendStart(RF12_ACK_REPLY,0,0);
         rf12_sendWait(RADIO_SYNC_MODE);
         rf12_recvDone();
         delay (2);     
         }
       }
 
     #endif
   activityLed(0);

}
*/
void doReport() {

    #ifdef LED_ON
    activityLed(1);
    #endif
 
       while (!rf12_canSend())    // wait until sending is allowed
       rf12_recvDone();      // process any incoming data (in the background)
      //if (rf12_canSend()) {
       delay(50);
               rf12_sendStart(BASEDNODE, &measure, sizeof measure);  //sync mode parameter 1 last parameter set
               rf12_sendWait(RADIO_SYNC_MODE);
               rf12_recvDone(); 
               
              if (RF12_WANTS_ACK) {
              rf12_sendStart(RF12_ACK_REPLY,0,0);
              delay (2);
               //rf12_recvDone(); 
              
             }

     
    
   
  #ifdef SECONDNODE
       if (rf12_canSend()){
       rf12_sendStart(SECONDNODE, &measure, sizeof measure);
       rf12_sendWait(RADIO_SYNC_MODE);
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

//reference voltage bangup
long readVcc() {
  long resultVcc; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(20); // Wait for Vref to settle - 2 was inadequate
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  resultVcc = ADCL;
  resultVcc |= ADCH<<8;
  resultVcc = 1125300L / resultVcc; // Back-calculate AVcc in mV 1125300L 1126400L
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
  return (battvol * 1000)-34;
}
//LED activation
void activityLed (byte on) {
  pinMode(ACT_LED, OUTPUT);
  digitalWrite(ACT_LED, on);
  delay(50);
}

//get values and set for transmission, DEBUG print to serial
void transmissionRS() {
#ifdef DEV_MODE
  Serial.println("==DEV MODE==");
  delay(2);
#endif
  
  //activityLed(1);
  Serial.println(' ');
  delay(2);
  #if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING 
  Serial.print("TEMP ");
  delay(2);
  #endif
  //internal temp *
  #if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING && !defined  DHT_SENSOR && !defined SHT21_SENSOR  
  Serial.print("INTERNAL ");
  delay(2);
  #endif

  Serial.println((float)measure.temp*0.1,2);
  delay(2);
  
 
  //Serial.print("LOBAT " );
  //Serial.println(measure.lobat, DEC);
  //delay(2);
  Serial.print("BATVOL ");
  delay(2);
  Serial.println((float)measure.battvol/1000,3);
  delay(2);
  Serial.println(' ');
  delay(2);
 #ifdef PULSE_MEASURE 
        
          Serial.print("PULSE COUNT TO SEND: ");
          delay(2);
          Serial.println((unsigned long)measure.pulse,DEC);
          delay(5);
          Serial.println(' ');
          delay(2);
  #endif
           
}
//get measure for sending
void doMeasure() {       //main measure
  
  adcreading = 0;
  measure.lobat = 0;//rf12_lowbat();  //probably if RF 69 it should be commented and measure should be changed 
  
  battVolts();
  delay (5);
  measure.battvol = battVolts();



//set DS main temperature, default DS sensor 0, you can change it to any


#if !defined DS_AS_MAIN_TEMP && !defined DS_COUNT  && !defined DS_BY_ADDRESS_GETTING && !defined  DHT_SENSOR && !defined SHT21_SENSOR
 
 measure.temp = readTemp()/1000;
 
#endif

}
//end common functions

//confirmation for payload receiving
void receiveRXrelayPayload () {

//#ifdef DEBUG

if (rxdata.destnode == NODEID && rxdata.receivedvalue != 0) 
 {
 Serial.println("Proccessing commands received for pulse value. ");delay(2);
 }else{
 Serial.println("Nothing to process after data receiving. ");delay(2);
 }

//#endif



if (rxdata.destnode == NODEID && rxdata.receivedvalue != 0) {
 
 //EEPROM WRITE
 //write value to EEPROM in adres 0 unsigned long 4 bytes adres 0 1 2 3
    //cli();  //disable global interputs
    while (!eeprom_is_ready());  //wait till EEPROM will be ready
    EEPROM.put(0, rxdata.receivedvalue);
    delay (5);
    //sei(); //enable global interputs 
 //EEPROM WRITE   
  measure.pulse=rxdata.receivedvalue;
   
   delay (2);
   Serial.print("VALUES FROM BASENODE SET FOR PULSE VALUE: ");
   delay (2);
   Serial.println(measure.pulse);
   delay (2);
   Serial.println("VALUE FROM BASENODE WRITEN TO EEPROM");
   delay (2);
   
 }

}

//checking if data came
void CheckIfdataCome () {

  //#ifdef DEBUG
    Serial.println();
    Serial.print("RX:");
    delay(2);
    Serial.print("|");
    delay(2);
    Serial.print((int) rf12_hdr);
    delay(2);
    for (byte i = 0; i < rf12_len; ++i) {
    Serial.print("|");
    delay(2);
    Serial.print((int) rf12_data[i]); //data print binary
    }
    delay(2);
    Serial.println("|  Receiving data from basenode to process for node: ");
    delay(2);
    Serial.println((int) rf12_data[0]); //data print binary
    delay(2);
 //#endif 
}

//sending/printing  process function 
void sendPayload() {

#ifdef LED_ON
  activityLed(1);
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
  result = ((result - 125) * 1075);; //Don't forget to change "x" with multiplication sign

  return result; 
  
}
#endif


/*

void coderelayStatusConfirmation(){
    #ifdef LED_ON
    activityLed(1);
    #endif
    rf12_recvDone();
    delay(10);
    if (rf12_canSend()) {
    rf12_sendStart(BASEDNODE, &measure, sizeof measure);  //sync mode parameter 1 last parameter set
    rf12_sendWait(RADIO_SYNC_MODE);
    rf12_recvDone();
    delay (2);
     if (RF12_WANTS_ACK) {
     rf12_sendStart(RF12_ACK_REPLY,0,0);
     rf12_sendWait(RADIO_SYNC_MODE);
     rf12_recvDone();
     delay (2);
     //rf12_sendWait(1);
     }

    }
 

    #ifdef LED_ON
    activityLed(0);
    #endif

}
*/

void coderelayStatusConfirmation(){
    #ifdef LED_ON
    activityLed(1);
    #endif
            while (!rf12_canSend())    // wait until sending is allowed
            rf12_recvDone();      // process any incoming data (in the background)
      //if (rf12_canSend()) {
            delay(50);
               rf12_sendStart(BASEDNODE, &measure, sizeof measure);  //sync mode parameter 1 last parameter set
               rf12_sendWait(RADIO_SYNC_MODE);
               rf12_recvDone();
               //rf12_sendWait(1);
                   if (RF12_WANTS_ACK) {
                    rf12_sendStart(RF12_ACK_REPLY,0,0);
                    delay (2);
                    //rf12_recvDone();
                    
                    }
                 
           // }




    #ifdef LED_ON
    activityLed(0);
    #endif


}



// Main loop

void loop() {

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>     
//THERE IS FOR BATTERY ONLY - save energy
#if defined BATTERY_ONLY && defined PULSE_MEASURE


  if (rf12_recvDone() && rf12_crc == 0 && ((int) (rf12_hdr & 0x1F)) == BASEDNODE && rf12_len == sizeof rxdata && ((int) rf12_data[0]) == NODEID ) { //&& rf12_len == sizeof rxdata
     
    //if (((int) rf12_data[0]) != NODEID) return;
         memcpy(&rxdata, (void*) rf12_data, sizeof rxdata ); 
         delay (5);

         consumeInData (); 
         delay (10);
    #ifdef LED_ON
    
    activityLed(1);
    delay(200);
    activityLed(0);
    
    activityLed(1);
    
    activityLed(0);

    #endif
         coderelayStatusConfirmation(); 
    }
  //set time when waiting for incomming data
if (sendTimer.poll(8000)) {
     //for (byte i = 0; i < WAIT_TIME_MULTIPLY;i++) {  // maximum time delay is 60000 milis , it is multiply by WAIT_TIME_MULTIPLY sending duration
     //send data if there are some
    
    //rf12_recvDone();
    rf12_sleep(RF12_SLEEP); //1,7 uA power consumption
    //send_when_battery ;
    
    
  
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    //SLEEP_MODE_ADC);
    //SLEEP_MODE_EXT_STANDBY
    //SLEEP_MODE_PWR_SAVE
    //SLEEP_MODE_ADC
    //SLEEP_MODE_STANDBY);
    //SLEEP_MODE_PWR_DOWN);
      #ifdef DEBUG
      Serial.print("sleep mode");
      Serial.println();
      delay(20);
      #endif
    
    power_adc_disable();
    power_spi_disable();
    power_usart0_disable();
    //power_timer0_disable();  //timer 0 TCNT0 needed for debounce function
    //power_timer1_disable();
    //power_timer2_disable();
    power_twi_disable();  
      

    sleep_enable();
    #ifdef LED_ON
    activityLed(1);
    delay(100);
    activityLed(0);
    delay(100);
    activityLed(1);
    delay(100);
    activityLed(0);
    delay(100);
    activityLed(1);
    delay(100);
    activityLed(0);
    #endif

    //Sleepy::powerDown();   // sometimes something break WDT Sleepy::loseSomeTime(60000);

    sleep_mode();
      
    /* The program will continue from here. */
    
    /* First thing to do is disable sleep. */
    

    sleep_disable();
    delayMicroseconds(1500); 
    power_all_enable();
    cli();  //disable global interputs
    
    delay(20);
    rf12_sleep(RF12_WAKEUP);
    delay(50);

    #ifdef DEBUG
      Serial.print("pulseCount: "); delay(2);Serial.println(pulseCount);delay(2);
      Serial.print("flag_pulse: "); delay(2);Serial.println(flag_pulse);delay(2);
    #endif
           
    
   
   //}


      #ifdef DEBUG
       Serial.print("Sleep break after get some interput, pulse is: "); delay(2);Serial.println(flag_pulse);
      #endif
   if (pulseCount > 0 && flag_pulse == 1) {
      
      //used for pulse measure 
      rf12_sleep(RF12_WAKEUP);
      delay(50);
      

      UpdateEEPROM (); 
      pulseCount = 0;
      flag_pulse = 0; 
      delay(5);
      sei(); //enable global interputs
      delay(5);
      pendingOutput = produceOutData();
      delay (5);
   } else {
    rf12_sleep(RF12_SLEEP); //1,7 uA power consumption
    sei(); //enable global interputs
   }

}

//send data if there are some

if (pendingOutput) {
      //detachInterrupt(digitalPinToInterrupt(3)); 
      pendingOutput = 0;
      delay(5);
      //rf12_control(0x82FE);
      //delay(5);
      doReport(); 
      delay(20);
      //long czas2 = millis();
      //Serial.println();Serial.print((float)(czas2 - czas1)*0.001,2);Serial.println();


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
     
// data to send use node ID ex. 02 and 3 times zeros from the begining and space, after that type value to transmit to node (currently used for pulse)max value  4294967295 
 
#ifdef PULSE_MEASURE    
     char incomingByte;  // for data after getting data from serial 
     //unsigned long values = 0;
     measure.pulse = 0;
     while(1) {
        
         //incomingByte = Serial.read();
         //cli();//disable global interputs
           
          incomingByte = Serial.read();
          if (incomingByte == '\n') break;   // exit the while(1), we're done receiving
          if (incomingByte == '\r') break;   // exit the while(1), we're done receiving
          
          if (incomingByte == 32) continue;  // if space read() 
          if (incomingByte == -1) continue; // if no characters are in the buffer read() returns -1
         measure.pulse *= 10;  // shift left 1 decimal place
          // convert ASCII to integer, add, and shift left 1 decimal place
         measure.pulse = ((incomingByte -48) + measure.pulse);
         //values = ((incomingByte) + values);
         //Serial.println(measure.pulse);
    }
   
         //Serial.flush(); //clear serial data 
         rxdata.receivedvalue = measure.pulse;
           //Serial.println(rxdata.receivedvalue);
      //sei(); //enable global interputs         
delay (10);
#endif

 
 

    #ifdef PULSE_MEASURE   
       if (measure.pulse!= 0) { 
          Serial.println();
          delay (2);
          Serial.print("RECEIVED PULSE VALUE: ");
          delay (2);
          //Serial.print(values);
          Serial.print(' ');
          delay (2);
          Serial.print(rxdata.receivedvalue);
          delay (2);
          Serial.print(' ');
          delay (2);
       }

      doMeasure(); //coment it for basenode
      cli(); 
      UpdateEEPROM();
      sei(); //enable global interputs
      coderelayStatusConfirmation(); //coment it for basenode
    #endif
    }   
 
}

#if defined PULSE_MEASURE
//used for pulse measure 
//EEPROM UPDATE FUNCTION
unsigned long UpdateEEPROM () {
 unsigned long getvalueEEPROM=0;
   //noInterrupts();
   //delayMicroseconds(1500);
   //cli();  //disable global interputs
    while (!eeprom_is_ready()); //wait till EEPROM will be avaliable
      //cli();//disable global interputs
      EEPROM.get(0,getvalueEEPROM);
      delay(10);
      //sei(); //enable global interputs
    if (measure.pulse/10 > getvalueEEPROM/10){
      
      
      #ifdef DEBUG
      Serial.println();
      Serial.print("Checked pulse value / 10 - previous value stored in EEPROM: ");
      delay(2);
      Serial.println(getvalueEEPROM);
      delay(2);
      Serial.print("Pulse value to be stored in EEPROM: ");
      delay(2);
      Serial.println(measure.pulse);
      delay(2);
      #endif
      
      while (!eeprom_is_ready()); 
      //cli();//disable global interputs
      EEPROM.put(0, (unsigned long)measure.pulse);
      delay(10);
      //sei(); //enable global interputs
      while (!eeprom_is_ready()); 
      //cli();
      EEPROM.get(0,getvalueEEPROM);
      delay(10);
      //sei();
      #ifdef DEBUG

      Serial.print("Pulse value has been stored in EEPROM: ");
      delay(2);
      Serial.println(getvalueEEPROM);
      delay (2);
      #endif
      //pendingOutput = produceOutData();
      //coderelayStatusConfirmation();
  } else {

      
      while (!eeprom_is_ready()); 
      //cli();
      EEPROM.get(0,getvalueEEPROM);
      delay(10);
      //sei();
      
      if (measure.pulse < getvalueEEPROM){
      
      #ifdef DEBUG
      Serial.println();
      Serial.print("Checked pulse value, lower than in EEPROM, EEPROM value: ");
      delay(2);
      Serial.println(getvalueEEPROM);
      delay(2);
      Serial.println();
      Serial.print("Checked pulse value: ");
      delay(2);
      Serial.println(measure.pulse);
      delay(2);
      #endif 
      if (measure.pulse < 10){
      measure.pulse = getvalueEEPROM + measure.pulse;
      }
      while (!eeprom_is_ready()); 
      //cli();//disable global interputs
      EEPROM.put(0, (unsigned long)measure.pulse);
      delay(10);
      //sei(); //enable global interputs

      #ifdef DEBUG
      Serial.print("Current pulse value set from EEPROM: ");
      delay(2);
      Serial.println(measure.pulse);
      delay(2);
      #endif
      
      //pendingOutput = produceOutData();
      //coderelayStatusConfirmation();


      }
  }

      while (!eeprom_is_ready()); 
      //cli();
      EEPROM.get(0,getvalueEEPROM);
      delay(10);
      //sei();
  //interrupts();
  //delayMicroseconds(1500);
  return getvalueEEPROM; 
  
  //sei(); //enable global interputs   
}

//pulse function
void onPulse() {
     
       //noInterrupts();
       //delayMicroseconds(1500);
       
       if ((long)(micros() -lastBlink)>600000L) { // Sometimes we get interrupt on FALLING
                     
                    lastBlink = micros();
                    delay(5);
                    cli();//disable global interputs
                   //unsigned long test = UpdateEEPROM();
                   #ifdef DEBUG
                   //Serial.print("Current value in EEPROM: ");delay(2);// Serial.println(test);
                   #endif                      
          //UpdateEEPROM();
          ++measure.pulse;
          
          pulseCount++;           //pulseCounter 
               
          flag_pulse = 1;
          delay(5);
          sei(); //enable global interputs
          
          #ifdef DEBUG

            Serial.print("Current measure: ");delay(5); Serial.println(measure.pulse);
            delay(5);
          #endif
          

              
       }
       //interrupts();
       //delayMicroseconds(1500);
}

#endif


