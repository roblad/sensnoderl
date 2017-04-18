// sensbase v2.0 artekw
// v2.1 -for  RL - added powermon, level tank, DS measurement for many DS devices with address control, converter for readable data
//add option for other RTC
//03.0515 - change all strings to values, remove print confirmation for receiving commands for relays, change baut rate to 9600 change "|" separator to " ", changed txemon period for 1 min (needed from txemon side update to sending every 1 min)
//05.2015 - added switches for PCF and correct send/receive comands for node cleanup code
//TODO gas and water pulse measure
//TODO CH4 detection
//TODO GLCD AND buttons control 
//TODO EEPROM set
//TOD BOOT for remote soft upgrade
//***********************************************************************************************************************
/*
NODEID 1 - relays garage all sensors  see NODE1 config - exahaust gas owen measure through I2C fan control , pulse to be checked
NODEID 2 - pulse gas sensor battery only NODE2 config
NODEID 3 - pulse water measure
NODEID 4 - outdoor
NODEID 10 - emon and tank level see NODE10 config

SEND TO NODES - use for relay 01011 -means node 01 relay 01 state 1
              - for pulse 02 space 1234567890  , it will send to node 2 values for pulse count i.e. gas measure
*/
//#define uint8_t byte  - for som libs needed


#include "configuration.h"
#if RFM69 == 1
   #define RF69_COMPAT 1
#else
   #define RF69_COMPAT 0
#endif

#include <JeeLib.h>
#include <stdio.h> //adding additional functions to program from AVR
#include <avr/sleep.h>
#include <Wire.h>   //I2C communication with RTC DS1370 with eeprom chip
/**************************DEFINE 2 types of RTC without changing code ******************************************************/
#include <OneButton.h> // one buton functions to operate in functions (sensbase GOLD) http://www.mathertel.de/Arduino/OneButtonLibrary.aspx 
//http://majsterkowo.pl/zegar-ds1307/ - zegar
//see http://www.jarzebski.pl/arduino/komponenty/zegar-czasu-rzeczywistego-rtc-ds1307.html
#if defined DS1307_CLOCK && !defined DS3231_CLOCK
//#include "DS1307.h" //RTC DS1370 with eeprom chip
//https://github.com/jarzebski/Arduino-DS1307
#endif
#if defined DS3231_CLOCK && !defined DS1307_CLOCK
//http://www.jarzebski.pl/arduino/komponenty/zegar-czasu-rzeczywistego-rtc-ds3231.html
//#include "DS3231.h"
//https://github.com/jarzebski/Arduino-DS3231    
#endif    
//port for jeenode compatibility
#ifdef NEW_REV
Port p1 (1); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p2 (2); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p3 (3); // JeeLabs Port P1 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20
Port p4 (4); // JeeLabs Port P2 - nobody now for what reason it is - when you will use different board than clon of jeelab it can be comented probably, it need to be checked it is probably pin definition from 0 to 20

#endif

byte needToSend;
//txpower RF12b control variable //need to be tested for RFM69
byte txPower=0;   //0 -max power, 7 -minimum there is last byte of comand control rf12_control(0x9850) -means max, rf12_control(0x9857); -means min -21dbm
// structure of data
// set in SENSSTRUCTPAYLOAD.h
// *Payloadrx for sending comands to nodes
#include "SENSSTRUCTPAYLOAD.h"
/*****************   DEBUG SET ************************************/
#ifndef DEBUG_BAUD
#define DEBUG_BAUD 9600
#endif

//RTC CLOCK SET
#ifdef RTC_CLOCK // clock start declaration for 2 of types RTC - set in code is the same

//see http://www.jarzebski.pl/arduino/komponenty/zegar-czasu-rzeczywistego-rtc-ds1307.html
  #if defined DS1307_CLOCK && !defined DS3231_CLOCK

   DS1307 clock;
   RTCDateTime dt;
  
  #endif
  #if defined DS3231_CLOCK && !defined DS1307_CLOCK
  
    DS3231 clock;
    RTCDateTime dt;

  #endif    
#endif

//bit control for sending post fixed data and decoding it in receiver side

byte dsSensoramount;
byte phaseAmount;

// controlbits 12 byte after fixing 11 bytes in structure - if you want to extend fixed data block above 11 change sequence of control bits variable
#if defined NSENSORS && NSENSORS > 0
// emon variables 
//int lastSampleTime_kwh, sampleDuration_kwh; //collect consumption kWh used for complete sample getting for emon
//long lastmillis_kwh;   //time
static float realPowersumcollect; //temp variable for summarizing power
static float apparentPowersum; //local variable for apparent power summarizing
#endif

/********************************************** START SETUP *******************************************/
void setup () {

//set clock when declared
#ifdef RTC_CLOCK  

#ifdef DS3231_CLOCK
   clock.begin();
  
   //clock.setDateTime(__DATE__, __TIME__);
   //Wire.beginTransmission(0x68); // address DS3231
   //Wire.write(0x0E); // select register
   //Wire.write(0b00011100); // write register bitmap, bit 7 is /EOSC
   //Wire.endTransmission();
  
#else

   clock.begin();
   //clock.setDateTime(__DATE__, __TIME__);
#endif
       // Ustawiamy date i czas z kompilacji szkicu
       //clock.setDateTime(__DATE__, __TIME__);
  
 if (!clock.isReady())
    {
    // Set sketch compiling time
    clock.setDateTime(__DATE__, __TIME__);
  
    //clock.setDateTime(__DATE__, __TIME__);

    // Set from UNIX timestamp
    // clock.setDateTime(1397408400);

    // Manual (YYYY, MM, DD, HH, II, SS
    // clock.setDateTime(2014, 4, 13, 19, 21, 00);
    }

    dt = clock.getDateTime(); 
  //set date/time formats clock when declared
  /************************************************************************/
  /*
   Serial.print("Long number format:          ");
   Serial.print("Aktualny czas:          ");
   Serial.println(clock.dateFormat("d/m/Y H:i:s ,l", dt));
   Serial.println();
   Serial.print("Long format with month name: ");
   Serial.println(clock.dateFormat("d F Y H:i:s",  dt));
   Serial.print("Short format witch 12h mode: ");
   Serial.println(clock.dateFormat("jS M y, h:ia", dt));
   Serial.print("Today is:                    ");
   Serial.print(clock.dateFormat("z ", dt));
   Serial.print(" days of the year.");
   Serial.print("Actual month has:            ");
   Serial.print(clock.dateFormat("t", dt));
   Serial.println(" days.");
   Serial.println();
   Serial.print("Unixtime:                    ");
   Serial.println("Timestamp:                    ");
   Serial.print(clock.dateFormat("U", dt));
   Serial.println();
   */
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

//serial and LED set
Serial.begin(DEBUG_BAUD);
pinMode(ACT_LED, OUTPUT);

}
/*********************************** END SETUP ************************************/

//function receiving data
static void ReceivingData () {

 if (rf12_recvDone() & rf12_crc == 0) {
 // check CRC correction receiving
     if (RF12_WANTS_ACK) {
      rf12_sendNow(RF12_ACK_REPLY,0,0);
     }
   activityLed(1);

#ifdef BINARY
  
    //test type of data to be printed to serial
    //set data output BINARY 1 - binary data 0- data converted according to global structure
    //print binary data (not recommended there are float variables that conversion is difficult)
    printBinaryData ();
    activityLed(0);

#else
    // Set structure for measure from rf12_data //map structure for converter data 
    measure = *(Payload*) rf12_data;
    // printing converted data  
    printConverter ();
    activityLed(0);
#endif    

 }

}

//functions set


//set data output BINARY - data convert according to global structure
//function binary data
static void printBinaryData () {

       //Serial.print(' ');  //tu k...wa byl blad przez ... i jego konwertery pytongowe
       Serial.println();
   
#ifdef RTC_CLOCK
       Serial.print(clock.dateFormat("U", dt));
       Serial.print(' ');
#endif

      if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0) // and no rf errors
      {
       Serial.print("OK");   
      }else{
       Serial.print("ERR");
       return;
      }
       Serial.print(' ');
     
       Serial.print((int) rf12_hdr);  
       if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) != 0) return;

        for (byte i = 0; i < rf12_len; ++i) {  //Serial.print(' ');
          
          Serial.print((int) (rf12_data[i]));
          Serial.print(' ');
       }
}

//1st fixed block print in converter
static void printfixedblock () {
 byte test_node = ((int) (rf12_hdr & 0x1F));
      //Serial.print(' ');   //tu k...wa byl blad przez ... i jego konwertery pytongowe
       Serial.println();   
#ifdef RTC_CLOCK
       Serial.print(clock.dateFormat("d.m.Y H:i:s", dt));  //time stamp for node last record 
       Serial.print(' ');
#endif

      if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0) // and no rf errors
      {
       Serial.print("OK");   
      }else{
       Serial.print("ERR");
       
      }
       Serial.print(' ');
       
       Serial.print((int) (rf12_hdr & 0x1F));  //node ID
       
       Serial.print(' ');
       if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) != 0) return; // break when error receiving occured
      
       if (test_node != 2 && test_node != 10  ) { //add additional nodes that this block will skiped
       Serial.print((float)measure.light/10,1);
       Serial.print(' ');
         if (test_node != 3  ) { 
       Serial.print((float)measure.humi/10,1);
       Serial.print(' ');
         }
       }
       Serial.print((float)measure.temp/10,1);
       Serial.print(' ');
       if (test_node != 2 && test_node != 3  && test_node != 10  ) { //add additional nodes that this block will skiped
       Serial.print((float)measure.pressure/10,1);
       Serial.print(' ');
       }
      if ( measure.lobat == 0) Serial.print(1);
      if ( measure.lobat == 1) Serial.print(0);
       Serial.print(' ');
      
       Serial.print((float)measure.battvol/1000,3);
       Serial.print(' ');
}


// function to print DS in converter
static void printDSfixedblock () {
if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) != 0) return; 
// print DS sensors
  

    
     if ((rf12_buf[2]) >= (STRUCT_FIXED_BLOCK) && ((((dsSensoramount) * 2) + (STRUCT_CONTROL_BYTE)) + STRUCT_FIXED_BLOCK) <= (rf12_buf[2]) &&  (dsSensoramount) <= (DS_COUNT) && (dsSensoramount) > 0) {
       
        Serial.print((byte)dsSensoramount);
        Serial.print(' ');
       for (byte i = 0; i < dsSensoramount; ++i) {  
        Serial.print(((measure.temp_DS[i])*0.01));
        delay(1);
        Serial.print(' ');
       }
    }
  
}


//function for emon converter "Power measurment"
#if defined STRUCT_NSENSORS && defined NSENSORS && NSENSORS >0
static void printEmon () {
if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) != 0) return; 
//clear sum data
 apparentPowersum=0;
 realPowersumcollect=0;

    for(int i=0; i < phaseAmount; ++i) {

     Serial.print((i)+1); 
     Serial.print(' ');
     Serial.print(float((measure.sensor[i].Irms)*0.01));
     Serial.print(' ');
     Serial.print(float((measure.sensor[i].powerFactor)*0.01));
     Serial.print(' ');
     Serial.print(float(measure.sensor[i].realPower));
     Serial.print(' ');
     Serial.print((float)((measure.sensor[i].realPower)/((measure.sensor[i].powerFactor)*0.01)),2);
     Serial.print(' ');
     realPowersumcollect += measure.sensor[i].realPower;
     apparentPowersum += ((measure.sensor[i].realPower)/((measure.sensor[i].powerFactor)*0.01));
    }
    
   if (phaseAmount) {
     Serial.print((((measure.Voltage)*0.01)+3)); // 3- voltage calibration from basenode +3v diferernces from multimeter measure
     Serial.print(' ');
     Serial.print(float(realPowersumcollect));
     Serial.print(' ');
     Serial.print(float(apparentPowersum));
     Serial.print(' ');
    //lastSampleTime_kwh = millis();     // Store time for kwh calc
    //sampleDuration_kwh = ((millis() - lastSampleTime_kwh)/1000); // Sample duration in seconds for counting kWh
     Serial.print(float(((((realPowersumcollect)*1.0155)*0.001))/60.25),4); //actual consumption kWh for every 1 min RWE measure apparent power consum , because of period of sent sum gives kWh - here is multiply x 60 for 1h +0.25 s for basenode loop delay, 1.0155 - voltage calibration from basenode.
    //100000)/3600)* sampleDuration_kwh)); //actual consumption kWh
     Serial.print(' ');
    }else{
     Serial.print(0);
     Serial.print(' ');
   }
}

#endif


#if defined STRUCT_RELAY_STATUS && defined RELAY_AMOUNT
//function decode relay status for node in converter 
static void readRelaystatus () {
if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) != 0) return; 
byte relay_ammount_received =0;
relay_ammount_received =((measure.relaystatus & 0xF000) >>12 );  //removed relays data 4096 and shifted bits to first 4 with amount of relays binary coded 

byte RelayValue[relay_ammount_received];
// do not forgot set variables for relays as byte
// set all values for relays expander/expanders from sender side
// 12 bits for status of relays and 4 bits for amount of relays needed i could be 2 expanders with 6 relays each 
// maax 12 relays status avaliable 
// Serial.println("PCF8574 real value is opposite logic value I/O - when PIN set to O value get from output is 1, printed value is real status of relay");
  
unsigned int relays_modulo = measure.relaystatus;

if (measure.relaystatus >=4096) {  //check if received amount of relays is >0
      
//decoding coded values received modulo by 2 (gives 0 or something - represents received ststuas LSB in reverce order - changed by comparison ==1? when 1 asign 0 in other hand 1, after that is devidedby 2 to lost digits after dot ) amount of relays, values printed are according to amount of relays received from right status for relay, when switched on it is represented by 1 when switched off it shows 0.
      for (byte i=0; i < (relay_ammount_received); i++) { 
                   
       RelayValue[i] = (((relays_modulo)%2) ==1? 0:1) ; relays_modulo/=2;
      
      delay(5); 
    }
    
// RelayValue[11] = (measure.relaystatus & 2048) //max 12 relays coded in bits from 1 - 12 max data transfered in 2 bytes.
// ...
// RelayValue[2] = (measure.relaystatus & 4) ? 1:0;
// RelayValue[1] = (measure.relaystatus & 2) ? 1:0;
// RelayValue[0] = (measure.relaystatus & 1) ? 1:0;  //value 4096
// all above values gives 12 bits representation and are shown in converter in oposite status 
    
     Serial.print (relay_ammount_received);  //decoded number of relays status received from node
     Serial.print (' ');
  
     for (byte i=0; i < (relay_ammount_received); i++) {   // print status for each [n] relays 
     
     Serial.print(RelayValue[i]); 
     Serial.print(' ');

     }

   } else {
     
     // print  field with number of relays = 0 and 12 fields = 0  /means received [n] relays and status for each is 0
      Serial.print(0); 
      Serial.print(' '); 
   }    

}
#endif

#if defined PULSE_MEASURE
//used for pulse measure 

static void printPulsemeasure () {
if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) != 0) return; // and no rf errors
      
       delay(2);
       if (sizeof(measure.pulse) > 8 )  {measure.pulse=measure.pulse >> 2;};
       Serial.print((float)measure.pulse/100);
       delay(2);
       Serial.print(' ');
}

#endif

//#if defined STRUCT_RESERVERD
// ad maximum 11 bytes size variables in this place, for future reuirements
//#endif


#if defined TANK_LEVEL
//function to print Tank level in converter 
static void printTankLevel () {
if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) != 0) return; 
  
  
  switch (measure.tanklevel) {
   case 0:
      
      Serial.print(1); //FULL
      Serial.print(' ');
      
      
      //add for additional digital pin output for control other stuff
      break;
   case 1:
      Serial.print(2);  //MEDIUM
      Serial.print(' ');
      
      
      //add for additional digital pin output for control other stuff
      break;
   case 2:
      Serial.print(3); //LOW
      Serial.print(' ');
      
      break;
   case 3:
      Serial.print(-1); //ERR sikac do wiaderka
      Serial.print(' ');
      
      break;
   default: 
     Serial.print(0); //  ERR OR NO TANK LEVEL sikac do wiaderka albo padl czujnik
     Serial.print(' ');
     
 }    

} 
#endif



// function for sending commands to nodes  

#ifdef PULSE_MEASURE || defined RELAY_AMOUNT  //curently for relays and pulse - add next if you want get data from basenode
static void DoSendToNode () {
/*   // replaced by serialEvent()
 if (Serial.available() >=5 ) {  // should  be >=5 characters 0 for testing
   
       // RX 03121 | NID CMD STATE //
        needToSend = 1;
    
    
       int rxb = Serial.available();
       for (int i = 0; i<rxb; i++) {
        ramka[i] = (Serial.read()-48);
       }
    
    rxdata.destnode = (ramka[0] * 10) + ramka[1];
    rxdata.cmd = (ramka[2] * 10) + ramka[3];
    rxdata.state = ramka[4];

#ifdef DEBUG 
       Serial.println();
#ifdef RTC_CLOCK
       Serial.print(clock.dateFormat("d.m.Y H:i:s", dt));  //time stamp for node last record 
       Serial.print(' ');
#endif
       Serial.print("SEND NODE ");
       Serial.print(rxdata.destnode);
       Serial.print(" RELAY ");
       Serial.print(rxdata.cmd);
       Serial.print(" STATE ");
       Serial.print(rxdata.state);
       Serial.print(' ');
#endif
 }
 
*/
    if (needToSend && rf12_canSend()) {
     
      needToSend = 0;
      
      rf12_sendStart((rxdata.destnode), &rxdata, sizeof rxdata);  //sync mode parameter 1 last parameter set
          if (RF12_WANTS_ACK) {
          rf12_sendNow(RF12_ACK_REPLY,0,0);
          }
          rf12_sendWait(1); 
          delay(50);
      if (rf12_canSend()){
      rf12_sendStart((rxdata.destnode), &rxdata, sizeof rxdata);  //sync mode parameter 1 last parameter set
      }
       if (RF12_WANTS_ACK) {
          rf12_sendNow(RF12_ACK_REPLY,0,0);
       }else{
          delay (1);
          if (rf12_canSend()){
          rf12_sendNow((rxdata.destnode), &rxdata, sizeof rxdata);  //retransmision for to be sure that command has been received by node
          }
          delay (1);
       }
       digitalWrite(ACT_LED, HIGH);
       delay (5);
       digitalWrite(ACT_LED, LOW);  
    }
}
#endif

//function led
static void activityLed (byte on) {
#ifdef ACT_LED
  digitalWrite(ACT_LED, !on);
  delay(50);
#endif
}



//main function  for converter
static void printConverter () { 
//convert to readable form

 byte test_node = ((int) (rf12_hdr & 0x1F));

//decode control byte decode amount of DS sensors get from node and emon 1 or 3 phase - for converter needed
    phaseAmount=measure.controlbits & 15;
    dsSensoramount= measure.controlbits >> 4;
 // }


//print 1st fixed block compatible with sensmontx
printfixedblock ();
//removed for node 2 - pulsenode

  
//DS temperature print according to received DS_COUNT
#if DS_COUNT
if (test_node != 2 && test_node !=3) { //add additional nodes that this block will skiped
// print DS sensors
printDSfixedblock();
  }
#endif

// emon print
#if defined NSENSORS && NSENSORS > 0

 if (test_node == 10 ) {
printEmon ();
 }
#endif

//relays status print
#if defined STRUCT_RELAY_STATUS && defined RELAY_AMOUNT

 if (test_node != 10 && test_node !=2 && test_node !=3) {
readRelaystatus ();
  }
#endif

#if defined TANK_LEVEL

   if (test_node == 10) {  //node 10 tank
        printTankLevel ();
   }

#endif
   

    if (test_node  == 2 || test_node  == 3) { //add additional nodes that this block will be printed as end block
    printPulsemeasure ();

    }

  
//TODO GLCD

} //end print converter


//main loop
void loop () {
//memset (&measure,0,sizeof (measure));
//memset (&rxdata,0,sizeof (rxdata));
#ifdef RTC_CLOCK 
//time refresh
dt = clock.getDateTime(); 

#endif

//send to node

#ifdef PULSE_MEASURE || defined RELAY_AMOUNT  //curently for relays and pulse - add next if you want get data from basenode

DoSendToNode ();
delay (5);
//DoSendToNode ();

#endif

//main - getting data from nodes rf12_receive
ReceivingData ();  
//digitalWrite(ACT_LED, LOW);   
/*  clock.forceConversion();
  Serial.print(' ');Serial.print(clock.dateFormat("d.m.Y|H:i:s", dt));  //time stamp for node last record 
  Serial.print(' ');
  Serial.print("Basenode temperature: ");Serial.print(clock.readTemperature()-3);
  Serial.println(" *C|");
  delay (1000);
*/
//clear all data received 
//memset(str, 0, sizeof(str)); // clear
//clear old data for structure and fill zeros
//memset (&measure,0,sizeof (measure));  //set all structure for 0 values
//memset ( &payload,0,sizeof(measure) );
//memcpy (&measure,(const void*)rf12_data,sizeof (measure [or can be rf12_buf[2]]))


// Different way to get data for converter
      /*
        //memcpy (&measure,(const void*)rf12_data,sizeof (measure [or can be rf12_buf[2]]));  pomocnicze do kpopiowania znacznikow z pamieci z paczki odbieranej
        Serial.print((float)((Payload*)rf12_data)->light/10,1);
        // Serial.print(((Payload*)rf12_data)->temp_DS[i]/10);  //inne wyswietlanie bezposrednie mapowanie 
        Serial.print(' ');
        Serial.print((float)((Payload*)rf12_data)->humi/10),1;
        Serial.println(' ');
     */
  

}
/************************END MAIN LOOP ****************************/


//this function is simillar to loop and breaks as interput all operations when som data are typed in serial port
void serialEvent() {
 
      // kod do wykonania podczas odbioru danych
if (Serial.available() >=5 && Serial.available()<16 ) {  // should  be >5 characters - digits first is node id  and \n as last or node id xx and 0000 space and max value  4294967295 and \n 
     
#ifdef RELAY_AMOUNT
  //setting variables before structure for relay control 
    char ramka[5];
    rxdata.destnode = 0;
    rxdata.cmd = 0;
    rxdata.state = 0;
         
       // RX 03121 | NID CMD STATE //
        needToSend = 1; //uncoment it for basenode
     
    // char ramka[5]; comment it for basenode
    
    int rxb = Serial.available();  //read values
     for (int i = 0; i<rxb; i++) {
       char test = Serial.read();
       if (test == 32 || test == 13) break;
       
       ramka[i] = test -48;
       delay (2);
     //delay (5);
     //ramka[i] = (Serial.parseInt()-48);
     //if (Serial.read() == (32 -48)) break;  // if space read() 
       
      }
   
      //proccess relays data
    rxdata.destnode = (ramka[0] * 10) + ramka[1];
    rxdata.cmd = (ramka[2] * 10) + ramka[3];
    rxdata.state = ramka[4];
#endif

// data to send use node ID ex. 02 and 3 times zeros from the begining and space, after that type value to transmit to node (currently used for pulse)max value  4294967295 
 
#ifdef PULSE_MEASURE    
char incomingByte;  // for data after getting data from serial
unsigned long values = 0;  
rxdata.receivedvalue = 0;

     while(1) {
        
         //incomingByte = Serial.read();
          incomingByte = Serial.read();
          
          if (incomingByte == '\n'|| incomingByte == 0 || incomingByte == '\t' || incomingByte == -1) break;   // exit the while(1), we're done receiving
          if (incomingByte == 32) continue;  // if space read() 
          //if (incomingByte == -1) break; // if no characters are in the buffer read() returns -1
         delay (2);
          values *= 10;  // shift left 1 decimal place
         // convert ASCII to integer, add, and shift left 1 decimal place
         values = ((incomingByte -48) + values);
         delay(2);
         //values = ((incomingByte) + values);
    }
   
         //Serial.flush(); //clear serial data 
         rxdata.receivedvalue = values;
         //measure.pulse=rxdata.receivedvalue;
delay (10);
#endif

//#ifdef DEBUG 
 
 
    #ifdef RELAY_AMOUNT 
       
       if (rxdata.receivedvalue == 0) {
       
       Serial.println();
       Serial.print("SEND NODE ");
       Serial.print(rxdata.destnode);
       Serial.print(" RELAY ");
       Serial.print(rxdata.cmd);
       Serial.print(" STATE ");
       Serial.print(rxdata.state);
       Serial.print(' ');
       }
   #endif  
   #ifdef PULSE_MEASURE   
       if (values != 0) { 
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
 
//#endif


//doMeasure(); //coment it for basenode
//coderelayStatusConfirmation(); //coment it for basenode
    }   
 
}

