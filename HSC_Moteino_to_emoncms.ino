/*

This sketch is designed to pull temperature and pressure data from a Honeywell HSC or SSC-series pressure sensor:  
https://sensing.honeywell.com/trustability-silicon-pressure-sensors-hsc-and-ssc-series

...Over I2C interface using a Moteino R4 with RFM12B radio:
https://lowpowerlab.com/shop/index.php?_route_=moteino-r4

...And forward pressure, temperature, and battery voltage to emoncms via emonHub running on an emonBase:
http://shop.openenergymonitor.com/emonbase-raspberry-pi-web-connected-base-station/

The code is based on the emonTH V1.4 example code from Glyn Hudson, modified by "GJP" so more generic for use with other arduino 328 platforms (i.e. Moteino), 
and the Honeywell HSC library and example code from Petre Rodan.

Mashup of the above code and some months of field testing by Jeff Dykhouse jeffsjunkcatcher at gmaildotcom
Licence: GNU GPL V3

Note:  Some humidity and temperature code still exists, even though we're only reading the Honeywell sensor and battery voltage in this sketch.  Sorry for the mess.


Original comments below (edited):
*********************************

  emonTH V1.4 Low Power DHT Humidity & Temperature & DS18B20 Temperature Node Example - modified 21.12.2013 GJP so more generic for use with other arduino 328 platforms 
 
  Technical hardware documentation wiki: http://wiki.openenergymonitor.org/index.php?title=EmonTH
 
  Part of the openenergymonitor.org project
  Licence: GNU GPL V3
 
  Authors: Glyn Hudson
  Builds upon JCW JeeLabs RF12 library, Arduino and Martin Harizanov's work
  
  
  THIS SKETCH REQUIRES:

  Libraries in the standard arduino libraries folder:
	- JeeLib		https://github.com/jcw/jeelib   //library to work with 328s
	- DHT Sensor Library    https://github.com/adafruit/DHT-sensor-library - be sure to rename the sketch folder to remove the '-'
        - OneWire library	http://www.pjrc.com/teensy/td_libs_OneWire.html
	- DallasTemperature	http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip
        - Honeywell HSC library https://github.com/rodan/honeywell_hsc_ssc_i2c

  Recommended node ID allocation
  ------------------------------------------------------------------------------------------------------------
  -ID-	-Node Type- 
  0	- Special allocation in JeeLib RFM12 driver - reserved for OOK use
  1-4     - Control nodes 
  5-10	- Energy monitoring nodes
  11-14	--Un-assigned --
  15-16	- Base Station & logging nodes
  17-30	- Environmental sensing nodes (temperature humidity etc.)
  31	- Special allocation in JeeLib RFM12 driver - Node31 can communicate with nodes on any network group
  -------------------------------------------------------------------------------------------------------------
*/

#define FREQ RF12_433MHZ  // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int nodeID = 25;  // Pressure RFM12B node ID - should be unique on network
const int networkGroup = 210;  // Pressure wireless network group - needs to be same as emonBase and emonGLCD
const int time_between_readings= 25;  // in seconds
const int TEMPERATURE_PRECISION=11;  // DS18B20 resolution no of bits 9 (93.8ms),10 (187.5ms) ,11 (375ms) or 12 (750ms) bits equal to 0.5C, 0.25C, 0.125C and 0.0625C
#define ASYNC_DELAY 375  // Delay for DS18B20 readings y for 9 bit requres 95ms, 10bit 187ms, 11bit 375ms and 12bit resolution takes 750ms

// See block comment above for library info
#include <avr/power.h>
#include <avr/sleep.h>
#include <JeeLib.h>                                                 
//#include <OneWire.h>
//#include <DallasTemperature.h>
//#include "DHT.h"
#include <Wire.h>
#include "hsc_ssc_i2c.h"

ISR(WDT_vect) { Sleepy::watchdogEvent(); }                            // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

// pin allocations 
const int HSC_PWR=4; //power Honeywell HSC sensor from D4 to save energy

const int LED=9; // D9 emonTx on D6 / emonTH on D9
const int BATT_ADC=1; //A1/D15 on emonTH

// ########### Honeywell HSC setup ###########################################
// see hsc_ssc_i2c.h for a description of these values
// these defaults are valid for the HSCSANN150PG2A3 chip "Pressure Sensors SIP Axial 150 PSI Gage 3.3V i2C interface"
#define SLAVE_ADDR 0x28         // Sensors are available with several addresses.  Enter yours here.
#define OUTPUT_MIN 0
#define OUTPUT_MAX 0x3fff       // 2^14 - 1
#define PRESSURE_MIN 0.0        // min is 0 for sensors that give absolute values
#define PRESSURE_MAX 150        // enter your sensor's max pressure range here, in PSI
uint32_t prev = 0; 
const uint32_t interval = 5000;
// ########### End Honeywell HSC setup ###########################################

typedef struct {                                                      // RFM12B RF payload datastructure
  	  int psi1;
          int temp_external;
          int humidity;    
          int battery;          	                                      
} Payload;
Payload emonth;


boolean debug;
int numSensors; 
//addresses of sensors, MAX 4!!  
byte allAddress [4][8];  // 8 bytes per address

//################################################################################################################################
//################################################################################################################################
void setup() {
//################################################################################################################################
  
  pinMode(LED,OUTPUT); digitalWrite(LED,HIGH);  // Status LED on
   
  rf12_initialize(nodeID, FREQ, networkGroup);  // Initialize RFM12B
  emonth.psi1 = 15;
  // Send RFM12B test sequence (for factory testing)
//  for (int i=0; i<10; i++)                                           
//  {
//    emonth.psi1=i; 
//    rf12_sendNow(0, &emonth, sizeof emonth);
//    delay(100);
//  }
  rf12_sendWait(2);
  emonth.psi1=1500;
  // end of factory test sequence
  
  rf12_sleep(RF12_SLEEP);
  
  if (Serial) debug = 1; else debug=0;  //if serial UART to USB is connected show debug O/P. If not then disable serial
  
  if (debug==1)
  {
    Serial.begin(9600);
    Serial.println("");
    Serial.println("Pressure Monitor"); 
    Serial.println("PeakMonitor.com");
    Serial.print("Node: "); 
    Serial.print(nodeID); 
    Serial.print(" Freq: "); 
    if (FREQ == RF12_433MHZ) Serial.print("433Mhz");
    if (FREQ == RF12_868MHZ) Serial.print("868Mhz");
    if (FREQ == RF12_915MHZ) Serial.print("915Mhz"); 
    Serial.print(" Network: "); 
    Serial.println(networkGroup);
    delay(100);
  }
  
  pinMode(HSC_PWR,OUTPUT);
  pinMode(BATT_ADC, INPUT);


  //################################################################################################################################
  // Power Save  - turn off what we don't need - http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
  //################################################################################################################################
  //ADCSRA =0;                           //needed for ADC to be re-enabled during loop()
  ACSR |= (1 << ACD);                     // disable Analog comparator    
  power_adc_disable();
  if (debug==0) power_usart0_disable();   //disable serial UART jd
//  power_twi_disable();                    //Disable the Two Wire Interface module.
  // power_timer0_disable();              //don't disable necessary for the DS18B20 library  // not sure why I can't disable this?
  power_timer1_disable();
  power_spi_disable();
 

  if (debug==1) delay(200);
  digitalWrite(LED,LOW);
  
  //##################################################################
  //  HSC Pressure Sensor
  //##################################################################
  Wire.begin();
  
  
} // end of setup


//################################################################################################################################
//################################################################################################################################
void loop()
//################################################################################################################################
{ 
  
  power_adc_enable();
  emonth.battery=int(analogRead(BATT_ADC)*0.3125806*2);                    //read battery voltage, convert ADC to volts x100
                                                                           //max voltage 3.3V so you might need a voltage divider to reduce voltage before ADC if batteries supply more than 3.3V.
                                                                           
  power_adc_disable();                                               
  
     if (debug==1) 
    {
      Serial.print("Battery voltage: ");  
      Serial.print(emonth.battery/100.0);  //
      Serial.println("V");
      delay(100);
    }

// ################# Honeywell HSC READ  ##################################

    digitalWrite(HSC_PWR,HIGH);                               // Power on sensor
    dodelay(1000);                                             //sleep for 1s to allow sensor to warm up
    unsigned long now = millis();
    struct cs_raw ps;
    char p_str[10], t_str[10];
    uint8_t el;
    float p, t;
    


    if ((now - prev > interval)) { // && (Serial.available() <= 0)) {
        prev = now;
        el = ps_get_raw(SLAVE_ADDR, &ps);
        
        // Poll sensor again so we can ignore the bad first read data that happens most times
        dodelay(500);                                          //sleep for 1s to allow sensor to forget bad read
        el = ps_get_raw(SLAVE_ADDR, &ps);
        // for some reason my chip triggers a diagnostic fault
        // on 50% of powerups without a notable impact 
        // to the output values.
        if ( el == 4 ) {
            Serial.println("err sensor missing");
              digitalWrite(LED,HIGH); // FLash LED 5s to show sensor missing
              dodelay(5000);
              digitalWrite(LED,LOW);  
        } else {
            if ( el == 3 ) {
                Serial.print("err diagnostic fault ");
                Serial.println(ps.status, BIN);
                digitalWrite(LED,HIGH); // FLash LED 3s to show diagnostic fault
                dodelay(3000);
                digitalWrite(LED,LOW);     
            }
            if ( el == 2 ) {
                // if data has already been feched since the last
                // measurement cycle
                Serial.print("warn stale data ");
                Serial.println(ps.status, BIN);
                digitalWrite(LED,HIGH); // FLash LED 1s to show stale data
              dodelay(1000);
              digitalWrite(LED,LOW);  
            }
            if ( el == 1 ) {
                // chip in command mode
                // no clue how to end up here
                Serial.print("warn command mode ");
                Serial.println(ps.status, BIN);
            }
              if (debug==1) 
            {
            Serial.print("status      ");
            Serial.println(ps.status, BIN);
            Serial.print("bridge_data ");
            Serial.println(ps.bridge_data, DEC);
            Serial.print("temp_data   ");
            Serial.println(ps.temperature_data, DEC);
            Serial.println("");
            }
            ps_convert(ps, &p, &t, OUTPUT_MIN, OUTPUT_MAX, PRESSURE_MIN,
                   PRESSURE_MAX);
            // floats cannot be easily printed out
            dtostrf(p, 2, 2, p_str);
            dtostrf(t, 2, 2, t_str);
            
            if (debug==1) 
            {
            Serial.print("pressure    (PSI) ");
            Serial.println(p_str);
            Serial.print("temperature (dC) ");
            Serial.println(t_str);
            Serial.println("");
            }
            digitalWrite(HSC_PWR,LOW);                               // Power off sensor            
            
            // ### load emoncms payload ###
            emonth.psi1 = atof(p_str)*100;
            emonth.temp_external = atof(t_str)*100; //*1.8)-32);    // convert temperature to Farenheight

            
       }
      
    }
    

 //####################### END Honeywell HSC Read  ########################################   
  
  power_spi_enable();  
  rf12_sleep(RF12_WAKEUP);
  rf12_sendNow(0, &emonth, sizeof emonth);   // JD changed from 0 to 2
  // set the sync mode to 2 if the fuses are still the Arduino default
  // mode 3 (full powerdown) can only be used with 258 CK startup fuses
  rf12_sendWait(2);
  rf12_sleep(RF12_SLEEP);
  power_spi_disable();  
  digitalWrite(LED,HIGH); // Flash LED to show successful loop
  dodelay(100);
  digitalWrite(LED,LOW);  
  
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;   
  Sleepy::loseSomeTime(time_between_readings*1000);  
  // Sleepy::loseSomeTime(2000);
  ADCSRA=oldADCSRA; // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}

void dodelay(unsigned int ms)
{
  byte oldADCSRA=ADCSRA;
  byte oldADCSRB=ADCSRB;
  byte oldADMUX=ADMUX;
      
  Sleepy::loseSomeTime(ms); // JeeLabs power save function: enter low power mode for x seconds (valid range 16-65000 ms)
      
  ADCSRA=oldADCSRA;         // restore ADC state
  ADCSRB=oldADCSRB;
  ADMUX=oldADMUX;
}
