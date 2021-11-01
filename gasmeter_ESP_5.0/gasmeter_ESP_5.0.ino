/***************************************************
  based on: Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

 MQTT Gas- en thermometer Makerspace Leiden 
 Aart 03-2017
 Lucas 10-2017
 
 Sensor: Reflectiesensor met 100 k pullup on pin A0  & DS18S20 probe aan pin D4
 Draait nu op een NodemCU Amice (Is dat een V1?)

 Set offset van de gasmeter: 
  mosquitto_pub -h makerspaceleiden.nl -t "makerspace/gasmeter/set" -m "22170.23"
  Please note; load will be > 100% until gasmeter is set. 
  (this does not seem to work, gives exception? Investigate or replace with serial input (Since one needs to physically be present to read it anyway...)

NOTES / uitbreidingen Lucas:
-> Stand elke .. minuten wegschrijven naar EEPROM, tenzij onveranderd. Bij opstarten controleren of er een hogere meterstand in eeprom staat dan in ram. 
(Wear leveling, standen blijven schrijven op steeds andere eeprom locatie, bij opstarten hoogste inlezen en volgende keer op de locatie erna schrijven)
(up to 100.000 write cycles for the (M25Q32) flash, so every 15 minutes would wear it out completely in less then 3 years... Spread over 10 locations, that's 28 years, good enough)
-> Status van meter ook over MQTT melden (Of 'ie is gaan afwijken doordat er een stand uit EEPROM is gehaald of dat 'ie zelf nog denkt te kloppen)  
  
 ****************************************************/
 
#define LOAD_PIN 16              // 2 = Internal red LED, beware inverted. CPU load. High = no load  = LED off

#define SENSOR_PIN A0           // Gas meter sensor pin (analog)
#define DS_PIN 2                // "D4", DS18S20 pin en blauwe led

#define DETECT 890              // Analog value "definately a mirror" (Changed! 500 is too low)
#define RELEASE 920             // Analog value "definately no mirror" 
#define SEND_INTERVAL 5000      // minimal milliseconds between message

#define WRITE_INTERVAL_MILLIS 1800000 // 15 minutes = 900.000ms ; 30 minutes = 1.800.000 millis
#define NUM_EEPROM 20         // number of EEPROM addresses used. (It will use them evenly in a round-robin fasion for wear leveling)
#define INITIAL_GASMETER 211382.00

#define debug_gas           // gasmeter routine debug messages
// #define debug_temp          // DS18S20 routine debug messages

  
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <EEPROM.h>


#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include "passwords.h"
/************************* WiFi Access Point ********************************

#ifndef #WLAN_SSID
#define WLAN_SSID       "IoT_accespoint"
#define WLAN_PASS       "Password"
#endif

/************************* MQTT Setup ****************************************/

#define AIO_SERVER      "makerspaceleiden.nl"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    ""
#define AIO_KEY         ""

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'temperatureFeed' for publishing.
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "makerspace/temp/CV");

// Setup a feed called 'gasMeterFeed' for publishing.
Adafruit_MQTT_Publish gasMeterFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "makerspace/gasmeter");

// Setup a feed called 'gasMeterStatus' for publishing.
Adafruit_MQTT_Publish gasMeterStatus = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "makerspace/gasmeter/status");

// Setup a feed called 'setMeterFeed' for subscribing to changes.
//Adafruit_MQTT_Subscribe setMeterFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "makerspace/gasmeter/set");

/*************************** Sketch Code ************************************/

// DS18S20 startup
OneWire  ds(DS_PIN);  

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

typedef union{
    float meterstand;
    byte bytes[sizeof(float)];
   }splitsMeterstandInBytes;

unsigned int globalEepromAddres;

void setup() {

  Serial.begin(115200);
  delay(500); // workaround so I can see bootup messages...

  Serial.println(F("Adafruit MQTT demo, aangepast voor de gasmeter van MSL"));

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());

  // Setup MQTT subscription for onoff feed.
//  mqtt.subscribe(&setMeterFeed);

  pinMode(LOAD_PIN, OUTPUT);
  digitalWrite (LOAD_PIN,LOW); 

  // EEPROM init. ESP8266 uses part of it's FLASH instead of EEPROM, API is different, 
  // see https://esp8266.github.io/Arduino/versions/2.0.0/doc/libraries.html#eeprom. 
  // Mostly, it will not be writen untill a EEPROM.commit() or EEPROM.end() call.

  EEPROM.begin( NUM_EEPROM*sizeof(float)  ); // enough EEPROM space for 20 "gasmeter" writes.


Serial.println("checking EEPROM for initial meterstand");
 // check EEPROM for old gasmeterstanden.
 for(int i = 0;i<(NUM_EEPROM*sizeof(float));i+=sizeof(float)){
  splitsMeterstandInBytes uitFlash;
  float grootsteSoFar = 0.02; // because starting at 0 has "(0 > 0)=true" issues...
  bool unused = true;

  ReadEeprom(i,&uitFlash,&unused);
  if(!unused){ // only if there actually is a valid meterstand in EEPROM    
    if ( (uitFlash.meterstand > grootsteSoFar) & (uitFlash.meterstand < 999999.99 ) ){ // otherwise, pick the biggest one as the most recent but within reason
      grootsteSoFar = uitFlash.meterstand;
      globalEepromAddres=i;                  // and store it's addres
      Serial.println("meterstand gevonden: ");
      Serial.print(grootsteSoFar);
      Serial.print("\n"); 
    }
    // if there is multiple with the same value this won't work. Writing is suposed to not write when value is not changed.
  }
  
  }
  
}

void loop() {
  static float gasMeter=INITIAL_GASMETER, prevGasMeter;    // current and previous value of gas meter
  static float temperature; // Make a wild guess :-)
  static long last_send;    // Time of last message in ms
  static long start_time;   // Millis() Start time of next 100 mS system tick
  static long last_Write;   // EEPROM Write Interval start time (millis)
  enum Staat {OK, MIGHT_BE_OFF, NOT_SET};
  static Staat staat=NOT_SET; // because status is allready a keyword
   
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  // try to spend your time here

  // setting of gasmeter
    Adafruit_MQTT_Subscribe *subscription;

 
  //read from EEPROM location with the most recent / biggest gasmeterstand, and when that's bigger then what's in RAM, copy it and change staat to "might be off" because apearently gasmeterreader missed something
  // (Because of power down?)
  bool unused, SerIn_flag=false;
  splitsMeterstandInBytes uitFlash;
  ReadEeprom(globalEepromAddres,&uitFlash,&unused);
  if(!unused && uitFlash.meterstand > gasMeter){ //only if actually read from a in use EEPROM location
    gasMeter = uitFlash.meterstand;
    prevGasMeter = gasMeter; // so it does not get written unless it changes first.
    staat=MIGHT_BE_OFF;
    Serial.println(F("Gasmeterstand re-read from EEPROM"));
    }

  /*
   *  if serial input works, then this can be removed, also the setup for it.
   * 
  //if (gasMeter < 1) {
  if(staat == NOT_SET || MIGHT_BE_OFF){ // somehow an exception gets thrown when anything is posted to makerspace/gasmeter/set
    Serial.println(F("Gasmeter not set"));
    while ((subscription = mqtt.readSubscription(1000))) {
      if (subscription == &setMeterFeed) {      
        // Set new gasMeterValue
          Serial.print(F("Set gasMeterValue: ")); // but when that exception gets thrown this text is not shown on serial debug
          gasMeter = atof( (char *) setMeterFeed.lastread); 
          Serial.println(gasMeter);
          staat = OK;
        }
      }
    }
  
  
  */


  // serial input to set gasmeter value.
  if( Serial.available() >= 14 ){ // "set 123456,78" is 13 + termination is 14. 
    if(Serial.find("set")){
      gasMeter = Serial.parseFloat(); //  seems a very arduino-y way of doing things
      Serial.print(F("Set gasMeterValue: "));
      Serial.println(gasMeter);
      staat = OK;
      SerIn_flag=true; // so the new setting gets written to EEPROM immedeately
    }
  }
  
  gasMeter = gas_Meter_Reader(gasMeter);
  temperature = DS_reader (); 

  if ((millis() - last_send) > SEND_INTERVAL) { 
    last_send = millis(); 
    // Now we can publish stuff!
    
    Serial.print(F("\nSending temperature "));
    Serial.print(temperature);
    Serial.print("...");
    if (! temperatureFeed.publish(temperature)) {
    Serial.println(F("FaiLED_PIN"));
    } else {
      Serial.println(F("OK!"));
      delay(50);
    }
    
    Serial.print(F("\nSending gasMeter "));
    Serial.print(gasMeter);
    Serial.print("...");
    if (! gasMeterFeed.publish(gasMeter)) {
    Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
    delay(50);
  

    if(staat!=OK){
      Serial.print(F("\nSending gasMeterStatus "));
      Serial.print(staat);
      Serial.print("...");
      if (! gasMeterStatus.publish("Reading might be off. Dear human, please verify and correct")){ 
        // would need powercycle or... we could change the software to accept changes even when ther is a value allready...
      Serial.println(F("Failed"));
      } else {
        Serial.println(F("OK!"));
      }
      delay(50);
  }
  }
  // Wait up to 100 ms in this idle routine
  
  digitalWrite (LOAD_PIN,HIGH); 
  
  while (millis() - start_time  < 100) {
    delay(1); // Do nosePicking();  
  }
  digitalWrite (LOAD_PIN,LOW); 
  start_time = millis(); 

  // save gasmeter value to EEPROM every xx minutes (But only if it has changed), preferably in a way with wear leveling...
  if ((millis()- last_Write > WRITE_INTERVAL_MILLIS) | SerIn_flag) {
    if(gasMeter!=prevGasMeter){ // only write if value is changed (Even if changed down. Because maybe that 'll come in usefull someday)
    // Arduino EEPROM libs do that automatically, but my "wear leveling" (by incrementing addres each time) ruins that, also I don't know if ESP EEPROM libs do that at all anyway...   
    
    // first increment the addres (because on startup it is read from location last written and now it should write the next location)
      if(globalEepromAddres<(NUM_EEPROM*sizeof(float))) {
        globalEepromAddres+=sizeof(float);
        }else globalEepromAddres = 0;
    // then write
    splitsMeterstandInBytes schrijfdit;
    schrijfdit.meterstand = gasMeter;
    WriteEeprom(globalEepromAddres,&schrijfdit);  
    EEPROM.commit(); // actually write.
    last_Write=millis(); // don' t forget this!
    prevGasMeter = gasMeter; 
    Serial.println(F("Written gasmeterstand to EEPROM location "));
    Serial.print(globalEepromAddres);
    Serial.print("\n");
    SerIn_flag = false;
    }
  }
  
}  

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;

        // TODO: staat = MIGHT_BE_OFF, because it' s not in its pulse detecting loop when it has lost WiFi. But Staat is not globlal.
       
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

float gas_Meter_Reader(float gasMeter) { 
  /*
   * Reading the input and adding 0.1 m3 per detected pulse. 
   */
  static long last_run;    // Time of last run of this routine in ms
  int sensor;   // sensor value
  enum states { // States of the input state machine 
    wait_for_start,
    doublecheck,
    count, 
    wait_for_end, 
  };
  static int state = wait_for_start;  
  
  if (millis() - last_run < 500) return gasMeter; 
  last_run = millis(); // Run once every 500 ms
  
  sensor = analogRead(SENSOR_PIN); 
  
  #ifdef debug_gas 
  Serial.print ("Sensor: "); 
  Serial.println (sensor); 
  #endif  
  
  switch (state) { 
    case wait_for_start: { 
      if (sensor < DETECT) state = doublecheck; 
      #ifdef debug_gas
        Serial.println("State: Wait for start"); 
      #endif
    } break; 
    
    case doublecheck: { 
      if (sensor < DETECT) state = count; 
      if (sensor > DETECT) state = wait_for_start;  
      #ifdef debug_gas
        Serial.println("State: Doublecheck"); 
      #endif
    } break; 
    
    case count: { 
      gasMeter += 0.1;  // 0,1 m3 per omwenteling (last digit has a mirror, but second to last digits gets incremented when mirror is seen)
      #ifdef debug_gas
        Serial.print("State: count; gasMeterValue: "); 
        Serial.println(gasMeter); 
      #endif
      state = wait_for_end; 
    } break; 
    
    case wait_for_end: { 
      if (sensor > RELEASE) state = wait_for_start; 
      #ifdef debug_gas
        Serial.println("State: Wait for end"); 
      #endif
    } break; 
    break; 
  }
  return gasMeter; 
}

float DS_reader () {
  /*
   * Reads one DS chip in two phases without using delay(). 
   */
  static long last_action;    // Time of last action in ms
  byte data[12];
  static byte addr[8];
  byte type_s, present = 0;
  static float celsius;
  enum states { // States of the DS read state machine 
    ds_start, 
    ds_read,
  };
  static int state = ds_start;  
  
  switch (state) { 
    case ds_start: {
      ds.reset_search();
      ds.search(addr); 
      if (OneWire::crc8(addr, 7) != addr[7]) {
        #ifdef debug_temp  
          Serial.println("CRC is not valid!");
        #endif
      }
      switch (addr[0]) {
        case 0x10:
          #ifdef debug_temp  
            Serial.println("  Chip = DS18S20");  // or old DS1820
          #endif
            type_s = 1;
        break;
        case 0x28:
          #ifdef debug_temp  
            Serial.println("  Chip = DS18B20");
          #endif
          type_s = 0;
        break;
        case 0x22:
          #ifdef debug_temp  
            Serial.println("  Chip = DS1822");
          #endif
          type_s = 0;
        break;
        default: {
        #ifdef debug_temp  
           Serial.println("Device is not a DS18x20 family device.");
         #endif 
        }
      }
      ds.reset();
      ds.select(addr);
      ds.write(0x44, 1); 
      state = ds_read; 
      last_action = millis(); 
    } break; 
    
    case ds_read: {
      if ((millis() - last_action) < 800) break; // doe pas na 1 seconde
      
      present = ds.reset();
      ds.select(addr);    
      ds.write(0xBE);         // Read Scratchpad
      #ifdef debug_temp  
        Serial.print("  Data = ");
        Serial.print(present, HEX);
        Serial.print(" ");
      #endif
          for (int i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
        #ifdef debug_temp  
          Serial.print(data[i], HEX);
          Serial.print(" "); 
        #endif
      }
      #ifdef debug_temp
      Serial.println(" ");
      Serial.print(" CRC=");
      Serial.print(OneWire::crc8(data, 8), HEX);
      Serial.println(" ");
      #endif
      int16_t raw = (data[1] << 8) | data[0];
      if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
          // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
      }
      celsius = (float)raw / 16.0;
      #ifdef debug_temp
        Serial.print("  Temperature = ");
        Serial.print(celsius);
        Serial.println(" Celsius, ");
      #endif
      state = ds_start; 
    }
  }
  return celsius;  
}

void ReadEeprom(int addres, splitsMeterstandInBytes* waarde, bool* unused){
  *unused = true;
  for(int j = 0;j<sizeof(float);j++){
    waarde->bytes[j] = EEPROM.read(addres+j);
   
   /* // was usefull for debug
    Serial.print("EEadr:: ");
    Serial.print(addres+j);
    Serial.print("\n");
   */
   
   /* 
    Serial.print("EEval: ");
    Serial.print(waarde->bytes[j]);
    Serial.print("\n");
   */ 
    if(waarde->bytes[j] != 0xFF) *unused = false; // when all bytes are 0xFF this location is empty 
  }
};

void WriteEeprom(int addres, splitsMeterstandInBytes* waarde){
  for(int j = 0;j<sizeof(float);j++){
    EEPROM.write(addres+j,waarde->bytes[j]);
  }
}
