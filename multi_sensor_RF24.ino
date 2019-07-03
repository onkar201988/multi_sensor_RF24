#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LowPower.h>
#include <Vcc.h>
#include <avr/wdt.h>
#include <DHT.h>

#define debug                                 // comment this to remove serial prints
//----------------------------------------------------------------------------------------------------
const uint8_t   CE_pin    = 8;                // This pin is used to set the nRF24 to standby (0) or active mode (1)
const uint8_t   CSN_pin   = 9;                // This pin is used for SPI comm chip select
#define         LDR_pin     A0                // LDR pin, connected to A0 analog pin
const uint8_t   DHT_pin   = 7;                // DHT 11 sensor pin
const uint8_t   PIR_pin   = 2;                // PIR pin
const uint8_t   SENSOR_VCC_pin = 4;           // VCC for DHT and LDR is provided via pin 4

const uint64_t  pipeAddress = 0xB00B1E50C3LL; // Create pipe address for the network, "LL" is for LongLong type
const uint8_t   rfChannel   = 89;             // Set channel frequency default (chan 84 is 2.484GHz to 2.489GHz)
const uint8_t   retryDelay  = 7;              // this is based on 250us increments, 0 is 250us so 7 is 2 ms
const uint8_t   numRetries  = 5;              // number of retries that will be attempted

const float     VccMin      = 1.8;            // Minimum expected Vcc level, in Volts. (0%)
const float     VccMax      = 3.2;            // Maximum expected Vcc level, in Volts. (100%)
const float     VccCorrection = 1.0/1.0;      // Measured Vcc by multimeter divided by reported Vcc

const int       sleepDuration = 8;            // Sleep duration to report Temp data (8 Sec x number of times)(10800 for a day)
volatile int    sleepCounter  = 0;            // Counter to keep sleep count

volatile int    motionDetectTimer = 0;        // motion detection timer, which will keep motion active for certain time
const int       motionTimerDuration = 3;      // constant motion timer duration (8 Sec x numebr of times) (8 for a min approx.)
volatile bool   motionDetected = false;       // motion detection flag
//----------------------------------------------------------------------------------------------------
const int       SENSORTYPE   = 0;             // Sensor type [D:Door, T:Temerature, etc]
const int       SENSORNUMBER = 1;             // Sensor number[1: Living Room, 2:kitchen, 3:Master bedroom, etc]
const int       MOTION       = 2;             // Motion sensor status, P:Presenmt, A:Absent]
const int       BATTERY      = 3;             // Battery status, 0-100%
const int       TEPMERATURE  = 4;             // Temperature (in F: 0 - 255)
const int       HUMIDITY     = 5;             // Humidity (0 - 100%)
const int       LIGHT        = 6;             // Light intensity (0 - 100%)

const int       PAYLOAD_LENGTH = 7;

volatile char   send_payload[PAYLOAD_LENGTH];

#define DHTTYPE DHT11                         // Type of DHT sensor

Vcc vcc(VccCorrection);                       // create VCC object
RF24 rfRadio(CE_pin, CSN_pin);                // Declare object from nRF24 library (Create your wireless SPI) 
DHT dht(DHT_pin, DHTTYPE);                    // Declaire object for DHT
//----------------------------------------------------------------------------------------------------

void setup() {
  
  send_payload[SENSORTYPE]   = 'T';           // Sensor type [D:Door, T:Temerature, etc]
  send_payload[SENSORNUMBER] = '1';           // Sensor number[1: Living Room, 2:kitchen, 3:Master bedroom, etc]
  send_payload[MOTION]       = 'A';           // Motion sensor status, P:Presenmt, A:Absent]
  send_payload[BATTERY]      = (char) 100;    // Battery status, 0-100%
  send_payload[TEPMERATURE]  = (char) 0;      // Temperature (in F: 0 - 255)
  send_payload[HUMIDITY]     = (char) 0;      // Humidity (0 - 100%)
  send_payload[LIGHT]        = (char) 0;      // Light intensity (0 - 100%)

  pinMode(PIR_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR_pin), motionDetection_ISR, RISING);
  
  pinMode(SENSOR_VCC_pin, OUTPUT);
  
  #ifdef debug
    Serial.begin(115200);                     //serial port to display received data
    Serial.println("Multi sensor module is online...");
  #endif

  dht.begin();
  readSensorData();
  sendData();                                 // and send to server
}

//----------------------------------------------------------------------------------------------------
void turnOnVcc()
{
  digitalWrite(SENSOR_VCC_pin, true);
  delay(1000);
}
//----------------------------------------------------------------------------------------------------
void turnOffVcc()
{
  digitalWrite(SENSOR_VCC_pin, false);
}
//----------------------------------------------------------------------------------------------------
void readSensorData()
{
  turnOnVcc();
  readBattery();
  readTempHumidity();
  readLightIntensity();
  turnOffVcc();
}

//----------------------------------------------------------------------------------------------------
void motionDetection_ISR()
{
  #ifdef debug
    Serial.println("Motion detected");
    delay(200);
    yield();
  #endif
  
  if (! motionDetected )
  {
    motionDetected = true;
    send_payload[MOTION] = 'P';               // Change the motion to present
    sendData();
  }

  motionDetectTimer = 0;                      // Reset the motion detection timer to 0
}

//----------------------------------------------------------------------------------------------------
void powerUpNRF()
{
  rfRadio.begin();                            //Start the nRF24 module
  rfRadio.setPALevel(RF24_PA_LOW);            //Set low power for RF24
  rfRadio.setChannel(rfChannel);              //set communication frequency channel
  rfRadio.setRetries(retryDelay,numRetries);  //if a transmit fails to reach receiver (no ack packet) then this sets retry attempts and delay between retries   
  rfRadio.openWritingPipe(pipeAddress);       //open writing or transmit pipe
  rfRadio.stopListening();                    //go into transmit mode
  delay(50);
}

//----------------------------------------------------------------------------------------------------
void powerDownNRF()
{
  rfRadio.powerDown();
  delay(50);
}

//----------------------------------------------------------------------------------------------------
void sendData()
{
  powerUpNRF();
  
  if (!rfRadio.write(send_payload, PAYLOAD_LENGTH))
  #ifdef debug
    {  //send data and remember it will retry if it fails
      Serial.println("Sending failed, check network");
    }
    else
    {
      Serial.println("Sending successful, data sent");
    }
  #else
    {
    }
  #endif

  powerDownNRF();
}

//----------------------------------------------------------------------------------------------------
void readLightIntensity()
{
  int lightValue = analogRead(LDR_pin);
  send_payload[LIGHT] = (char)(map(lightValue, 0, 1023, 0, 100));
  #ifdef debug
    Serial.print("Light intensity:");
    Serial.println(map(lightValue, 0, 1023, 0, 100));
    delay(200);
    yield();
  #endif
}

//----------------------------------------------------------------------------------------------------
void readBattery()
{
  send_payload[BATTERY] = (char) vcc.Read_Perc(VccMin, VccMax);
  #ifdef debug
    Serial.print("Battery percentage:");
    Serial.println(vcc.Read_Perc(VccMin, VccMax));
    delay(200);
    yield();
  #endif
}

//----------------------------------------------------------------------------------------------------
void readTempHumidity()
{
  float newTempValue = dht.readTemperature(true); //to use celsius remove the true text inside the parentheses  
  float newHumValue = dht.readHumidity();

  #ifdef debug
    Serial.print("Temperature: ");
    Serial.print(newTempValue);
    Serial.println(" *F");

    Serial.print("Humidity: ");
    Serial.print(newHumValue);
    Serial.println("%");
  #endif

  send_payload[TEPMERATURE] = (char)((int) newTempValue); // Convert float to int and then char
  send_payload[HUMIDITY]    = (char)((int) newHumValue);  // Because we can only send a 8 bit data
}

//----------------------------------------------------------------------------------------------------
ISR(WDT_vect)
{
  //wdt_disable();
}

//----------------------------------------------------------------------------------------------------

void loop() {

  if(motionDetected)
  {
    if(!digitalRead(PIR_pin))                 // Check if PIR pin is low, then only increment counter
    {                                         // Beccause if motion is continuously active, no interrupt will be fired
      motionDetectTimer++;
    }
    
    if(motionDetectTimer >= motionTimerDuration)
    {
      #ifdef debug
        Serial.println("Motion timer expired, no motion present");
        delay(200);
        yield();
      #endif

      motionDetected = false;                 // motion not detected, 
      send_payload[MOTION] = 'A';             // Change the motion to Absent
      sendData();
    }
  }

  sleepCounter++;
  if(sleepCounter > sleepDuration)
  {
    #ifdef debug
      Serial.println("Sending data...");
      delay(200);
      yield();
    #endif
    delay(200);                               // This delay is to stabilize VCC voltage
    readSensorData();
    sendData();
    sleepCounter = 0;
  }
  else
  {
    #ifdef debug
      Serial.println("Going to sleep");
      delay(200);
      yield();
    #endif
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}
