/*
LAKAS-1
a Tripoli Level 1 (L1) High-Power Rocket (HPR)

Code description:
Ground Station for onboard avionics on LAKAS-1
Uses the following devices:
- Telemetry:
  - nRF24L01 + PA + LNA (power amplifier + low noise amplifier)
    - Operates at 250 KBPS
- Logging
  - Plan to copy and paste serial monitor output
  - Will probably change in coming iterations

Rationale:
Get basic flight data from rocket during flight and send in real-time to ground station (aka my laptop)

Wiring:
nRF24L01  Nano
VCC       5V (note: using adapter)
GND       GND
CE        9
CSN       10
SCK       13
MOSI      11
MISO      12

*/

#include <SPI.h> //SPI

// nRF24L01 libs
#include <RF24.h>
#include <RF24_config.h>
#include <nRF24L01.h>
#include <printf.h>

#define CE_PIN  49
#define CSN_PIN 53

// sensor definitions
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio



const byte slaveAddress[5] = {'U','r','m','o','m'};


void setup(){
  Serial.begin(115200);

  radio.begin();    // radio initialize
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setRetries(10,10);
  radio.openReadingPipe(1, slaveAddress);
  radio.startListening();

}
void loop(){
  //NOTE: THIS WILL PRINT NOTHING IF NO SIGNAL!
  int datasize = 8; // match with number of data points to collect
  float data[datasize]={};
  if(radio.available()){
    radio.read(&data, sizeof(data));
    for(int i = 0; i < 3; i++)
    {
      Serial.print(data[i]);
      Serial.print('\t');
    }
    for(int i = 0; i<5;i++){
      Serial.print(data[i+3],6);
      Serial.print('\t');
    }
    Serial.print(sizeof(data));
    Serial.println();
  }
}


// notes
/*
Bibliography (spec. for modified code):
https://github.com/arashmarzi/speedometer/blob/master/speedometer.ino
https://forum.arduino.cc/t/simple-nrf24l01-2-4ghz-transceiver-demo/405123/2
https://aprs.gids.nl/nmea/
https://randomnerdtutorials.com/guide-to-neo-6m-gps-module-with-arduino/
https://lastminuteengineers.com/neo6m-gps-arduino-tutorial/
https://learnarduinonow.com/2014/02/22/arduino-ublox-neo-6m-gps.html
*/