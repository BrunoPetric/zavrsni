#include <Arduino.h>
#include "MHZ19.h"                                        
#include <SoftwareSerial.h>     
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"  

Adafruit_BME680 bme; // I2C

#define RX_PIN 18                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 19                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)

MHZ19 myMHZ19;                                             // Constructor for library
SoftwareSerial mySerial(RX_PIN, TX_PIN);                   // (Uno example) create device to MH-Z19 serial

unsigned long getDataTimer = 0;

void setup()
{
    Serial.begin(115200);                                     // Device to serial monitor feedback
    while (!Serial);
      Serial.println(F("BME680 async test"));

  if (!bme.begin(0x76)) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }
    mySerial.begin(BAUDRATE);                               // (Uno example) device to MH-Z19 serial start   
    myMHZ19.begin(mySerial);                                // *Serial(Stream) refence must be passed to library begin(). 

    myMHZ19.autoCalibration();                              // Turn auto calibration ON (OFF autoCalibration(false))

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop()
{
    if (millis() - getDataTimer >= 2000)
    {
        int CO2; 

        /* note: getCO2() default is command "CO2 Unlimited". This returns the correct CO2 reading even 
        if below background CO2 levels or above range (useful to validate sensor). You can use the 
        usual documented command with getCO2(false) */

        CO2 = myMHZ19.getCO2();                             // Request CO2 (as ppm)
        
        Serial.print("MHZ19 data: CO2 (ppm): ");                      
        Serial.print(CO2);                                

        int8_t Temp;
        Temp = myMHZ19.getTemperature();                     // Request Temperature (as Celsius)
        Serial.print(" Temperature (C): ");                  
        Serial.println(Temp);  

if (! bme.performReading()) 
  {
    Serial.println("Failed to perform reading :(");
    return;
  }

  Serial.print(F("BME680 data: Temperature = "));
  Serial.print(bme.temperature);
  Serial.print(F(" *C"));

  Serial.print(F(" Pressure = "));
  Serial.print(bme.pressure / 100.0);
  Serial.print(F(" hPa"));

  Serial.print(F(" Humidity = "));
  Serial.print(bme.humidity);
  Serial.println(F(" %"));
                       

        getDataTimer = millis();
    }
}
