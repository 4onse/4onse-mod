// temperature
#include <OneWire.h>
#include <DallasTemperature.h>


// internal temp
#include <DHT.h>

#define ONE_WIRE_BUS 9 // 9 ext-temp or 11 aux-temp
#define LIGHT 7

#define DHTPIN 10
#define DHTTYPE DHT11

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dstemp(&oneWire);

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("TEST DS18B20 and DHT"); 
  // Start up the library 
  dstemp.begin();
  // check DHT
  dht.begin();

}

void loop() {
  dstemp.requestTemperatures();

  delay(150);

  float temp = dstemp.getTempCByIndex(0);
  Serial.print("DS18B20: ");
  Serial.println(temp);
  // read int temp
  float intTemp = dht.readTemperature();
  Serial.print("DHT11: ");
  Serial.println(intTemp);
  delay(5000);

}
