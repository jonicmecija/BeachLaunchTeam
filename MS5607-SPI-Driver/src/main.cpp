#include <Arduino.h>
#include <MS5607-Driver.h>


MS5607 newSensor;


void setup() {
  // put your setup code here, to run once:
  SPI.begin();
  Serial.begin(9600);
  newSensor.reset();
  delay(20);
  Serial.println("before init");
  newSensor.MS5607Init();
  Serial.println("after init");

  // Serial.println();
  newSensor.printCoefficients();
  newSensor.setSampleRateTemp(newSensor.TEMP_SAMPLING_256); // set sensor sampling rate
  Serial.println("temp rate: ");
  Serial.println(newSensor.getSampleRateTemp());

  newSensor.setSampleRatePressure(newSensor.PRESSURE_SAMPLING_256); // set pressure sampling rate
  Serial.println("pressure rate set");
  Serial.println(newSensor.getSampleRatePressure());


  // newSensor.printCoefficients();
}

void loop() {
  delay(500);
  
  Serial.print("Temperature (Degrees Fahrenheit): ");
  Serial.println((newSensor.getTemperature()/100)*1.8+32);

  Serial.print("Pressure (mBar): ");
  Serial.println(newSensor.getPressure()/100);


}