#include <Arduino.h>
#include <MS5607-Driver.h>

void setup() {
  // put your setup code here, to run once:
  MS5607 newSensor;

  newSensor.MS5607Init();
  newSensor.reset();
  newSensor.setSampleRateTemp(newSensor.TEMP_SAMPLING_256);
  newSensor.setSampleRatePressure(newSensor.PRESSURE_SAMPLING_256);
  // newSensor.
}

void loop() {
  // put your main code here, to run repeatedly:
}