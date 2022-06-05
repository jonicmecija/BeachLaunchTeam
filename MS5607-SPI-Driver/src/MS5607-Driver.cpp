/*!
 *  @file MS5607-Driver.cpp
 *
 *  This is a library for the MS5607-02BA03 Barometric Pressure Sensor.
 *
 *  This sensor uses SPI to communicate.
 * 
 *  By: Jonic Mecija
 */

#include <MS5607-Driver.h>

/*!
 *  @brief Initializes the sensor.
 */
void MS5607::MS5607Init(){
    pinMode(CS_PIN,OUTPUT);
    readCoefficients();
}

/*!
 *  @brief  Resets the chip via soft reset
 */
void MS5607::reset(){
    SPI.beginTransaction(SPISettings(SPI_RATE, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN,LOW);
    SPI.transfer(RESET_COMMAND);
    digitalWrite(CS_PIN,HIGH);
    SPI.endTransaction();
}

/*!
 * Reads a 16 bit value given an address.
 * @param addr
 *        The address for a value to be read from.
 * @return 16-bit value.
 */
uint16_t MS5607::read16(uint8_t addr){
    uint16_t coeff = 0;

    SPI.beginTransaction(SPISettings(SPI_RATE, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN,0);
    coeff = SPI.transfer(addr);
    digitalWrite(CS_PIN,1);
    SPI.endTransaction();

    return coeff;
}

/*!
 *  @brief  Reads the PROM coefficients
 */
void MS5607::readCoefficients(){
    reset();
    MS5607_calib.COEFF1 = read16(C1_ADDR);
    MS5607_calib.COEFF2 = read16(C2_ADDR);
    MS5607_calib.COEFF3 = read16(C3_ADDR);
    MS5607_calib.COEFF4 = read16(C4_ADDR);
    MS5607_calib.COEFF5 = read16(C5_ADDR);
    MS5607_calib.COEFF6 = read16(C6_ADDR);
}

/*!
 * Sets the sampling config for the device.
 * @param sampleRate
 *        The sampling scheme for temp readings.
 */
void MS5607::setSampleRateTemp(TEMP_SAMPLING_RATES sampleRate){
    MS5607_calib.D1 = sampleRate;
}

/*!
 * Sets the sampling config for the device.
 * @param sampleRate
 *        The sampling scheme for pressure readings.
 */
void MS5607::setSampleRatePressure(PRESSURE_SAMPLING_RATES sampleRate){
    MS5607_calib.D2 = sampleRate;
}

/*!
 * Reads the temperature from the device.
 * @return The temperature in degress celcius.
 */
float MS5607::getTemperature(){

    // define over sampling ratio
    SPI.beginTransaction(SPISettings(SPI_RATE, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN,0);
    SPI.transfer(MS5607_calib.D2);
    digitalWrite(CS_PIN,1);
    SPI.endTransaction();

    // get the ADC value for temperature 
    SPI.beginTransaction(SPISettings(SPI_RATE, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN,0);

    SPI.transfer(MS5607_calib.D2);

    digitalWrite(CS_PIN,1);
    SPI.endTransaction();

}

/*!
 * Reads the pressure from the device.
 * @return The pressure in milibar (mBar).
 */
float MS5607::getPressure(){

}