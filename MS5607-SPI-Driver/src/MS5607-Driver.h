/*!
 *  @file MS5607-Driver.h
 *
 *  This is a library for the MS5607-02BA03 Barometric Pressure Sensor.
 *
 *  This sensor uses SPI to communicate, 3 pins are required to interface.
 * 
 *  By: Jonic Mecija
 *  Date: 6/4/22
 */

#include <SPI.h>
#include <Arduino.h>

/* SPI Settings / Pins */
#define SPI_RATE 20000000
#define CS_PIN 10

/* MS5607 Commands */
#define RESET_COMMAND 0x1E
#define ADC_COMMAND 0x00

/* Registers available on MS5607 */
enum : const uint8_t{
    C1_ADDR = 0xA2,
    C2_ADDR = 0xA4,
    C3_ADDR = 0xA6,
    C4_ADDR = 0xA8,
    C5_ADDR = 0xAA,
    C6_ADDR = 0xAC,
};

/* Struct to hold calibration data. */
typedef struct {
    uint16_t COEFF1 = 0;
    uint16_t COEFF2 = 0;
    uint16_t COEFF3 = 0;
    uint16_t COEFF4 = 0;
    uint16_t COEFF5 = 0;
    uint16_t COEFF6 = 0;

    uint32_t D1 = 0x40;
    uint32_t D2 = 0x50;

    int32_t DT = 0;
    int32_t TEMP = 0;
    int64_t OFF = 0;
    int64_t SENS = 0;
    int32_t PRESSURE = 0;

} MS5607_calib_data;


class MS5607{
    private:
        MS5607_calib_data MS5607_calib;        
        uint8_t osrTemp; /** Temperature over-sampling ratio */
        uint8_t osrPressure; /** Pressure over-sampling ratio */
   
    public:
        
        /* Over Sampling Ratio for D1 (Temperature) */
        enum TEMP_SAMPLING_RATES{
            /** 256 over-sampling ratio */
            TEMP_SAMPLING_256 = 0x40, 
            /** 512 over-sampling ratio */
            TEMP_SAMPLING_512 = 0x42,
            /** 1024 over-sampling ratio */
            TEMP_SAMPLING_1024 = 0x44,
            /** 2048 over-sampling ratio */
            TEMP_SAMPLING_2048 = 0x46,
            /** 4096 over-sampling ratio */
            TEMP_SAMPLING_4096 = 0x48
        };

        /* Over Sampling Ratio for D2 (Pressure) */
        enum PRESSURE_SAMPLING_RATES{
            /** 256 over-sampling ratio */
            PRESSURE_SAMPLING_256 = 0x50,
            /** 512 over-sampling ratio */
            PRESSURE_SAMPLING_512 = 0x52,
            /** 1024 over-sampling ratio */
            PRESSURE_SAMPLING_1024 = 0x54,
            /** 2048 over-sampling ratio */
            PRESSURE_SAMPLING_2048 = 0x56,
            /** 4096 over-sampling ratio */
            PRESSURE_SAMPLING_4096 = 0x58
        };

        void MS5607Init();
        void reset();
        void readCoefficients();
        void setSampleRateTemp(TEMP_SAMPLING_RATES sampleRate);
        void setSampleRatePressure(PRESSURE_SAMPLING_RATES sampleRate);
        uint16_t read16(uint8_t addr);
        uint32_t getTemperature();
        uint32_t getPressure();

};