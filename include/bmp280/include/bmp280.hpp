#pragma once
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

#pragma region BMP280_Definitions
#define ID 0xD0
#define RESET 0xE0
#define STATUS 0xF3
#define CTRL_MEAS 0xF4
#define CONFIG 0xF5
#define PRESS_MSB 0xF7
#define PRESS_LSB 0xF8
#define PRESS_XLSB 0xF9
#define TEMP_MSB 0xFA
#define TEMP_LSB 0xFB
#define TEMP_XLSB 0xFC
#pragma endregion

//Module datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf

namespace BMP280{
    enum PowerMode{
        Sleep = 0,
        Forced = 1, 
        Normal = 2
    };

    enum Type{
        Temperature,
        Pressure
    };

    class BMP280{
    public:
        BMP280(spi_inst_t *spi, uint cs);
        ~BMP280();

        uint8_t readForChipID();
        uint8_t getChipID() const;
        int32_t getData(uint8_t reg, bool burst);

        bool setRegister(uint8_t reg, uint8_t config, bool check = false);
        uint8_t readRegister(uint8_t reg);

        bool setPowerMode(PowerMode mode, bool check = false);
        PowerMode readPowerMode();
        PowerMode getPowerMode() const;

        void reset();

        bool setOversampling(Type type, uint8_t oversampling, bool check = false);
        uint8_t readOversampling(Type type);
        uint8_t getOversampling(Type type) const;

        int32_t readRawTemperature();
        int32_t getRawTemperature() const;
        double readTemperature();
        double getTemperature() const;

        uint32_t readRawPressure();
        uint32_t getRawPressure() const;
        double readPressure(int unit = 0);
        double getPressure(int unit = 0) const;
    private:
        spi_inst_t *_spiInst;
        uint _cs;
        uint8_t _chipID;
        uint8_t _temperatureOversampling;
        uint8_t _pressureOversampling;

        PowerMode _powerMode;

        int32_t _rawTemperature;
        double _temperature;

        uint32_t _rawPressure;
        double _pressure;

        int32_t _tFine;
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;
        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;

    private:
        void getTrimmingParameters();
    };
}