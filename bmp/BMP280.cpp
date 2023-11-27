#include "BMP280.hpp"
#define DEBUG true  

#if DEBUG == true
    #define DEBUG_REGISTER
    #define DEBUG_PRESSURE
#endif

namespace BMP280
{
    // ---------------- Constructor/Destructor ----------------
    #pragma region Constructor/Destructor
    /// @brief Default Constructor.
    /// @param spi Instance of SPI.
    /// @param cs Index of Chip-select Pin.
    BMP280::BMP280(spi_inst_t *spi, uint cs)
    {
        _spiInst = spi;
        _cs = cs;
        gpio_init(_cs);
        gpio_set_dir(_cs, true);
        gpio_put(_cs, true);

        //Default configuration.
        _powerMode = PowerMode::Sleep;
        this->getTrimmingParameters();
        this->setOversampling(Type::Temperature, 8);
        sleep_ms(100);
        this->setOversampling(Type::Pressure, 8);
        sleep_ms(100);
    }

    /// @brief Default destructor.
    BMP280::~BMP280(){

    }
    #pragma endregion

    uint8_t BMP280::readForChipID()
    {
        uint8_t result = this->readRegister(ID);
        _chipID = result;
        return result;
    }

    uint8_t BMP280::getChipID() const
    {
        return _chipID;
    }

    int32_t BMP280::getData(uint8_t reg, bool burst)
    {
        uint8_t buffer[3] = {0x00, 0x00, 0x00};
        int32_t result = 0;
        uint8_t readRegister = reg |= 0b1000000; // AND with 0b1000000 for read operation in SPI comunication.
        if (burst == true)
        {
            // Read 3 register one after another
            gpio_put(_cs, false);
            spi_write_blocking(_spiInst, &readRegister, 1);
            spi_read_blocking(_spiInst, 0, buffer, 3);
            gpio_put(_cs, true);
            // printf("Buffer[0]: %#x\n", buffer[0]);
            // printf("Buffer[1]: %#x\n", buffer[1]);
            // printf("Buffer[2]: %#x\n", buffer[2]);

            // buffer[0] is MSB... last 4 bits of buffer[2] are not taken into account, thats why its bitshifted.
            result = (((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((uint32_t)buffer[2] >> 4));
        }
        else
        {
            // Read only one register
            gpio_put(_cs, false);
            spi_write_blocking(_spiInst, &readRegister, 1);
            spi_read_blocking(_spiInst, 0, &buffer[0], 1);
            gpio_put(_cs, true);
            result = buffer[0];
        }
        return result;
    }

    // ---------------- Register ----------------
    #pragma region Register
    /// @brief Set value of specified register. After setting check parameter to true, 
    /// @brief sends SPI command to read value of this register. Defalut value of check
    /// @brief is set to False. If check is set to false,
    /// @brief function will always return true, and register's value won't be checked.
    /// @param reg Type of register.
    /// @param config Value to put into register.
    /// @param check Default value is False. True - check if value was set, False - dont check.
    /// @return true if register was set with given value, otherwise false.
    bool BMP280::setRegister(uint8_t reg, uint8_t config, bool check)
    {
        uint8_t registr = reg;
        #ifdef DEBUG_REGISTER
        printf("REGISTER - value to be set: %i\n", config);
        #endif
        uint8_t data[2] = {(reg &= 0b01111111), config}; // OR with reg because R='0' bit in SPI write operation
        gpio_put(_cs, false);
        spi_write_blocking(_spiInst, data, 2);
        gpio_put(_cs, true);
        if(check == false){
            return true;
        }
        uint8_t value = this->readRegister(registr);
        #ifdef DEBUG_REGISTER
        printf("REGISTER - set: %i\n", value);
        #endif
        if (value != config)
        {
            #ifdef DEBUG_REGISTER
            printf("REGISTER NOT set\n");
            #endif
            return false;
        }
        #ifdef DEBUG_REGISTER
        printf("REGISTER set\n");
        #endif
        return true;
    }

    /// @brief Read value of specified register via SPI command.
    /// @param reg Type of register.
    /// @return Value of specified register.
    uint8_t BMP280::readRegister(uint8_t reg)
    {
        uint8_t data = (reg | 0b10000000);
        uint8_t buffer = 0;
        gpio_put(_cs, false);
        spi_write_blocking(_spiInst, &data, 1);
        spi_read_blocking(_spiInst, 0, &buffer, 1);
        gpio_put(_cs, true);
        return buffer;
    }
    #pragma endregion

    // ---------------- Power mode ----------------
    #pragma region PowerMode
    /// @brief Set power mode of BMP280 sensor.
    /// @param mode PowerMode enum to set.
    /// @return true if action succeded, otherwise false.
    bool BMP280::setPowerMode(PowerMode mode, bool check)
    {
        uint8_t config = this->readRegister(CTRL_MEAS);
        switch (mode)
        {
        case PowerMode::Sleep:
            config = ((config & 0b11111100) | 0b00000000);
            break;
        case PowerMode::Normal:
            config = ((config & 0b11111100) | 0b00000011);
            break;
        case PowerMode::Forced:
            config = ((config & 0b11111100) | 0b00000010);
            break;
        }
        bool value = this->setRegister(CTRL_MEAS, config, check);
        if (value == true)
        {
            _powerMode = mode;
        }
        return value;
    }

    /// @brief Get current power mode via sending SPI command.
    /// @return PowerMode enum.
    PowerMode BMP280::readPowerMode()
    {
        uint8_t data = this->readRegister(CTRL_MEAS);
        data &= 0b00000011;
        PowerMode mode;
        switch (data)
        {
        case 0b00000000:
            mode = PowerMode::Sleep;
            break;
        case 0b00000011:
            mode = PowerMode::Normal;
            break;
        case 0b00000010:
            mode = PowerMode::Forced;
            break;
        case 0b00000001:
            mode = PowerMode::Forced;
            break;
        }
        _powerMode = mode;
        return mode;
    }

    /// @brief Get PowerMode value of object variable.
    /// @return PowerMode enum.
    PowerMode BMP280::getPowerMode() const
    {
        return _powerMode;
    }
    #pragma endregion

    // ---------------- Oversampling ----------------
    #pragma region Oversampling
    /// @brief Set oversampling of BMP280 sensor via SPI command.
    /// @param type Type::Temperature or Type::Pressure.
    /// @param oversampling Accetps uint8_t values = {0, 1, 2, 4, 8, 16 }.
    /// @param check Default value is set to false. True - check if value was set, False - dont check.
    /// @return True if register was set with given value, otherwise False.
    bool BMP280::setOversampling(Type type, uint8_t oversampling, bool check){
        uint8_t registerValue = this->readRegister(CTRL_MEAS);
        //Out of range.
        if(oversampling != 0 and oversampling != 1 and oversampling != 2 and oversampling != 4 and
            oversampling != 8 and oversampling != 16){
            return false;
        }
        uint8_t value = 0;
        switch(oversampling){
            case 0:
                value = 0;
            break;
            case 1:
                value = 1;
            break;
            case 2:
                value = 2;
            break;
            case 4:
                value = 3;  
            break;
            case 8:
                value = 4;
            break;
            case 16:
                value = 5;
            break;
        }
        switch(type){
            case Type::Temperature:
            value <<= 5;
            registerValue = ((registerValue & 0b00011111) | value);
            break;
            case Type::Pressure:
            value <<= 2;
            registerValue = ((registerValue & 0b11100011) | value);
            break;
        }
        bool result = this->setRegister(CTRL_MEAS, registerValue, check);
        if(result == true){
            switch(type){
                case Type::Temperature:
                    _temperatureOversampling = oversampling;
                    break;
                case Type::Pressure:
                    _pressureOversampling = oversampling;
                    break;
            }
        }
        return result;
    }

    /// @brief Read current temperature oversampling via SPI command.
    /// @param type Type::Temperature or Type::Pressure.
    /// @return Value of the oversampling setting.
    uint8_t BMP280::readOversampling(Type type){
        uint8_t reg = this->readRegister(CTRL_MEAS);
        uint8_t result = 0;
        switch (type)
        {
        case Type::Pressure:
            reg &= 0b00011100;
            reg >>= 2;
            if(reg == 0){
                result = 0;
            } else if(reg == 1){
                result = 1;
            } else if(reg == 2){
                result = 2;
            } else if(reg == 3){
                result = 4;
            } else if(reg == 4){
                result = 8;
            } else{
                result = 16;
            }
            _pressureOversampling = result;
            return result;
            break;
        case Type::Temperature:
            reg &= 0b11100000;
            reg >>= 5;
            if(reg == 0){
                result = 0;
            } else if(reg == 1){
                result = 1;
            } else if(reg == 2){
                result = 2;
            } else if(reg == 3){
                result = 4;
            } else if(reg == 4){
                result = 8;
            } else if(reg == 5 or reg == 6 or reg == 7){
                result = 16;
            }
            _temperatureOversampling = result;
            return result;
            break;
        default:
            return 0x00;
            break;
        }
    }

    /// @brief Get oversampling value of object variable.
    /// @param type Type::Temperature or Type::Pressure.
    /// @return Oversampling config.
    uint8_t BMP280::getOversampling(Type type) const{
        switch(type){
            case Type::Temperature:
                return _temperatureOversampling;
                break;
            case Type::Pressure:
                return _pressureOversampling;
                break;
            default:
                return 0x00;
                break;
        }
    }
    #pragma endregion

    // ---------------- Temperature ----------------
    #pragma region Temperature
    /// @brief Read current temperature in raw format via SPI command.
    /// @return Temperature in raw format.
    int32_t BMP280::readRawTemperature()
    {
        //Based of datasheet formula.
        int32_t adc_T = this->getData(TEMP_MSB, true);
        int32_t var1, var2, temp;
        var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        _tFine = var1 + var2;
        temp = (_tFine * 5 + 128) >> 8;
        _rawTemperature = temp;
        return temp;
    }

    /// @brief Get temperature of object variable.
    /// @return Temeperature in raw format.
    int32_t BMP280::getRawTemperature() const
    {
        return _rawTemperature;
    }

    /// @brief Read temperature in celsius deg double format via SPI command.
    /// @return Temperature in celsius deg double.
    double BMP280::readTemperature()
    {
        double temp = this->readRawTemperature() / 100.0;
        _temperature = temp;
        return (this->readRawTemperature() / 100.0);
    }

    /// @brief Get temperature of object variable.
    /// @return Temperature in celsius deg double.
    double BMP280::getTemperature() const
    {
        return _temperature;
    }
    #pragma endregion

    // ---------------- Pressure ----------------
    #pragma region Pressure
    /// @brief Read current pressure in raw format via SPI command.
    /// @return Pressure in raw format.
    uint32_t BMP280::readRawPressure()
    {
        //Based on datasheet formula.
        int32_t adc_P = this->getData(PRESS_MSB, true);
        #ifdef DEBUG_PRESSURE
        printf("PRESSURE - data from register: %i\n", adc_P);
        #endif
        int64_t var1, var2, p;
        var1 = ((int64_t)this->_tFine) - 128000;
        var2 = var1 * var1 * (int64_t)this->dig_P6;
        var2 = var2 + ((var1*(int64_t)this->dig_P5)<<17);
        var2 = var2 + (((int64_t)this->dig_P4)<<35);
        var1 = ((var1 * var1 * (int64_t)this->dig_P3)>>8) + ((var1 * (int64_t)this->dig_P2)<<12);
        var1 = (((((int64_t)1)<<47)+var1)) * ((int64_t)this->dig_P1)>>33;
        if(var1 == 0){
            return 0;
        }
        p = 1048576 - adc_P;
        p = (((p<<31)-var2)*3125)/var1;
        var1 = (((int64_t)this->dig_P9)*(p>>13)*(p>>13))>>25;
        var2 = (((int64_t)this->dig_P8)*p)>>19;
        p = ((p+var1+var2) >> 8) + (((int64_t)this->dig_P7)<<4);
        _rawPressure = (uint32_t)p;
        #ifdef DEBUG_PRESSURE
        printf("PRESSURE - pressure after calculation: %i\n", p);
        #endif
        return _rawPressure;
    }

    /// @brief Get pressure of object variable.
    /// @return Pressure in raw format.
    uint32_t BMP280::getRawPressure() const
    {
        return _rawPressure;
    }

    /// @brief Read pressure in celsius deg double format via SPI command.
    /// @param unit If != 0 method returns pressure in [hPa], returns pressure in [Pa] by default.
    /// @return Pressure [Pa] or [hPa] in double.
    double BMP280::readPressure(int unit)
    {
        double pressure = this->readRawPressure() / 256;
        _pressure = pressure;
        return (unit == 0 ? pressure : ( pressure / 100.0));
    }

    /// @brief Get pressure of object variable.
    /// @param unit If != 0 method returns pressure in [hPa], returns pressure in [Pa] by default.
    /// @return Pressure [Pa] or [hPa] in double.
    double BMP280::getPressure(int unit) const
    {
        return (unit == 0 ? _pressure : (_pressure / 100.0));
    }
    #pragma endregion

    // ---------------- Utilities ----------------
    #pragma region Utilities
    /// @brief Power on reset of BMP280 module.
    void BMP280::reset(){
        this->setRegister(RESET, 0xB6);
    }

    /// @brief Get trimming parameters required to calculate temperature and pressure.
    void BMP280::getTrimmingParameters()
    {
        uint8_t reg = 0x88 | 0x80;
        uint8_t buffer[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        gpio_put(_cs, false);
        spi_write_blocking(_spiInst, &reg, 1);
        spi_read_blocking(_spiInst, 0, buffer, 24);
        gpio_put(_cs, true);
        dig_T1 = (buffer[1] << 8) | buffer[0];
        dig_T2 = (buffer[3] << 8) | buffer[2];
        dig_T3 = (buffer[5] << 8) | buffer[4];
        dig_P1 = (buffer[7] << 8) | buffer[6];
        dig_P2 = (buffer[9] << 8) | buffer[8];
        dig_P3 = (buffer[11] << 8) | buffer[10];
        dig_P4 = (buffer[13] << 8) | buffer[12];
        dig_P5 = (buffer[15] << 8) | buffer[14];
        dig_P6 = (buffer[17] << 8) | buffer[16];
        dig_P7 = (buffer[19] << 8) | buffer[18];
        dig_P8 = (buffer[21] << 8) | buffer[20];
        dig_P9 = (buffer[23] << 8) | buffer[22];
    }
    #pragma endregion
}