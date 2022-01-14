//
// Created by Paweł on 14.01.2022.
//
//
//
/*
 * to read pressure init Sensor_BMP180 object with handle to i2c
 * and then you can read with methods
 *
 */
#ifndef CZUJNIK_TEMP_BMP180_HPP
#define CZUJNIK_TEMP_BMP180_HPP


#define BMP180_OVERSAMPLING  3//set it from 0 to 3, higher increase precision but also increases needed time for sampling
#define BMP180_TEMP_SAMPLING_TIME 0.01f // must be at least 5ms
#define BMP180_PRESS_SAMPLING_TIME 0.05f //depends on oversampling (for 3 it should work down to 25ms, for lower check documentation)

#define CTRL_REG 0xF4


uint16_t read_MSB_LSB(int handle, uint8_t r1, uint8_t r2);
uint32_t read_MSB_LSB_XLSB(int handle, uint8_t r1, uint8_t r2, uint8_t r3);

struct Calib_param{
    short AC1 = 0;
    short AC2 = 0;
    short AC3 = 0;
    unsigned short AC4 = 0;
    unsigned short AC5 = 0;
    unsigned short AC6 = 0;
    short B1 = 0;
    short B2 = 0;
    short MB = 0;
    short MC = 0;
    short MD = 0;
};

class Sensor_BMP180{
public:
    Sensor_BMP180() = delete;
    Sensor_BMP180(int handle):i2c_handle_(handle){};
    void calculate_press_and_temp();
    double get_pressure() const{return pressure_;}
    double get_temperature() const{return temperature_;}
    //to get well estimated altitude you must know sea level pressure ( you can find in in any weather forecast)
    double get_estimated_altitude(double sea_level_pressure = 1020) const;
private:
    void read_calib_param();
    Calib_param calibParam_;
    int i2c_handle_;
    double pressure_;
    double temperature_;
};
#endif //CZUJNIK_TEMP_BMP180_HPP
