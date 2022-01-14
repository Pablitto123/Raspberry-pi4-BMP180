//
// Created by Paweł on 14.01.2022.
//

#ifndef CZUJNIK_TEMP_BMP180_HPP
#define CZUJNIK_TEMP_BMP180_HPP
#define CTRL_REG 0xF4

uint16_t read_MSB_LSB(int handle, uint8_t r1, uint8_t r2);

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
    Sensor_BMP180(int handle):i2c_handle_(handle){};
    void read_calib_param();
    double read_temperature();
    double read_pressure(uint8_t oversampling = 3);
    void read_press_and_temp();
    double get_pressure() const{return pressure_;}
    double get_temperature() const{return temperature_;}
    double get_estimated_altitude(double sea_level_pressure = 1020) const;
    Calib_param calibParam_;
private:
    int i2c_handle_;
    double pressure_;
    double temperature_;
};
#endif //CZUJNIK_TEMP_BMP180_HPP
