#include <iostream>
#include <pigpio.h>
#include <iostream>
#include <fstream>
#include "include/BMP180.hpp"
#include <iomanip>
#include <ctime>

int main() {
    gpioInitialise();
    time_t rawtime;
    tm *curr_tim;
    int i2c_handle = i2cOpen(1, 0x77, 0);
    Sensor_BMP180 sensor(i2c_handle);
    std::ofstream temp_readings;
    temp_readings<<std::fixed<<std::setprecision(2)<<std::endl;
    std::ofstream pressure_readings;
    pressure_readings<<std::fixed<<std::setprecision(2)<<std::endl;
    int key = 'r';
    int i = 1;
    temp_readings.open("temp.dat", std::ios::app);
    pressure_readings.open("press.dat", std::ios::app);
    while(true){
        time(&rawtime);
        curr_tim = localtime(&rawtime);
        sensor.calculate_press_and_temp();
        std::cout <<"current temperature: "<<sensor.get_temperature()<<" °C"<< std::endl;
        std::cout <<"current pressure: "<<sensor.get_pressure() <<" hPa" <<std::endl;
        std::cout <<"estimated altitude: "<<sensor.get_estimated_altitude() <<" m" <<std::endl;
        if(temp_readings.is_open()){
            temp_readings<<sensor.get_temperature()<<" "<<curr_tim->tm_year+1900<<"-"<<curr_tim->tm_mon<<"-"<<curr_tim->tm_mday<<"|"<<curr_tim->tm_hour<<":"<<curr_tim->tm_min<<":"<<curr_tim->tm_sec<<"\n";
        }
        if(pressure_readings.is_open()){
            pressure_readings<<sensor.get_pressure()<<" "<<curr_tim->tm_year+1900<<"-"<<curr_tim->tm_mon<<"-"<<curr_tim->tm_mday<<"|"<<curr_tim->tm_hour<<":"<<curr_tim->tm_min<<":"<<curr_tim->tm_sec<<"\n";
        }
        if(!(i%10)){
            std::cout <<"\nsaved to file\n"<<std::endl;
            temp_readings.close();
            pressure_readings.close();
            time_sleep(0.01);
            temp_readings.open("temp.dat",std::ios::app);
            pressure_readings.open("press.dat",std::ios::app);
        }
        i++;
        time_sleep(30);
    }

    i2cClose(i2c_handle);
    return 0;
}
