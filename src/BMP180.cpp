//
// Created by Paweł on 14.01.2022.
//
#include <pigpio.h>
#include "../include/BMP180.hpp"
#include <cmath>


uint16_t read_MSB_LSB(int handle, uint8_t r1, uint8_t r2){
    uint8_t MSB = i2cReadByteData(handle,r1);
    uint8_t LSB = i2cReadByteData(handle,r2);
    uint16_t UT = (uint16_t)LSB + (uint16_t) (MSB << 8);
    return UT;
}

uint32_t read_MSB_LSB_XLSB(int handle, uint8_t r1, uint8_t r2, uint8_t r3){
    uint8_t MSB = i2cReadByteData(handle,r1);
    uint8_t LSB = i2cReadByteData(handle,r2);
    uint8_t XLSB = i2cReadByteData(handle,r3);
    uint32_t UT = ((uint32_t) ((uint32_t)MSB << 16) + (uint32_t)((uint32_t)LSB << 8) + (uint32_t)XLSB);
    return UT;
}

void Sensor_BMP180::read_calib_param() {
    calibParam_.AC1 =  read_MSB_LSB(i2c_handle_, 0xAA,0xAB);
    calibParam_.AC2 =  read_MSB_LSB(i2c_handle_, 0xAC,0xAD);
    calibParam_.AC3 =  read_MSB_LSB(i2c_handle_, 0xAE,0xAF);
    calibParam_.AC4 =  read_MSB_LSB(i2c_handle_, 0xB0,0xB1);
    calibParam_.AC5 =  read_MSB_LSB(i2c_handle_, 0xB2,0xB3);
    calibParam_.AC6 =  read_MSB_LSB(i2c_handle_, 0xB4,0xB5);
    calibParam_.B1 =  read_MSB_LSB(i2c_handle_, 0xB6,0xB7);
    calibParam_.B2 =  read_MSB_LSB(i2c_handle_, 0xB8,0xB9);
    calibParam_.MB = read_MSB_LSB(i2c_handle_, 0xBA, 0xBB);
    calibParam_.MC = read_MSB_LSB(i2c_handle_, 0xBC, 0xBD);
    calibParam_.MD = read_MSB_LSB(i2c_handle_, 0xBE, 0xBF);
}
/*
 * It is fucked                   ||||
 *                                VVVV
 */
void Sensor_BMP180::calculate_press_and_temp() {
    read_calib_param();
    i2cWriteByteData(i2c_handle_,CTRL_REG,0x2E);
    time_sleep(BMP180_TEMP_SAMPLING_TIME);
    uint16_t UT = read_MSB_LSB(i2c_handle_, 0xF6, 0xF7);
    i2cWriteByteData(i2c_handle_,CTRL_REG,0x34 + (BMP180_OVERSAMPLING<<6));
    time_sleep(BMP180_PRESS_SAMPLING_TIME);
    uint32_t UP = read_MSB_LSB_XLSB(i2c_handle_,0xF6,0xF7,0xF8)>>(8-BMP180_OVERSAMPLING);
    long X1 = (long)((UT - calibParam_.AC6)*calibParam_.AC5/(1<<15));
    long X2 = (long)(calibParam_.MC*(1<<11)/(X1 +calibParam_.MD));
    long B5 = X1 + X2;
    long temp = (B5+8)/(1<<4);
    temperature_ = (double)temp / 10;
    long B6 = B5 - 4000;
    X1 = ((calibParam_.B2*(B6*B6/(1<<12)))/(1<<11));
    X2 = calibParam_.AC2*B6/(1<<11);
    long X3 = X1 + X2;
    long B3 = ((((long)calibParam_.AC1*4+X3)<<BMP180_OVERSAMPLING)+2)/4;
    X1 = calibParam_.AC3*B6/(1<<13);
    X2 = (calibParam_.B1*(B6*B6/(1<<12)))/(1<<16);
    X3 = ((X1+X2) + 2)/(1<<2);
    long p;
    unsigned long B4 = calibParam_.AC4*(unsigned long)(X3 + 32768)/(1<<15);
    unsigned long B7 = ((unsigned long)UP - B3)*(50000>>BMP180_OVERSAMPLING);
    if(B7< 0x80000000){
        p = (B7*2)/B4;
    }
    else{
        p = (B7/B4)*2;
    }
    X1 = (p/(1<<8))*(p/(1<<8));
    X1 = (X1 * 3038)/(1<<16);
    X2 = (-7357*p)/(1<<16);
    p = p + (X1 + X2 +3791)/(1<<4);
    pressure_ = (double)p/100;
}

double Sensor_BMP180::get_estimated_altitude(double sea_level_pressure) const {
    return 44330*(1-pow(pressure_/sea_level_pressure,1/5.255));
}
