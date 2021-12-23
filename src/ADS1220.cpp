#include <Arduino.h>
#include "ADS1220.h"
#include <SPI.h>
//#include <stdint.h>

ADS1220::ADS1220(){

}

void ADS1220::WriteConfig (uint8_t address, uint8_t value){
    digitalWrite(default_cs_pin, LOW);
    SPI.transfer (WREG | (address<<2));
    SPI.transfer (value);
    digitalWrite(default_cs_pin, HIGH);
}

void ADS1220::ReadConfig (uint8_t address){
    uint8_t register_value;

    digitalWrite(default_cs_pin, LOW);
    SPI.transfer (RREG | (address << 2));
    register_value = SPI.transfer(SPI_READ);
    digitalWrite(default_cs_pin, HIGH);
    Serial.print ("Register " + String(address) + " = " + String (register_value) + "\n\n");
}

void ADS1220::SetDefaultSettings (void){
    WriteConfig (config_address_reg0, default_value_reg0);
    WriteConfig (config_address_reg1, default_value_reg1);
    WriteConfig (config_address_reg2, default_value_reg2);
    WriteConfig (config_address_reg3, default_value_reg3);
}

void ADS1220::begin (void){
    pinMode(default_cs_pin, OUTPUT);
    pinMode(default_rdy_pin, INPUT);
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE1);
    SetDefaultSettings();
}

void ADS1220::begin(uint8_t cs_pin, uint8_t rdy_pin){
    default_cs_pin = cs_pin;
    default_rdy_pin = rdy_pin;

    pinMode(default_cs_pin, OUTPUT);
    pinMode(default_rdy_pin, INPUT);

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE1);
    SetDefaultSettings();
}

void ADS1220::ADS1220_START (void){
    digitalWrite(default_cs_pin, LOW);
    SPI.transfer(START);
    digitalWrite(default_cs_pin, HIGH);
}

void ADS1220::ADS1220_RESET (void){
    digitalWrite(default_cs_pin, LOW);
    SPI.transfer(RESET); 
    digitalWrite(default_cs_pin, HIGH);
}



void ADS1220::SetSettings (uint8_t address, uint8_t value){

}







