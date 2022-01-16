#include <Arduino.h>
#include "ADS1220.h"
#include <SPI.h>

ADS1220::ADS1220()
{
}

void ADS1220::begin(void)
{
    pinMode(default_cs_pin, OUTPUT);
    pinMode(default_rdy_pin, INPUT);

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE1);

    // SetDefaultSettings();
}

void ADS1220::begin(uint8_t cs_pin, uint8_t rdy_pin)
{
    default_cs_pin = cs_pin;
    default_rdy_pin = rdy_pin;

    pinMode(default_cs_pin, OUTPUT);
    pinMode(default_rdy_pin, INPUT);

    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE1);

    // SetDefaultSettings();
}

void ADS1220::WriteConfig(uint8_t address, uint8_t value)
{
    digitalWrite(default_cs_pin, LOW);
    SPI.transfer(WREG | (address << 2));
    SPI.transfer(value);
    digitalWrite(default_cs_pin, HIGH);
}

uint8_t ADS1220::ReadConfig(uint8_t address)
{
    uint8_t register_value;

    digitalWrite(default_cs_pin, LOW);
    SPI.transfer(RREG | (address << 2));
    register_value = SPI.transfer(SPI_READ);
    digitalWrite(default_cs_pin, HIGH);

    Serial.print("Register" + String(address) + " = " + String(register_value, HEX) + "\n");
    return register_value;
}

void ADS1220::GetRegistersValue(void)
{
    Serial.print("Register 0 - ");
    Serial.println(config_register0_value, HEX);
    Serial.print("Register 1 - ");
    Serial.println(config_register1_value, HEX);
    Serial.print("Register 2 - ");
    Serial.println(config_register2_value, HEX);
    Serial.print("Register 3 - ");
    Serial.println(config_register3_value, HEX);
}

void ADS1220::SetExternalVref(float ext_vref)
{
    vref = ext_vref;
}

void ADS1220::SetDefaultSettings(void)
{
    WriteConfig(config_address_reg0, default_value_reg0);
    WriteConfig(config_address_reg1, default_value_reg1);
    WriteConfig(config_address_reg2, default_value_reg2);
    WriteConfig(config_address_reg3, default_value_reg3);
    Serial.println("Default settings are set");
    config_register0_value = ReadConfig(config_address_reg0);
    config_register1_value = ReadConfig(config_address_reg1);
    config_register2_value = ReadConfig(config_address_reg2);
    config_register3_value = ReadConfig(config_address_reg3);

    vref = 2.048;
    ads_pga = 1;
    vfsr = vref / ads_pga;
}

// Register 0 configuration metods
void ADS1220::PGA(int pga_mode)
{
    config_register0_value &= ~MASK_PGA_BYPASS;
    config_register0_value |= pga_mode;
    WriteConfig(config_address_reg0, config_register0_value);
}

void ADS1220::Gain(int gain)
{
    config_register0_value &= ~MASK_GAIN;
    config_register0_value |= gain;
    WriteConfig(config_address_reg0, config_register0_value);
    switch (gain)
    {
    case GAIN_1:
        ads_pga = 1;
        break;
    case GAIN_2:
        ads_pga = 2;
        break;
    case GAIN_4:
        ads_pga = 4;
        break;
    case GAIN_8:
        ads_pga = 8;
        break;
    case GAIN_16:
        ads_pga = 16;
        break;
    case GAIN_32:
        ads_pga = 32;
        break;
    case GAIN_64:
        ads_pga = 64;
        break;
    case GAIN_128:
        ads_pga = 128;
        break;
    default:
        ads_pga = 1;
        break;
    }
    vfsr = vref / ads_pga;
}

void ADS1220::MuxChanel(int mux_chanel)
{
    config_register0_value &= ~MASK_MUX;
    config_register0_value |= mux_chanel;
    WriteConfig(config_address_reg0, config_register0_value);
}

// Register 1 configuration metods
void ADS1220::BCS(int bcs_mode)
{
    config_register1_value &= ~MASK_BCS;
    config_register1_value |= bcs_mode;
    WriteConfig(config_address_reg1, config_register1_value);
}

void ADS1220::TemperatureSensor(int ts_mode)
{
    config_register1_value &= ~MASK_TS;
    config_register1_value |= ts_mode;
    WriteConfig(config_address_reg1, config_register1_value);
}

void ADS1220::ConversionMode(int conversion_mode)
{
    config_register1_value &= ~MASK_CM;
    config_register1_value |= conversion_mode;
    WriteConfig(config_address_reg1, config_register1_value);

    if (conversion_mode == CONTINUOUS_MODE)
    {
        Start();
    }
}

void ADS1220::OperatingMode(int operating_mode)
{
    config_register1_value &= ~MASK_MODE;
    config_register1_value |= operating_mode;
    WriteConfig(config_address_reg1, config_register1_value);
}

void ADS1220::DataRate(int data_rate_mode)
{
    config_register1_value &= ~MASK_DR;
    config_register1_value |= data_rate_mode;
    WriteConfig(config_address_reg1, config_register1_value);
}

// Register 2 configuration metods
void ADS1220::IDAC(int idac_current)
{
    config_register2_value &= ~MASK_IDAC;
    config_register2_value |= idac_current;
    WriteConfig(config_address_reg2, config_register2_value);
}

void ADS1220::PSW(int psw_mode)
{
    config_register2_value &= ~MASK_PSW;
    config_register2_value |= psw_mode;
    WriteConfig(config_address_reg2, config_register2_value);
}

void ADS1220::FIR(int fir_mode)
{
    config_register2_value &= ~MASK_FIR;
    config_register2_value |= fir_mode;
    WriteConfig(config_address_reg2, config_register2_value);
}

void ADS1220::VREF(int vref_mode)
{
    config_register2_value &= ~MASK_VREF;
    config_register2_value |= vref_mode;
    WriteConfig(config_address_reg2, config_register2_value);

    if (vref_mode == VREF_INTERNAL)
    {
        vref = 2.048;
    }
    else
    {
        Serial.println("You set external ref voltage.");
        Serial.println("Please enter ref voltage value. Using the 'SetExternalVref(float ext_vref)' method.");
    }
}

// Register 3 configuration metods
void ADS1220::DRDYM(int vref_mode)
{
    config_register3_value &= ~MASK_DRDYM;
    config_register3_value |= vref_mode;
    WriteConfig(config_address_reg3, config_register3_value);
}

void ADS1220::I2MUX(int i2mux_mode)
{
    config_register3_value &= ~MASK_I2MUX;
    config_register3_value |= i2mux_mode;
    WriteConfig(config_address_reg3, config_register3_value);
}

void ADS1220::I1MUX(int i1mux_mode)
{
    config_register3_value &= ~MASK_I1MUX;
    config_register3_value |= i1mux_mode;
    WriteConfig(config_address_reg3, config_register3_value);
}

// Read ADC convertation data
void ADS1220::SetInterrupt(uint8_t ads_interrupt) // automatic use of interrupts or setting interrupts manually in the program sketch
{
    interrupt = ads_interrupt;
}

int32_t ADS1220::ReadContinuous(void) // Read ADC converting value if Input multiplexer configuration installed earlier
{
    static byte SPI_buffer[3];
    int32_t result_32bit = 0;
    long int bit24;

    digitalWrite(default_cs_pin, LOW);

    // delayMicroseconds(100);
    if (interrupt == INTERNAL)
    {
        if (digitalRead(default_rdy_pin) == LOW)
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                SPI_buffer[i] = SPI.transfer(SPI_READ);
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            SPI_buffer[i] = SPI.transfer(SPI_READ);
        }
    }

    // delayMicroseconds(100);

    digitalWrite(default_cs_pin, HIGH);

    bit24 = SPI_buffer[0]; // Writing  with 8 bit shift to left
    bit24 = (bit24 << 8) | SPI_buffer[1];
    bit24 = (bit24 << 8) | SPI_buffer[2];

    bit24 = (bit24 << 8);
    result_32bit = (bit24 >> 8);

    return result_32bit;
}

int32_t ADS1220::ReadContinuousChanel(int mux_chanel) // Read ADC converting value if you want set Input multiplexer configuration in method
{
    MuxChanel(mux_chanel); // select chanels to convert

    static byte SPI_buffer[3];
    int32_t result_32bit = 0;
    long int bit24;
    uint8_t timer = 0;

    digitalWrite(default_cs_pin, LOW);

    // delayMicroseconds(100);
    if (interrupt == INTERNAL)
    {
        while (digitalRead(default_rdy_pin) == HIGH) //
        {

        }
        if (digitalRead(default_rdy_pin) == LOW)
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                SPI_buffer[i] = SPI.transfer(SPI_READ);
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            SPI_buffer[i] = SPI.transfer(SPI_READ);
        }
    }
    // delayMicroseconds(100);

    digitalWrite(default_cs_pin, HIGH);

    bit24 = SPI_buffer[0]; // Writing  with 8 bit shift to left
    bit24 = (bit24 << 8) | SPI_buffer[1];
    bit24 = (bit24 << 8) | SPI_buffer[2];

    bit24 = (bit24 << 8);
    result_32bit = (bit24 >> 8);

    return result_32bit;
}

int32_t ADS1220::ReadSingleShot(void) // Read ADC converting value if select Single-shot mode and go to poweroff
{
    static byte SPI_buffer[3];
    int32_t result_32bit = 0;
    long int bit24;

    Start();

    // delayMicroseconds(100);

    if (interrupt == INTERNAL)
    {
        if (digitalRead(default_rdy_pin) == LOW)
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                SPI_buffer[i] = SPI.transfer(SPI_READ);
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            SPI_buffer[i] = SPI.transfer(SPI_READ);
        }
    }

    // delayMicroseconds(100);

    digitalWrite(default_cs_pin, HIGH);
    bit24 = SPI_buffer[0];
    bit24 = (bit24 << 8) | SPI_buffer[1];
    bit24 = (bit24 << 8) | SPI_buffer[2];

    bit24 = (bit24 << 8);
    result_32bit = (bit24 >> 8);
    PowerDown(); // Go to sleep

    return result_32bit;
}

int32_t ADS1220::ReadSingleShotChanel(int mux_chanel) // Read ADC converting value if you want set Input multiplexer configuration in method
{
    MuxChanel(mux_chanel); // select chanels to convert

    static byte SPI_buffer[3];
    int32_t result_32bit = 0;
    long int bit24;

    Start();

    // delayMicroseconds(100);

    if (interrupt == INTERNAL)
    {
        if (digitalRead(default_rdy_pin) == LOW)
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                SPI_buffer[i] = SPI.transfer(SPI_READ);
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            SPI_buffer[i] = SPI.transfer(SPI_READ);
        }
    }

    // delayMicroseconds(100);

    digitalWrite(default_cs_pin, HIGH);
    bit24 = SPI_buffer[0];
    bit24 = (bit24 << 8) | SPI_buffer[1];
    bit24 = (bit24 << 8) | SPI_buffer[2];

    bit24 = (bit24 << 8);
    result_32bit = (bit24 >> 8);
    PowerDown(); // Go to sleep

    return result_32bit;
}

float ADS1220::ConvertToMilivolt(int32_t ads_value)
{
    return (float)((ads_value * vfsr * 1000) / FULL_SCALE);
}

// SPI commands
void ADS1220::Start(void)
{
    digitalWrite(default_cs_pin, LOW);
    SPI.transfer(START);
    digitalWrite(default_cs_pin, HIGH);
    delayMicroseconds(50);
}

void ADS1220::Reset(void)
{
    digitalWrite(default_cs_pin, LOW);
    SPI.transfer(RESET);
    digitalWrite(default_cs_pin, HIGH);
    delayMicroseconds(100);
}

void ADS1220::PowerDown(void)
{
    digitalWrite(default_cs_pin, LOW);
    SPI.transfer(POWERDOWN);
    digitalWrite(default_cs_pin, HIGH);
    delayMicroseconds(50);
}
