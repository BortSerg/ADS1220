#include "ADS1220.h"
#include <SPI.h>

#define cs_pin D8
#define rdy_pin D2

ADS1220 ADS;

void setup()
{
    Serial.begin(115200);
    Serial.println();
    delay(10);
    pinMode(cs_pin, OUTPUT);
    pinMode(rdy_pin, INPUT);

    ADS.begin(cs_pin, rdy_pin);
    ADS.PGA(PGA_BYPASS_ON);
    ADS.Gain(GAIN_1);
    ADS.MuxChanel(MUX_AIN0_AIN1);
    // ADS.TemperatureSensor(TS_ON);
    ADS.ConversionMode(CONTINUOUS_MODE);
    ADS.OperatingMode(DUTY_CYCLE_MODE);
    ADS.DataRate(DR_20SPS);
    ADS.FIR(FIR_50);
    ADS.GetRegistersValue();
}

void loop()
{
    Serial.println(ADS.ReadContinuous());
    delay(50);
}
