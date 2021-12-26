#include "ADS1220.h"

#define cs_pin 7
#define rdy_pin 8

ADS1220 ADC;

void setup()
{
    ADC.begin(cs_pin, rdy_pin);
    ADC.PGA(PGA_BYPASS_ON);
    ADC.Gain(GAIN_1);
    ADC.MuxChanel(MUX_AIN0_AIN1);
    ADC.TemperatureSensor(TS_ON);
    ADC.ConversionMode(CONTINUOUS_MODE);
    ADC.OperatingMode(DUTY_CYCLE_MODE);
    ADC.DataRate(DR_20SPS);
    ADC.FIR(FIR_50);
    ADC.GetRegistersValue();
}

void loop()
{

}