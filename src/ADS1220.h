#include Arduino.h
#include "SPI.h"

#define SPI_READ   0xFF
#define RESET      0x06   
#define START      0x08    
#define WREG       0x40
#define RREG       0x20

#define config_address_reg0 0x00
#define config_address_reg1 0x01
#define config_address_reg2 0x02
#define config_address_reg3 0x03

#define default_value_reg0 0x00
#define default_value_reg1 0x04
#define default_value_reg2 0x10
#define default_value_reg3 0x00

class ADS1220
{
private:

        uint8_t default_cs_pin  = 7;
        uint8_t default_rdy_pin = 6;
public:

        ADS1220();
        void begin (void);
        void begin (uint8_t cs_pin, uint8_t rdy_pin);
        void ReadConfig (uint8_t address);
        void SetDefaultSettings (void);
        void ADS1220_START (void);
        void ADS1220_RESET (void);

}