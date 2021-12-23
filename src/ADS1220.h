#include <Arduino.h>
#include "SPI.h"

//SPI commands
#define SPI_READ            0xFF        //SPI command for read data after transmitter 
#define RESET               0x06        //Reset the device    
#define START               0x08        //Start or restart conversions 
#define POWERDOWN           0x02        //Enter power-down mode
#define RDATA               0X10        //Read data by command     
#define WREG                0x40        //Write nn registers starting at address rr
#define RREG                0x20        //Read nn registers starting at address rr

//register address and default settings
#define config_address_reg0 0x00
#define config_address_reg1 0x01
#define config_address_reg2 0x02
#define config_address_reg3 0x03

#define default_value_reg0  0x00
#define default_value_reg1  0x04
#define default_value_reg2  0x10
#define default_value_reg3  0x00

//Settings Masks and parameters

/* Register 0
 _____ ___________________________________________________________________
|bytes| 7      6       5       4   |    3       2       1    |     0      |
 -----|--------------------------------------------------------------------
|param|          MUX[3:0]          |         GAIN[2:0]       | PGA_BYPASS |
 -------------------------------------------------------------------------
*/
#define MASK_PGA_BYPASS     0x01
#define MASK_GAIN           0x0E
#define MASK_MUX            0xF0

//      Disables and bypasses the internal low-noise PGA   
//              The PGA can only be disabled for gains 1, 2, and 4.
//              The PGA is always enabled for gain settings 8 to 128     
#define PGA_BYPASS_ON       0x00        //PGA enabled (default)
#define PGA_BYPASS_OFF      0x01        //PGA disabled and bypassed

//      Gain configuration
#define GAIN_1              0x00        // FSR ±2.048 V (default)
#define GAIN_2              0x02        // FSR ±1.024 V
#define GAIN_4              0x04        // FSR ±0.512 V
#define GAIN_8              0x06        // FSR ±0.256 V
#define GAIN_16             0x08        // FSR ±0.128 V
#define GAIN_32             0x0A        // FSR ±0.064 V
#define GAIN_64             0x0C        // FSR ±0.032 V
#define GAIN_128            0x0E        // FSR ±0.016 V

//      Input multiplexer configuration
#define MUX_AIN0_AIN1       0x00        //AINP = AIN0, AINN = AIN1 (default)
#define MUX_AIN0_AIN2       0x10        //AINP = AIN0, AINN = AIN2
#define MUX_AIN0_AIN3       0x20        //AINP = AIN0, AINN = AIN3
#define MUX_AIN1_AIN2       0x30        //AINP = AIN1, AINN = AIN2
#define MUX_AIN1_AIN3       0x40        //AINP = AIN1, AINN = AIN3
#define MUX_AIN2_AIN3       0x50        //AINP = AIN2, AINN = AIN3
#define MUX_AIN1_AIN0       0x60        //AINP = AIN1, AINN = AIN0
#define MUX_AIN3_AIN2       0x70        //AINP = AIN3, AINN = AIN2
#define MUX_AIN0_AVSS       0x80        //AINP = AIN0, AINN = AVSS
#define MUX_AIN1_AVSS       0x90        //AINP = AIN1, AINN = AVSS
#define MUX_AIN2_AVSS       0xA0        //AINP = AIN2, AINN = AVSS
#define MUX_AIN3_AVSS       0xB0        //AINP = AIN3, AINN = AVSS




 /* Register 1
   _____ ___________________________________________________________________
  |bytes| 7      6       5    |   4       3   |    2    |   1    |     0    |
   -----|-------------------------------------------------------------------
  |param|      DR[2:0]        |   MODE[1:0]   |    CM   |   TS   |    BCS   |
   -------------------------------------------------------------------------
*/
#define MASK_BCS            0x01
#define MASK_TS             0x02
#define MASK_CM             0x04
#define MASK_MODE           0x18
#define MASK_DR             0xE0

//      Burn-out current sources
//                This bit controls the 10-µA, burn-out current sources.
//                The burn-out current sources can be used to detect sensor faults such as wire
//                breaks and shorted sensors.
#define BCS_OFF             0x00        //Current sources off (default)
#define BCS_ON              0x01        //Current sources on

//      Temperature sensor mode
//              This bit enables the internal temperature sensor and puts the device in
//              temperature sensor mode.
#define TS_OFF              0x00        //Disables temperature sensor (default)
#define TS_ON               0x02        //Enables temperature sensor

//      Conversion mode
//              This bit sets the conversion mode for the device
#define SINGLE_SHORT_MODE   0x00        //Single-shot mode (default)
#define CONTINUOUS_MODE     0x04        //Continuous conversion mode

//      Operating mode
//              These bits control the operating mode the device operates in.
#define NORMAL_MODE         0x00        //256-kHz modulator clock      default
#define DUTY_CYCLE_MODE     0x08        //internal duty cycle of 1:4
#define TURBO_MODE          0x10        //512-kHz modulator clock

//      Data rate
//              These bits control the data rate setting depending on the selected operating
//              mode for NORMAL MODE
#define DR_20SPS            0x00
#define DR_45SPS            0x20
#define DR_90SPS            0x40
#define DR_175SPS           0x60
#define DR_330SPS           0x80
#define DR_600SPS           0xA0
#define DR_1000SPS          0xC0

/*                      Table  DR Bit Settings
    NORMAL MODE             DUTY-CYCLE MODE                     TURBO MODE
   000 = 20 SPS              000 = 5 SPS                     000 = 40 SPS
   001 = 45 SPS              001 = 11.25 SPS                 001 = 90 SPS
   010 = 90 SPS              010 = 22.5 SPS                  010 = 180 SPS
   011 = 175 SPS             011 = 44 SPS                    011 = 350 SPS
   100 = 330 SPS             100 = 82.5 SPS                  100 = 660 SPS
   101 = 600 SPS             101 = 150 SPS                   101 = 1200 SPS
   110 = 1000 SPS            110 = 250 SPS                   110 = 2000 SPS
   111 = Reserved            111 = Reserved                  111 = Reserved                     
*/




/* Register 2
  _____ ___________________________________________________________________
 |bytes| 7      6   |    5       4   |    3   |    2       1        0      |
  -----|-------------------------------------------------------------------
 |param| VREF[1:0]  |   50/60HZ[1:0] |   PSW  |         IDAC[2:0]          |
  -------------------------------------------------------------------------
*/
#define MASK_IDAC           0x07
#define MASK_PSW            0x08
#define MASK_FIR            0x30
#define MASK_VREF           0xC0

//      IDAC current setting
//              These bits set the current for both IDAC1 and IDAC2 excitation current sources.
#define IDAC_OFF            0x00        //default                
#define IDAC_10uA           0x01
#define IDAC_50uA           0x02
#define IDAC_100uA          0x03
#define IDAC_250uA          0x04
#define IDAC_500uA          0x05
#define IDAC_1000uA         0x06
#define IDAC_1500uA         0x07

//      Low-side power switch configuration
//              This bit configures the behavior of the low-side switch connected between
//              AIN3/REFN1 and AVSS.
#define PSW_OPEN            0x00        //Switch is always open (default)
#define PSW_AUTO            0x08        //Switch automatically closes when the START/SYNC command is sent and opens when the POWERDOWN command is issued

//      FIR filter configuration
//              These bits configure the filter coefficients for the internal FIR filter.
//              Only use these bits together with the 20-SPS setting in normal mode and the 5-
//              SPS setting in duty-cycle mode. Set to 00 for all other data rates.
#define FIR_OFF             0x00        //No 50-Hz or 60-Hz rejection (default)
#define FIR_50_60           0x10        //Simultaneous 50-Hz and 60-Hz rejection
#define FIR_50              0x20        //50-Hz rejection only
#define FIR_60              0x30        //60-Hz rejection only

//      Voltage reference selection
//              These bits select the voltage reference source that is used for the conversion
#define VREF_INTERNAL       0x00        //Internal 2.048-V reference selected (default)
#define VREF_REFP0_REFN0    0x40        //External reference selected using dedicated REFP0 and REFN0 inputs
#define VREF_AN0_AN3        0x80        //External reference selected using AIN0/REFP1 and AIN3/REFN1 inputs
#define VREF_ANALOG         0xC0        //Analog supply (AVDD – AVSS) used as reference




/*Register 3
   _____ _______________________________________________________________________
  |bytes| 7      6       5    |   4       3        2     |     1   |     0      |
   -----|-----------------------------------------------------------------------
  |param|     I1MUX[2:0]      |         I2MUX[2:0]       |  DRDYM  |  RESERVED  |
   -----------------------------------------------------------------------
*/
#define MASK_DRDYM          0x02
#define MASK_I2MUX          0x1C
#define MASK_I1MUX          0xE0

//      DRDY mode
//              This bit controls the behavior of the DOUT/DRDY pin when new data are ready.
#define DRDYM_DRDY_ONLY     0x00        //Only the dedicated DRDY pin is used to indicate when data are ready (default)
#define DRDYM_DRDY_DOUT     0x02        //Data ready is indicated simultaneously on DOUT/DRDY and DRDY

//      IDAC2 routing configuration
//              These bits select the channel where IDAC2 is routed to.
#define IDAC2_OFF           0x00        //IDAC2 disabled (default)
#define IDAC2_AIN0_REFP1    0x04        //IDAC2 connected to AIN0/REFP1
#define IDAC2_AIN1          0x08        //IDAC2 connected to AIN1
#define IDAC2_AIN2          0x0C        //IDAC2 connected to AIN2
#define IDAC2_AIN3_REFN1    0x10        //IDAC2 connected to AIN3/REFN1
#define IDAC2_REFP0         0x14        //IDAC2 connected to REFP0
#define IDAC2_REFN0         0x18        //IDAC2 connected to REFN0  
          
//      IDAC1 routing configuration
//              These bits select the channel where IDAC1 is routed to.
#define IDAC1_OFF           0x00        //IDAC1 disabled (default)
#define IDAC1_AIN0_REFP1    0x20        //IDAC1 connected to AIN0/REFP1
#define IDAC1_AIN1          0x40        //IDAC1 connected to AIN1
#define IDAC1_AIN2          0x60        //IDAC1 connected to AIN2
#define IDAC1_AIN3_REFN1    0x80        //IDAC1 connected to AIN3/REFN1
#define IDAC1_REFP0         0xA0        //IDAC1 connected to REFP0
#define IDAC1_REFN0         0xC0        //IDAC1 connected to REFN0  



class ADS1220
{
private:
        uint8_t config_register0_value;
        uint8_t config_register1_value;
        uint8_t config_register2_value;
        uint8_t config_register3_value;

        uint8_t default_cs_pin  = 7;
        uint8_t default_rdy_pin = 6;
        void WriteConfig(uint8_t address, uint8_t value);
public:
        ADS1220();
        void begin (void);
        void begin (uint8_t cs_pin, uint8_t rdy_pin);
        uint8_t ReadConfig (uint8_t address);
        void SetDefaultSettings (void);
        void SetSettings (uint8_t address, uint8_t value);

        // Register 0 configuration metods 
        void PGA (int pga_mode);
        void Gain (int gain);       
        void MuxChanel (int mux_chanel);

        // Register 1 configuration metods
        void BCS (int bcs_mode);
        void TemperatureSensor (int ts_mode );
        void ConversionMode (int conversion_mode);
        void OperatingMode (int operating_mode);
        void DataRate (int data_rate_mode);

        // Register 2 configuration metods
        void IDAC (int idac_current);
        void PSW (int psw_mode);
        void FIR (int fit_mode);
        void VREF (int vref_mode);

        // Register 2 configuration metods
        void DRDYM (int vref_mode);
        void I2MUX (int i2mux_mode);
        void I1MUX (int i1mux_mode);

        // SPI commands
        void Start (void);
        void Reset (void);
        void PowerDown (void);

};