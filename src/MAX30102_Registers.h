/*
Arduino-MAX30102 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAX30102_REGISTERS_H
#define MAX30102_REGISTERS_H

#define MAX30102_I2C_ADDRESS                    0x57

// Interrupt status 1 register (RO)
#define MAX30102_REG_INTERRUPT_STATUS_1           0x00
#define MAX30102_IS_PWR_RDY                     (1 << 0)
#define MAX30102_IS_PROX_INT                    (1 << 4)
#define MAX30102_IS_ALC_OVF                     (1 << 5)
#define MAX30102_IS_PPG_RDY                     (1 << 6)
#define MAX30102_IS_A_FULL                      (1 << 7)

// Interrupt status 2 register (RO)
#define MAX30102_REG_INTERRUPT_STATUS_2           0x01
#define MAX30102_IS_DIE_TEMP_RDY                (1 << 1)

// Interrupt enable 1 register
#define MAX30102_REG_INTERRUPT_ENABLE_1           0x02
#define MAX30102_IE_ENB_PROX_INT_1              (1 << 4)
#define MAX30102_IE_ENB_ALC_OVF                 (1 << 5)
#define MAX30102_IE_ENB_PPG_RDY                 (1 << 6)
#define MAX30102_IE_ENB_A_FULL                  (1 << 7)

// Interrupt enable 2 register
#define MAX30102_REG_INTERRUPT_ENABLE_2           0x03
#define MAX30102_IE_ENB_PROX_INT_2                (1 << 1)

// FIFO control and data registers
#define MAX30102_REG_FIFO_WRITE_POINTER         0x04
#define MAX30102_REG_FIFO_OVERFLOW_COUNTER      0x05
#define MAX30102_REG_FIFO_READ_POINTER          0x06
#define MAX30102_REG_FIFO_DATA                  0x07  // Burst read does not autoincrement addr

//FIFO Configuration register
#define MAX30102_REG_FIFO_CONFIGURATION			0x08

typedef enum SamplingAverage{
	MAX30102_SAMP_AVE_1			=0x00,
	MAX30102_SAMP_AVE_2			=0x01,
	MAX30102_SAMP_AVE_4			=0x02,
	MAX30102_SAMP_AVE_8			=0x03,
	MAX30102_SAMP_AVE_16		=0x04,
	MAX30102_SAMP_AVE_32		=0x05,
	MAX30102_SAMP_AVE_32_A		=0x06,
	MAX30102_SAMP_AVE_32_B		=0x07	
} SamplingAverage;   						//should move 5 bits left as ( [smp_ave] << 5 )

#define MAX30102_FC_ENB_ROLLOVER			(1 << 4)

typedef enum FifoEmptySamples{
	MAX30102_EMP_SAMP_0  	=0x00,
	MAX30102_EMP_SAMP_1  	=0x01,
	MAX30102_EMP_SAMP_2  	=0x02,
	MAX30102_EMP_SAMP_3  	=0x03,
	MAX30102_EMP_SAMP_4  	=0x04,
	MAX30102_EMP_SAMP_5  	=0x05,
	MAX30102_EMP_SAMP_6  	=0x06,
	MAX30102_EMP_SAMP_7  	=0x07,
	MAX30102_EMP_SAMP_8  	=0x08,
	MAX30102_EMP_SAMP_9  	=0x09,
	MAX30102_EMP_SAMP_10  	=0x0a,
	MAX30102_EMP_SAMP_11 	=0x0b,
	MAX30102_EMP_SAMP_12  	=0x0c,
	MAX30102_EMP_SAMP_13  	=0x0d,
	MAX30102_EMP_SAMP_14 	=0x0e,
	MAX30102_EMP_SAMP_15 	=0x0f
} FifoEmptySamples;


// Mode Configuration register
#define MAX30102_REG_MODE_CONFIGURATION         0x09
#define MAX30102_MC_TEMP_EN                    (1 << 3)
#define MAX30102_MC_RESET                      (1 << 6)
#define MAX30102_MC_SHDN                       (1 << 7)
typedef enum Mode {
    MAX30102_MODE_HRONLY    = 0x02,
    MAX30102_MODE_SPO2_HR   = 0x03,
	MAX30102_MODE_MULT_LED  = 0x07
} Mode;

// SpO2 Configuration register
// Check tables 8 and 9, p19 of the MAX30102 datasheet to see the permissible
// combinations of sampling rates and pulse widths
#define MAX30102_REG_SPO2_CONFIGURATION         0x0a

typedef enum AdcRange {
	MAX30102_ADC_RANGE_2048NA 	= 0x00,
	MAX30102_ADC_RANGE_4096NA 	= 0x01,
	MAX30102_ADC_RANGE_8192NA 	= 0x02,
	MAX30102_ADC_RANGE_16384NA 	= 0x03,
} AdcRange;

typedef enum SamplingRate {
    MAX30102_SAMPRATE_50HZ      = 0x00,
	MAX30102_SAMPRATE_100HZ     = 0x01,
	MAX30102_SAMPRATE_200HZ     = 0x02,
	MAX30102_SAMPRATE_400HZ     = 0x03,
	MAX30102_SAMPRATE_800HZ     = 0x04,
	MAX30102_SAMPRATE_1000HZ    = 0x05,
	MAX30102_SAMPRATE_1600HZ    = 0x06,
	MAX30102_SAMPRATE_3200HZ    = 0x07
} SamplingRate;

typedef enum LEDPulseWidth {
    MAX30102_SPC_PW_69US_15BITS    = 0x00,
    MAX30102_SPC_PW_118US_16BITS   = 0x01,
    MAX30102_SPC_PW_215US_17BITS   = 0x02,
    MAX30102_SPC_PW_411US_18BITS   = 0x03
} LEDPulseWidth;

// LED Configuration register
#define MAX30102_REG_RED_PULSE_AMPLITUDE        0x0c
#define MAX30102_REG_IR_PULSE_AMPLITUDE         0x0d
#define MAX30102_REG_PILOT_PULSE_AMPLITUDE      0x10
typedef enum LEDCurrent {
	MAX30102_LED_CURR_0MA      = 0,
	MAX30102_LED_CURR_4_4MA    = 23,
	MAX30102_LED_CURR_7_6MA    = 39,
	MAX30102_LED_CURR_11MA     = 56,
	MAX30102_LED_CURR_14_2MA   = 73,
	MAX30102_LED_CURR_17_4MA   = 89,
	MAX30102_LED_CURR_20_8MA   = 106,
	MAX30102_LED_CURR_24MA     = 123,
	MAX30102_LED_CURR_27_1MA   = 139,
	MAX30102_LED_CURR_30_6MA   = 157,
	MAX30102_LED_CURR_33_8MA   = 173,
	MAX30102_LED_CURR_37MA     = 189,
	MAX30102_LED_CURR_40_2MA   = 206,
	MAX30102_LED_CURR_43_6MA   = 223,
	MAX30102_LED_CURR_46_8MA   = 240,
	MAX30102_LED_CURR_50MA     = 255
} LEDCurrent;

//Multi-LED Mode Control registers
#define MAX30102_REG_MULTI_LED_CONTROL_0		0x11
#define MAX30102_REG_MULTI_LED_CONTROL_1		0x12
typedef enum SlotActive {
	MAX30102_SLOT_OFF 			= 0x00,
	MAX30102_SLOT_RED_ACTIVE	= 0x01,
	MAX30102_SLOT_IR_ACTIVE		= 0x02,
	MAX30102_SLOT_RED_PILOT		= 0x05,
	MAX30102_SLOT_IR_PILOT		= 0x06
	

} SlotActive;

		
// Temperature integer part register
#define MAX30102_REG_TEMPERATURE_DATA_INT       0x1f
// Temperature fractional part register
#define MAX30102_REG_TEMPERATURE_DATA_FRAC      0x20

// Temperature config register					
#define MAX30102_REG_TEMPERATURE_CONFIG      	0x21
#define MAX30102_TC_ENB_TEMP					(1<< 0)

//Proximity Mode interrupt threshold
#define MAX30102_REG_PROX_MODE_INT_THRES		0x30
	

// Revision ID register (RO)
#define MAX30102_REG_REVISION_ID                0xfe
// Part ID register
#define MAX30102_REG_PART_ID                    0xff

#define MAX30102_FIFO_DEPTH                     0x20

#endif
