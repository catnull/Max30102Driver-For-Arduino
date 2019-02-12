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

#include <Wire.h>

#include "MAX30102.h"

MAX30102::MAX30102()
{
}

bool MAX30102::begin()
{
    Wire.begin();
    Wire.setClock(I2C_BUS_SPEED);

    if (getPartId() != EXPECTED_PART_ID) {
        return false;
    }

    setMode(DEFAULT_MODE);
    setLedsPulseWidth(DEFAULT_PULSE_WIDTH);
    setSamplingRate(DEFAULT_SAMPLING_RATE);
    setLedsCurrent(DEFAULT_IR_LED_CURRENT, DEFAULT_RED_LED_CURRENT);
	setAdcRange(DEFAULT_ADC_RANGE);
	setSamplingAverage(DEFAULT_SAMPLING_AVERAGE);
	setFIFORolloverEnable(true);
    return true;
}

void MAX30102::setMode(Mode mode)
{
    writeRegister(MAX30102_REG_MODE_CONFIGURATION, mode);
	setMultiLedSlot(mode);
}

void MAX30102::setLedsPulseWidth(LEDPulseWidth ledPulseWidth)
{
    uint8_t previous = readRegister(MAX30102_REG_SPO2_CONFIGURATION);
    writeRegister(MAX30102_REG_SPO2_CONFIGURATION, (previous & 0xfc) | ledPulseWidth);
}

void MAX30102::setSamplingRate(SamplingRate samplingRate)
{
    uint8_t previous = readRegister(MAX30102_REG_SPO2_CONFIGURATION);
    writeRegister(MAX30102_REG_SPO2_CONFIGURATION, (previous & 0xe3) | (samplingRate << 2));
}

void MAX30102::setLedsCurrent(LEDCurrent irLedCurrent, LEDCurrent redLedCurrent)
{
    writeRegister(MAX30102_REG_IR_PULSE_AMPLITUDE, irLedCurrent);
	writeRegister(MAX30102_REG_RED_PULSE_AMPLITUDE, redLedCurrent);
}

void MAX30102::setIrPulseAmplitude(uint8_t irLedCurrent)
{
	writeRegister(MAX30102_REG_IR_PULSE_AMPLITUDE, irLedCurrent);
}

void MAX30102::setRedPulseAmplitude(uint8_t redLedCurrent)
{
	writeRegister(MAX30102_REG_RED_PULSE_AMPLITUDE, redLedCurrent);
}

void MAX30102::setPilotPulseAmplitude(uint8_t pilotCurrent)
{
	writeRegister(MAX30102_REG_PILOT_PULSE_AMPLITUDE, pilotCurrent);
}

void MAX30102::setMultiLedSlot(Mode mode)
{
	writeRegister(MAX30102_REG_MULTI_LED_CONTROL_0, MAX30102_SLOT_RED_ACTIVE );
	if(mode > MAX30102_MODE_HRONLY){
		writeRegister(MAX30102_REG_MULTI_LED_CONTROL_0, MAX30102_SLOT_IR_ACTIVE << 4 );
	}
}

void MAX30102::setAdcRange(AdcRange adcRange)
{
    uint8_t previous = readRegister(MAX30102_REG_SPO2_CONFIGURATION);
    writeRegister(MAX30102_REG_SPO2_CONFIGURATION, (previous & 0x1f) | (adcRange << 5));
}

void MAX30102::setSamplingAverage(SamplingAverage samplingAverage)
{
    uint8_t previous = readRegister(MAX30102_REG_FIFO_CONFIGURATION);
    writeRegister(MAX30102_REG_FIFO_CONFIGURATION, (previous & 0x1f) | (samplingAverage << 5));
}

void MAX30102::setFIFORolloverEnable(bool enabled)
{
    uint8_t previous = readRegister(MAX30102_REG_FIFO_CONFIGURATION);
	if(enabled) {
		writeRegister(MAX30102_REG_FIFO_CONFIGURATION, previous | MAX30102_FC_ENB_ROLLOVER);
	}else {
		writeRegister(MAX30102_REG_FIFO_CONFIGURATION, previous & ~MAX30102_FC_ENB_ROLLOVER);
	}
}

void MAX30102::update()
{
    readFifoData();
}

bool MAX30102::getRawValues(uint32_t *red, uint32_t *ir)
{
    if (!readoutsBuffer.isEmpty()) {
        SensorReadout readout = readoutsBuffer.pop();

        *ir = readout.ir;
        *red = readout.red;

        return true;
    } else {
        return false;
    }
}

void MAX30102::resetFifo()
{
    writeRegister(MAX30102_REG_FIFO_WRITE_POINTER, 0);
    writeRegister(MAX30102_REG_FIFO_READ_POINTER, 0);
    writeRegister(MAX30102_REG_FIFO_OVERFLOW_COUNTER, 0);
}

uint8_t MAX30102::readRegister(uint8_t address)
{
    Wire.beginTransmission(MAX30102_I2C_ADDRESS);
    Wire.write(address);
    Wire.endTransmission(false);
    Wire.requestFrom(MAX30102_I2C_ADDRESS, 1);

    return Wire.read();
}

void MAX30102::writeRegister(uint8_t address, uint8_t data)
{
    Wire.beginTransmission(MAX30102_I2C_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

void MAX30102::burstRead(uint8_t baseAddress, uint8_t *buffer, uint8_t length)
{
    Wire.beginTransmission(MAX30102_I2C_ADDRESS);
    Wire.write(baseAddress);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MAX30102_I2C_ADDRESS, length);

    uint8_t idx = 0;
    while (Wire.available()) {
        buffer[idx++] = Wire.read();
    }
}

void MAX30102::readFifoData()
{
    uint8_t buffer[MAX30102_FIFO_DEPTH*6];
    uint8_t toRead;

    toRead = (readRegister(MAX30102_REG_FIFO_WRITE_POINTER) - readRegister(MAX30102_REG_FIFO_READ_POINTER)) & (MAX30102_FIFO_DEPTH-1);

    if (toRead) {
        burstRead(MAX30102_REG_FIFO_DATA, buffer, 6 * toRead);

        for (uint8_t i=0 ; i < toRead ; ++i) {
            // Warning: the values are always left-aligned
            readoutsBuffer.push({
					.red=(uint32_t)(((buffer[i*6]&0x03) << 16) | (buffer[i*6 + 1] << 8) | buffer[i*6 + 2] ),
					.ir=(uint32_t)(((buffer[i*6 + 3]&0x03) << 16) | (buffer[i*6 + 4] << 8) | buffer[i*6 + 5])					
					});
        }
    }
}

void MAX30102::startTemperatureSampling()
{
    uint8_t modeConfig = readRegister(MAX30102_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30102_MC_TEMP_EN;

    writeRegister(MAX30102_REG_MODE_CONFIGURATION, modeConfig);
}

bool MAX30102::isTemperatureReady()
{
    return !(readRegister(MAX30102_REG_MODE_CONFIGURATION) & MAX30102_MC_TEMP_EN);
}

float MAX30102::retrieveTemperature()
{
    int8_t tempInteger = readRegister(MAX30102_REG_TEMPERATURE_DATA_INT);
    float tempFrac = readRegister(MAX30102_REG_TEMPERATURE_DATA_FRAC);

    return tempFrac * 0.0625 + tempInteger;
}

void MAX30102::shutdown()
{
    uint8_t modeConfig = readRegister(MAX30102_REG_MODE_CONFIGURATION);
    modeConfig |= MAX30102_MC_SHDN;

    writeRegister(MAX30102_REG_MODE_CONFIGURATION, modeConfig);
}

void MAX30102::resume()
{
    uint8_t modeConfig = readRegister(MAX30102_REG_MODE_CONFIGURATION);
    modeConfig &= ~MAX30102_MC_SHDN;

    writeRegister(MAX30102_REG_MODE_CONFIGURATION, modeConfig);
}

uint8_t MAX30102::getPartId()
{
    return readRegister(0xff);
}
