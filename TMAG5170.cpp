#include "TMAG5170.h"

#include <SPI.h>

TMAG5170::TMAG5170(uint16_t SPI_CSpin) {
  _SPI_CS = SPI_CSpin;
  pinMode(_SPI_CS, OUTPUT);
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  readWriteFrame();  // write initial frame to disable CRC
  outBuf[3] = 0x00;
}

void TMAG5170::readWriteFrame(void) {
  digitalWrite(_SPI_CS, LOW);
  for (int i = 0; i < 4; i++) {  // transfer 4 bytes
    inBuf[i] = SPI.transfer(outBuf[i]);
  }
  digitalWrite(_SPI_CS, HIGH);

  return;
}

void TMAG5170::generateFrame(uint8_t read, uint8_t offset,
                             uint16_t regContent) {
  outBuf[0] = (read << 7) | offset;
  outBuf[1] = (regContent & 0xff00) >> 8;
  outBuf[2] = regContent & 0x00ff;
  outBuf[3] = 0x00;

  return;
}

void TMAG5170::setAverage(uint16_t convAvg) {
  readReg(DEVICE_CONFIG);
  _inReg &= ~CONV_AVG_MASK;
  _inReg |= convAvg;
  generateFrame(0, DEVICE_CONFIG, _inReg);
  readWriteFrame();

  return;
}

void TMAG5170::setOperatingMode(uint16_t operatingMode) {
  readReg(DEVICE_CONFIG);
  _inReg &= ~OPERATING_MODE_MASK;
  _inReg |= operatingMode;
  generateFrame(0, DEVICE_CONFIG, _inReg);
  readWriteFrame();

  return;
}

void TMAG5170::setTriggerMode(uint16_t triggerMode) {
  readReg(SYSTEM_CONFIG);
  _inReg &= ~TRIGGER_MODE_MASK;  // clear TRIGGER_MODE bits
  _inReg |= triggerMode;
  generateFrame(0, SYSTEM_CONFIG, _inReg);
  readWriteFrame();
  return;
}

void TMAG5170::enableChannels(bool enableX, bool enableY, bool enableZ) {
  // form MAG_CH_EN bits from individual boolean value
  uint16_t MAG_CH_EN = 0;
  MAG_CH_EN |= (enableZ << 2) | (enableY << 1) | enableX;
  MAG_CH_EN <<= 6;

  readReg(SENSOR_CONFIG);
  _inReg &= ~MAG_CH_EN_MASK;  // clear MAG_CH_EN bits
  _inReg |= MAG_CH_EN;        // configure MAG_CH_EN bits
  generateFrame(0, SENSOR_CONFIG, _inReg);
  readWriteFrame();

  return;
}

void TMAG5170::setRange(uint8_t rangeX, uint8_t rangeY, uint8_t rangeZ) {
  _magneticRange[0] = rangeX;
  _magneticRange[1] = rangeY;
  _magneticRange[2] = rangeZ;
  // form RANGE bits from individual value
  uint16_t RANGE = 0;
  RANGE |= ((rangeZ) << 4) | (rangeY << 2) | rangeX;

  readReg(SENSOR_CONFIG);
  _inReg &= ~(Z_RANGE_MASK | Y_RANGE_MASK | X_RANGE_MASK);
  // clear RANGE bits
  _inReg |= RANGE;  // configure RANGE bits
  generateFrame(0, SENSOR_CONFIG, _inReg);
  readWriteFrame();

  return;
}

uint16_t TMAG5170::readReg(uint8_t offset) {
  generateFrame(1, offset, 0);
  // set the first bit to 1 for reading
  readWriteFrame();
  _inReg = (uint16_t)inBuf[1] << 8 | (uint16_t)inBuf[2];

  return _inReg;
}

int16_t TMAG5170::getBRaw(uint8_t axis) {
  readReg(axis);
  return _inReg;
}

float TMAG5170::getB(uint8_t axis) {
  readReg(axis);  // read channel result register

  int16_t raw;
  raw = (int16_t)inBuf[1] << 8 | (int16_t)inBuf[2];
  float result;
  result = raw / 65536.f * 2;  // convert into ratio

  int range;
  switch (axis) {
    case AXIS_X:
      range = _magneticRange[0];
      break;
    case AXIS_Y:
      range = _magneticRange[1];
      break;
    case AXIS_Z:
      range = _magneticRange[2];
      break;
  }
  switch (range) {
    case RANGE_150mT:
      result *= 150;
      break;
    case RANGE_75mT:
      result *= 75;
      break;
    case RANGE_300mT:
      result *= 300;
      break;
  }
  return result;
}