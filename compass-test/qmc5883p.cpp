// qmc5883p.cpp
#include "qmc5883p.h"
#include <Arduino.h>

/* Register-Adressen */
#define REG_CHIP_ID        0x00
#define REG_DATA_OUT_X_LSB 0x01
#define REG_STATUS         0x09
#define REG_CTL1           0x0A
#define REG_CTL2           0x0B

QMC5883P::QMC5883P(uint8_t addr, TwoWire &bus)
    : _addr(addr), _bus(&bus),
      _offX(0.0f), _offY(0.0f), _offZ(0.0f),
      _scaleX(1.0f), _scaleY(1.0f), _scaleZ(1.0f),
      _lastRawX(0), _lastRawY(0), _lastRawZ(0),
      _lastReadTime(0) {}

bool QMC5883P::begin() {
    _bus->begin(); // Pins vorher im Sketch setzen

    uint8_t id;
    if (!readReg(REG_CHIP_ID, &id, 1) || id != 0x80) return false;

    // Standard-Konfiguration: 200 Hz, Continuous, ±2G
    writeReg(0x0D, 0x40); delay(10);
    writeReg(0x29, 0x06); delay(10);
    writeReg(REG_CTL1, 0xCF); delay(10);
    writeReg(REG_CTL2, 0x00); delay(10);

    _lastReadTime = 0;
    return true;
}

bool QMC5883P::readRaw() {
    unsigned long now = millis();
    if (now - _lastReadTime < _minInterval) {
        // noch nicht genug Zeit vergangen, verwende Cache
        return false;
    }

    uint8_t status;
    if (!readReg(REG_STATUS, &status, 1) || !(status & 0x01)) {
        // kein neues Datum
        return false;
    }

    uint8_t buf[6];
    if (!readReg(REG_DATA_OUT_X_LSB, buf, 6)) return false;

    _lastRawX = int16_t(buf[1] << 8 | buf[0]);
    _lastRawY = int16_t(buf[3] << 8 | buf[2]);
    _lastRawZ = int16_t(buf[5] << 8 | buf[4]);
    _lastReadTime = now;
    return true;
}

bool QMC5883P::readXYZ(float *xyz) {
    if (!readRaw()) return false; // keine neuen Daten

    // Rohdaten → µT und kalibrieren (Hard- + Soft-Iron)
    float x = _lastRawX / 1000.0f;
    float y = _lastRawY / 1000.0f;
    float z = _lastRawZ / 1000.0f;

    xyz[0] = (x - _offX) * _scaleX;
    xyz[1] = (y - _offY) * _scaleY;
    xyz[2] = (z - _offZ) * _scaleZ;
    return true;
}

float QMC5883P::getHeadingDeg(float declDeg) {
    // versuche neue Daten zu lesen, verwende ansonsten letzten Cache
    readRaw();

    // gleiche Umrechnung wie in readXYZ, ohne Rückgabewert
    float x = (_lastRawX / 1000.0f - _offX) * _scaleX;
    float y = (_lastRawY / 1000.0f - _offY) * _scaleY;

    // 1) Grundwinkel (-π … π)
    float hdg = atan2(y, x);
    // 2) Deklination hinzufügen (deg → rad)
    hdg += declDeg * DEG_TO_RAD;
    // 3) Normieren auf 0 … 2π
    if (hdg < 0)        hdg += TWO_PI;
    else if (hdg > TWO_PI) hdg -= TWO_PI;
    // 4) in Grad
    return hdg * RAD_TO_DEG;
}

void QMC5883P::setHardIronOffsets(float xOff, float yOff, float zOff) {
    _offX = xOff;
    _offY = yOff;
    _offZ = zOff;
}

void QMC5883P::setSoftIronScales(float scaleX, float scaleY, float scaleZ) {
    _scaleX = scaleX;
    _scaleY = scaleY;
    _scaleZ = scaleZ;
}

bool QMC5883P::readReg(uint8_t reg, uint8_t *buf, uint8_t len) {
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    if (_bus->endTransmission(false) != 0) return false;
    if (_bus->requestFrom(_addr, len) != len) return false;
    for (uint8_t i = 0; i < len; i++) buf[i] = _bus->read();
    return true;
}

bool QMC5883P::writeReg(uint8_t reg, uint8_t val) {
    _bus->beginTransmission(_addr);
    _bus->write(reg);
    _bus->write(val);
    return (_bus->endTransmission() == 0);
}
