// qmc5883p.h
#pragma once
#include <Arduino.h>
#include <Wire.h>

class QMC5883P {
public:
    // Konstruktor: optional andere SDA/SCL-Pins & I²C-Adresse
    QMC5883P(uint8_t addr = 0x2C, TwoWire &bus = Wire);

    bool begin();                              // Init, true = OK
    bool readXYZ(float *xyz);                  // xyz[3] → µT, true = neue Daten und kalibriert
    float getHeadingDeg(float declDeg = 0.0f); // Heading-Berechnung mit internem Daten-Caching
    void setHardIronOffsets(float xOff, float yOff, float zOff = 0.0f);
    void setSoftIronScales(float scaleX, float scaleY, float scaleZ = 1.0f);

private:
    uint8_t _addr;
    TwoWire *_bus;

    // Kalibrierungsparameter
    float _offX, _offY, _offZ;
    float _scaleX, _scaleY, _scaleZ;

    // Zwischenspeicher für rohe Messwerte und Zeit
    int16_t _lastRawX, _lastRawY, _lastRawZ;
    unsigned long _lastReadTime;
    static const unsigned long _minInterval = 5; // minimale Zeitspanne zwischen Reads in ms

    bool readReg(uint8_t reg, uint8_t *buf, uint8_t len);
    bool writeReg(uint8_t reg, uint8_t val);
    bool readRaw(); // liest rohe Daten, aktualisiert Cache nur wenn DRDY gesetzt und genug Zeit vergangen
};
