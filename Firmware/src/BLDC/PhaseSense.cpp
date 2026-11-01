#include "PhaseSense.h"

#include <Arduino.h>

PhaseSense::PhaseSense(int senseA, int senseB, int senseC, int senseSupply) :
    _phasePin{senseA, senseB, senseC},
    _supplyPin(senseSupply)
{}

void PhaseSense::setup()
{
    // 11 dB attenuation -> ~0..3.1 V input range
    for (int pin : _phasePin)
    {
        analogSetPinAttenuation(pin, ADC_11db);
    }
    analogSetPinAttenuation(_supplyPin, ADC_11db);
}

float PhaseSense::readPhase(DRV8302::Phase phase) const
{
    return readDivided(_phasePin[static_cast<int>(phase)]);
}

float PhaseSense::readSupply() const
{
    return readDivided(_supplyPin);
}

float PhaseSense::readDivided(int pin)
{
    // analogReadMilliVolts uses the eFuse ADC calibration
    return (analogReadMilliVolts(pin) / 1000.0f) * DIVIDER_RATIO;
}
