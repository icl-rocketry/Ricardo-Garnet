#pragma once
/*
 * Phase voltage sense via the 39k / 2k2 dividers on each motor phase.
 *
 * From the Ricardo-Garnet schematic (note the numbering is reversed):
 *   SENS1 (GPIO1) -> SH_C (phase C)
 *   SENS2 (GPIO2) -> SH_B (phase B)
 *   SENS3 (GPIO5) -> SH_A (phase A)
 *   AN_IN (GPIO13, ADC2) -> V_SUPPLY, same divider
 *
 * Divider ratio (39k + 2k2) / 2k2 = 18.73, so ~3.1 V ADC full scale ~= 58 V.
 */

#include "DRV8302.h"

class PhaseSense
{
public:
    static constexpr float R_TOP = 39000.0f;
    static constexpr float R_BOTTOM = 2200.0f;
    static constexpr float DIVIDER_RATIO = (R_TOP + R_BOTTOM) / R_BOTTOM;

    PhaseSense(int senseA, int senseB, int senseC, int senseSupply);

    void setup();

    // volts at the phase node / supply, after undoing the divider
    float readPhase(DRV8302::Phase phase) const;
    float readSupply() const;

private:
    const int _phasePin[3];
    const int _supplyPin;

    static float readDivided(int pin);
};
