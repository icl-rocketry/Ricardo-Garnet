#pragma once

#include <cstdint>

#include "DRV8302.h"
#include "PhaseSense.h"

/*
 * Slow single-phase bring-up test for the DRV8302.
 *
 * Cycles phase A: OFF -> LOW -> HIGH -> OFF ... spending StepTime_ms in each,
 * so you can watch SH_A (connector P9 / "MOTOR3") on a scope or multimeter:
 *   OFF  -> floating (drifts toward 0 V through the sense divider)
 *   LOW  -> ~0 V
 *   HIGH -> ~V_SUPPLY
 * LOW comes before HIGH so the bootstrap cap is charged before the high side turns on.
 *
 * Phases B and C are held off. If FAULT goes low, everything is switched off
 * and the test stops.
 *
 * With a motor connected, B and C follow A through the windings (no return
 * path, so no current flows): all three read ~V_SUPPLY in HIGH and ~0 V otherwise.
 *
 * Phase + supply voltages are printed to Serial every PrintTime_ms.
 */
class BLDCPhaseTest
{
public:
    static constexpr uint32_t StepTime_ms = 2000;
    static constexpr uint32_t PrintTime_ms = 250;
    static constexpr DRV8302::Phase TestPhase = DRV8302::Phase::A;

    BLDCPhaseTest();

    void setup();
    void update(); // non-blocking, call from the main loop

private:
    enum class Step : uint8_t { Off, Low, High };

    DRV8302 _drv;
    PhaseSense _sense;
    Step _step = Step::Off;
    uint32_t _lastStepTime = 0;
    uint32_t _lastPrintTime = 0;
    bool _stopped = false;

    void applyStep();
    void printVoltages();
    const char* stepName() const;
};
