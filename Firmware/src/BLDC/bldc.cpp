#include "bldc.h"

#include <Arduino.h>

#include "Config/pinmap_config.h"

BLDCPhaseTest::BLDCPhaseTest() :
    _drv(PinMap::EnGate, PinMap::Fault, PinMap::DcCal,
         PinMap::H1, PinMap::L1,
         PinMap::H2, PinMap::L2,
         PinMap::H3, PinMap::L3),
    // SENS3 = phase A, SENS2 = phase B, SENS1 = phase C
    _sense(PinMap::Sens3, PinMap::Sens2, PinMap::Sens1, PinMap::AnIn)
{}

void BLDCPhaseTest::setup()
{
    _sense.setup();
    _drv.setup();
    _drv.enable();

    if (_drv.isFault())
    {
        Serial.println("BLDC test: FAULT active after enable, stopping");
        _drv.disable();
        _stopped = true;
        return;
    }

    _step = Step::Off;
    applyStep();
    _lastStepTime = millis();
    Serial.println("BLDC test: started, switching phase A (P9 / MOTOR3)");
}

void BLDCPhaseTest::update()
{
    // keep printing even after a fault so you can still see the voltages
    if (millis() - _lastPrintTime >= PrintTime_ms)
    {
        _lastPrintTime = millis();
        printVoltages();
    }

    if (_stopped)
    {
        return;
    }

    if (_drv.isFault())
    {
        _drv.disable();
        _stopped = true;
        Serial.println("BLDC test: FAULT, gate driver disabled");
        return;
    }

    if (millis() - _lastStepTime < StepTime_ms)
    {
        return;
    }
    _lastStepTime = millis();

    switch (_step)
    {
        case Step::Off:  _step = Step::Low;  break;
        case Step::Low:  _step = Step::High; break;
        case Step::High: _step = Step::Off;  break;
    }
    applyStep();
}

void BLDCPhaseTest::applyStep()
{
    switch (_step)
    {
        case Step::Off:  _drv.setPhase(TestPhase, DRV8302::PhaseState::Off);  break;
        case Step::Low:  _drv.setPhase(TestPhase, DRV8302::PhaseState::Low);  break;
        case Step::High: _drv.setPhase(TestPhase, DRV8302::PhaseState::High); break;
    }
    Serial.printf("BLDC test: phase A -> %s\n", stepName());
}

void BLDCPhaseTest::printVoltages()
{
    // DEBUG: every pin read identically, inline, same as the V_SUPPLY path.
    // raw = mV at the ESP32 pin, scaled = after the 39k/2k2 divider.
    // GPIO3 is unconnected on the board, as a floating-pin reference.
    const uint32_t mvA   = analogReadMilliVolts(PinMap::Sens3); // GPIO5
    const uint32_t mvB   = analogReadMilliVolts(PinMap::Sens2); // GPIO2
    const uint32_t mvC   = analogReadMilliVolts(PinMap::Sens1); // GPIO1
    const uint32_t mvSup = analogReadMilliVolts(PinMap::AnIn);  // GPIO13
    const uint32_t mvNC  = analogReadMilliVolts(3);             // GPIO3, not connected

    Serial.printf("[%-4s] A(io5)=%4lumV %6.2fV  B(io2)=%4lumV %6.2fV  C(io1)=%4lumV %6.2fV  Vsup(io13)=%4lumV %6.2fV  NC(io3)=%4lumV%s\n",
                  _stopped ? "STOP" : stepName(),
                  (unsigned long)mvA,   mvA   / 1000.0f * PhaseSense::DIVIDER_RATIO,
                  (unsigned long)mvB,   mvB   / 1000.0f * PhaseSense::DIVIDER_RATIO,
                  (unsigned long)mvC,   mvC   / 1000.0f * PhaseSense::DIVIDER_RATIO,
                  (unsigned long)mvSup, mvSup / 1000.0f * PhaseSense::DIVIDER_RATIO,
                  (unsigned long)mvNC,
                  _drv.isFault() ? "  FAULT" : "");
}

const char* BLDCPhaseTest::stepName() const
{
    switch (_step)
    {
        case Step::Off:  return "OFF";
        case Step::Low:  return "LOW";
        case Step::High: return "HIGH";
    }
    return "?";
}
