#include "DRV8302.h"

#include <Arduino.h>

DRV8302::DRV8302(int enGate, int fault, int ocTw, int dcCal,
                 int inhA, int inlA,
                 int inhB, int inlB,
                 int inhC, int inlC) :
    _enGate(enGate),
    _fault(fault),
    _ocTw(ocTw),
    _dcCal(dcCal),
    _inh{inhA, inhB, inhC},
    _inl{inlA, inlB, inlC}
{}

void DRV8302::setup()
{
    // gate inputs low before anything else so no FET can switch on
    for (int i = 0; i < 3; i++)
    {
        digitalWrite(_inh[i], LOW);
        digitalWrite(_inl[i], LOW);
        pinMode(_inh[i], OUTPUT);
        pinMode(_inl[i], OUTPUT);

        // high side goes to LEDC, starts at 0% duty
        ledcAttach(_inh[i], PwmFreq_Hz, PwmRes_bits);
        ledcWrite(_inh[i], 0);
    }

    digitalWrite(_enGate, LOW);
    pinMode(_enGate, OUTPUT);

    // DC_CAL low = shunt amps in normal operation
    digitalWrite(_dcCal, LOW);
    pinMode(_dcCal, OUTPUT);

    pinMode(_fault, INPUT_PULLUP);
    pinMode(_ocTw, INPUT_PULLUP);

    _enabled = false;
}

void DRV8302::enable()
{
    allOff();
    digitalWrite(_enGate, HIGH);
    // let GVDD / charge pump come up before switching anything
    delay(10);
    _enabled = true;
}

void DRV8302::disable()
{
    allOff();
    digitalWrite(_enGate, LOW);
    _enabled = false;
}

void DRV8302::setPhase(Phase phase, PhaseState state)
{
    const int idx = static_cast<int>(phase);

    // always turn the conducting FET off first, then the other on
    switch (state)
    {
        case PhaseState::High:
            digitalWrite(_inl[idx], LOW);
            ledcWrite(_inh[idx], PwmMax);
            break;
        case PhaseState::Low:
            ledcWrite(_inh[idx], 0);
            digitalWrite(_inl[idx], HIGH);
            break;
        case PhaseState::Off:
        default:
            ledcWrite(_inh[idx], 0);
            digitalWrite(_inl[idx], LOW);
            break;
    }
}

void DRV8302::setPhasePwm(Phase phase, float duty)
{
    const int idx = static_cast<int>(phase);

    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;

    digitalWrite(_inl[idx], LOW);
    ledcWrite(_inh[idx], static_cast<uint32_t>(duty * PwmMax + 0.5f));
}

void DRV8302::allOff()
{
    setPhase(Phase::A, PhaseState::Off);
    setPhase(Phase::B, PhaseState::Off);
    setPhase(Phase::C, PhaseState::Off);
}

bool DRV8302::isFault() const
{
    return digitalRead(_fault) == LOW;
}

bool DRV8302::isOcTw() const
{
    return digitalRead(_ocTw) == LOW;
}

const char* DRV8302::faultName() const
{
    const bool fault = isFault();
    const bool octw = isOcTw();
    if (fault && octw) return "OVERCURRENT (VDS trip) or OVERTEMP shutdown";
    if (fault)         return "PVDD/GVDD UNDERVOLTAGE or GVDD overvoltage";
    if (octw)          return "overtemp warning";
    return "none";
}
