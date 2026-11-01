#pragma once
/*
 * Minimal DRV8302 gate driver wrapper.
 *
 * Board strapping (from Ricardo-Garnet schematic):
 *  - M_PWM = GND  -> 6-PWM mode: every FET has its own input (INH_x / INL_x)
 *  - M_OC  = 3V3  -> overcurrent latches off; clear by toggling EN_GATE
 *  - GAIN  = GND  -> shunt amps at 10 V/V
 *  - OC_ADJ = DVDD -> OC threshold at/near max, do NOT rely on OC protection
 *  - FAULT is open drain with no external pull-up -> internal pull-up used
 *
 * Phase A = H1/L1 -> SH_A -> connector P9 ("MOTOR3")
 * Phase B = H2/L2 -> SH_B -> connector P7 ("MOTOR2")
 * Phase C = H3/L3 -> SH_C -> connector P6 ("MOTOR1")
 */

#include <cstdint>

class DRV8302
{
public:
    enum class Phase : uint8_t { A = 0, B = 1, C = 2 };

    enum class PhaseState : uint8_t
    {
        Off,  // both FETs off, phase floating
        High, // high side on, phase -> V_SUPPLY
        Low   // low side on, phase -> GND
    };

    // high side inputs are driven by LEDC so they can be PWM'd
    static constexpr uint32_t PwmFreq_Hz = 20000;
    static constexpr uint8_t PwmRes_bits = 10;
    static constexpr uint32_t PwmMax = (1u << PwmRes_bits) - 1; // 100% duty

    DRV8302(int enGate, int fault, int dcCal,
            int inhA, int inlA,
            int inhB, int inlB,
            int inhC, int inlC);

    // configure pins, all phases off, gate driver disabled
    void setup();

    // EN_GATE high/low. Disabling also turns every phase off.
    void enable();
    void disable();
    bool isEnabled() const { return _enabled; }

    // never turns both FETs of a leg on together
    void setPhase(Phase phase, PhaseState state);

    // low side off, high side PWM'd at duty (0..1). Safe to call from a timer task.
    void setPhasePwm(Phase phase, float duty);
    void allOff();

    // FAULT pin is active low
    bool isFault() const;

private:
    const int _enGate;
    const int _fault;
    const int _dcCal;
    const int _inh[3];
    const int _inl[3];

    bool _enabled = false;
};
