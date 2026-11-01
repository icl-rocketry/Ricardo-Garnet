#pragma once
/*
 * Minimal DRV8302 gate driver wrapper.
 *
 * Board strapping (from Ricardo-Garnet schematic):
 *  - M_PWM = GND  -> 6-PWM mode: every FET has its own input (INH_x / INL_x)
 *  - M_OC  = 3V3  -> overcurrent LATCHES off (FAULT + OCTW low); clear by toggling EN_GATE
 *  - GAIN  = GND  -> shunt amps at 10 V/V
 *  - OC_ADJ = 3V3 * R602 / (R601 + R602). VDS trip voltage, so
 *             I_trip ~= V(OC_ADJ) / Rds(on) (IRFS7530 ~2 mOhm).
 *             R601/R602 have NO value in the schematic/BOM - check what's fitted.
 *  - DTC   = 10k to GND -> dead time
 *  - FAULT / OCTW are open drain with external pull-ups (R603 / R605)
 *  - Shunts: 10 mOhm on legs A and C, leg B source straight to GND
 *
 * Phase A = H1/L1 -> SH_A -> connector P701
 * Phase B = H2/L2 -> SH_B -> connector P702
 * Phase C = H3/L3 -> SH_C -> connector P703
 *
 * Fault decoding (DRV8302 datasheet, latch OC mode):
 *   FAULT low + OCTW low  -> overcurrent (VDS trip) or over-temperature shutdown
 *   FAULT low, OCTW high  -> PVDD/GVDD undervoltage or GVDD overvoltage
 *   FAULT high, OCTW low  -> over-temperature warning only
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

    DRV8302(int enGate, int fault, int ocTw, int dcCal,
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

    // FAULT / OCTW pins are active low
    bool isFault() const;
    bool isOcTw() const;

    // human readable cause, based on FAULT + OCTW
    const char* faultName() const;

private:
    const int _enGate;
    const int _fault;
    const int _ocTw;
    const int _dcCal;
    const int _inh[3];
    const int _inl[3];

    bool _enabled = false;
};
