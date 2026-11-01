#pragma once

#include <atomic>
#include <cstdint>

#include "esp_timer.h"

#include "DRV8302.h"
#include "PhaseSense.h"

/*
 * Open-loop 6-step (trapezoidal) spin-up. No position feedback.
 *
 * Sequence (starts StartDelay_ms after boot):
 *   ALIGN : hold step 0 (A+ B-) at AlignVoltage_V so the rotor snaps to a known position
 *   RAMP  : step through the 6-step table, electrical frequency rising at
 *           RampRate_HzPerS from StartFreq_Hz to RunFreq_Hz
 *   RUN   : hold RunFreq_Hz for RunTime_ms
 * Duty is scaled with frequency (V/f) from the motor KV and the supply voltage
 * measured just before starting (not the live value: with a current-limited
 * supply, compensating for sag raises duty, draws more current, sags further
 * and collapses).
 *   STOP  : all FETs off, gate driver disabled (press reset to run again)
 *
 * High side is PWM'd at 20 kHz, low side is held solidly on.
 * Commutation runs from a one-shot esp_timer so step timing doesn't depend on
 * the main loop's 1 ms tick.
 *
 * WARNING: there is no current measurement on this board revision (SO1/SO2 land
 * on non-ADC pins) and OC_ADJ puts the DRV8302 overcurrent trip at maximum.
 * Applied voltage vs back-EMF is the only thing limiting current. Use a current-limited bench supply
 * and keep the duties low until you know the motor's winding resistance.
 *
 * Mechanical RPM = electrical Hz * 60 / pole pairs.
 */
class OpenLoopSpin
{
public:
    // --- motor (3112 900KV, 12N14P) ---
    static constexpr float    MotorKV       = 900.0f;
    static constexpr float    PolePairs     = 7.0f;
    // back-EMF per electrical Hz: rpm = f * 60 / pp, V = rpm / KV
    static constexpr float    BemfPerHz_V   = 60.0f / (PolePairs * MotorKV); // ~0.0095 V/Hz

    // --- profile, tune these ---
    // duty = (boost(f) + BemfPerHz_V * f) / Vsupply_at_start
    static constexpr uint32_t StartDelay_ms = 2000;
    static constexpr float    AlignVoltage_V = 1.20f;  // = the 10% @ 12 V that worked; small duties get eaten by dead time / switching
    static constexpr uint32_t AlignTime_ms  = 500;
    // boost is only needed to break away from standstill; once spinning it's pure I*R heat
    // (~30 A at 1.2 V into this motor). Taper it from BoostStart_V to BoostRun_V by BoostTaperFreq_Hz.
    static constexpr float    BoostStart_V  = 1.20f;   // = the 10% @ 12 V start that worked
    static constexpr float    BoostRun_V    = 0.40f;   // lower until it loses sync, then back off
    static constexpr float    BoostTaperFreq_Hz = 30.0f;
    static constexpr float    StartFreq_Hz  = 2.0f;    // electrical
    static constexpr float    RunFreq_Hz    = 100.0f;  // electrical (~857 rpm)
    static constexpr float    RampRate_HzPerS = 10.0f;
    static constexpr float    MaxDuty       = 0.60f;   // hard clamp
    static constexpr float    MinSupply_V   = 9.0f;    // stop if the supply sags (CC limit)
    static constexpr uint32_t RunTime_ms    = 10000;
    static constexpr bool     Reverse       = false;
    static constexpr uint32_t PrintTime_ms  = 250;
    static constexpr float    OverTemp_C    = 80.0f;  // MOSFET NTC cutoff

    // MOS_TEMP: NTC R111 (Murata NCU15XH103F6SRC) 3V3 -> GPIO14, R112 10k GPIO14 -> GND
    static constexpr float NtcR25 = 10000.0f;
    static constexpr float NtcBeta = 3380.0f;  // B25/50
    static constexpr float NtcRBottom = 10000.0f;
    static constexpr float NtcVref_mV = 3300.0f;

    OpenLoopSpin();

    void setup();
    void update(); // non-blocking, call from the main loop

private:
    enum class Mode : uint8_t { Waiting, Align, Ramp, Run, Stopped };

    DRV8302 _drv;
    PhaseSense _sense;
    esp_timer_handle_t _timer = nullptr;

    // written by the timer task, read by the main loop for printing
    std::atomic<Mode> _mode{Mode::Waiting};
    std::atomic<float> _freq{0.0f};
    std::atomic<float> _duty{0.0f};
    std::atomic<bool> _faulted{false};
    std::atomic<float> _vsup{0.0f};    // live, updated by the main loop (print + MinSupply check)
    std::atomic<float> _vsupRef{0.0f}; // captured at start, used for the duty calc

    // snapshot taken the moment a fault is seen
    const char* _faultCause = nullptr;
    Mode _faultMode = Mode::Waiting;
    float _faultFreq = 0.0f;
    float _faultDuty = 0.0f;  // updated by the main loop, used by the timer task

    uint8_t _step = 0;          // timer task only
    uint32_t _modeStart = 0;    // timer task only (after start)
    uint32_t _bootTime = 0;
    uint32_t _lastPrintTime = 0;

    static void timerCallback(void* arg);
    void onTimer();
    void start();
    void applyStep(uint8_t step, float duty);
    float dutyFor(float volts) const;
    static float boostFor(float freq);
    void shutdown(const char* reason);
    void arm(uint32_t us);
    float printStatus(); // returns MOSFET temp in C
    static float readMosTempC();
    static const char* modeName(Mode m);
};
