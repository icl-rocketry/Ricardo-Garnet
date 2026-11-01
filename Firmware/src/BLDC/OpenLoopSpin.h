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
 *   ALIGN : hold step 0 (A+ B-) at AlignDuty so the rotor snaps to a known position
 *   RAMP  : step through the 6-step table, electrical frequency rising linearly
 *           StartFreq_Hz -> RunFreq_Hz over RampTime_ms, duty StartDuty -> RunDuty
 *   RUN   : hold RunFreq_Hz / RunDuty for RunTime_ms
 *   STOP  : all FETs off, gate driver disabled (press reset to run again)
 *
 * High side is PWM'd at 20 kHz, low side is held solidly on.
 * Commutation runs from a one-shot esp_timer so step timing doesn't depend on
 * the main loop's 1 ms tick.
 *
 * WARNING: there is no current measurement on this board revision (SO1/SO2 land
 * on non-ADC pins) and OC_ADJ puts the DRV8302 overcurrent trip at maximum.
 * Duty is the only thing limiting current. Use a current-limited bench supply
 * and keep the duties low until you know the motor's winding resistance.
 *
 * Mechanical RPM = electrical Hz * 60 / pole pairs.
 */
class OpenLoopSpin
{
public:
    // --- profile, tune these ---
    static constexpr uint32_t StartDelay_ms = 2000;
    static constexpr float    AlignDuty     = 0.10f;
    static constexpr uint32_t AlignTime_ms  = 500;
    static constexpr float    StartFreq_Hz  = 2.0f;   // electrical
    static constexpr float    RunFreq_Hz    = 30.0f;  // electrical
    static constexpr float    StartDuty     = 0.10f;
    static constexpr float    RunDuty       = 0.15f;
    static constexpr uint32_t RampTime_ms   = 4000;
    static constexpr uint32_t RunTime_ms    = 10000;
    static constexpr bool     Reverse       = false;
    static constexpr uint32_t PrintTime_ms  = 250;

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

    uint8_t _step = 0;          // timer task only
    uint32_t _modeStart = 0;    // timer task only (after start)
    uint32_t _bootTime = 0;
    uint32_t _lastPrintTime = 0;

    static void timerCallback(void* arg);
    void onTimer();
    void start();
    void applyStep(uint8_t step, float duty);
    void shutdown(const char* reason);
    void arm(uint32_t us);
    void printStatus();
    static const char* modeName(Mode m);
};
