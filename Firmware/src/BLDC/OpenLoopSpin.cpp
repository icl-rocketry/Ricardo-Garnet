#include "OpenLoopSpin.h"

#include <Arduino.h>

#include "Config/pinmap_config.h"

namespace
{
    using Phase = DRV8302::Phase;
    using PState = DRV8302::PhaseState;

    struct Step { Phase high; Phase low; Phase floating; };

    // standard 6-step table: one phase PWM'd high, one held low, one floating
    constexpr Step StepTable[6] = {
        {Phase::A, Phase::B, Phase::C},
        {Phase::A, Phase::C, Phase::B},
        {Phase::B, Phase::C, Phase::A},
        {Phase::B, Phase::A, Phase::C},
        {Phase::C, Phase::A, Phase::B},
        {Phase::C, Phase::B, Phase::A},
    };

    float ramp(float a, float b, float t) { return a + (b - a) * t; }
}

OpenLoopSpin::OpenLoopSpin() :
    _drv(PinMap::EnGate, PinMap::Fault, PinMap::DcCal,
         PinMap::H1, PinMap::L1,
         PinMap::H2, PinMap::L2,
         PinMap::H3, PinMap::L3),
    // SENS3 = phase A, SENS2 = phase B, SENS1 = phase C
    _sense(PinMap::Sens3, PinMap::Sens2, PinMap::Sens1, PinMap::AnIn)
{}

void OpenLoopSpin::setup()
{
    _sense.setup();
    _drv.setup();

    const esp_timer_create_args_t args = {
        .callback = &OpenLoopSpin::timerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "bldc_commutate",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&args, &_timer);

    _bootTime = millis();
    Serial.printf("Open-loop spin: starting in %lu ms\n", (unsigned long)StartDelay_ms);
}

void OpenLoopSpin::update()
{
    if (millis() - _lastPrintTime >= PrintTime_ms)
    {
        _lastPrintTime = millis();
        printStatus();
    }

    if (_mode.load() == Mode::Waiting && millis() - _bootTime >= StartDelay_ms)
    {
        start();
    }
}

void OpenLoopSpin::start()
{
    _drv.enable();
    if (_drv.isFault())
    {
        shutdown("FAULT active after enable");
        return;
    }

    _step = 0;
    _freq = 0.0f;
    _duty = AlignDuty;
    _modeStart = millis();
    _mode = Mode::Align;
    applyStep(_step, AlignDuty);
    Serial.println("Open-loop spin: ALIGN");
    arm(AlignTime_ms * 1000);
}

void OpenLoopSpin::timerCallback(void* arg)
{
    static_cast<OpenLoopSpin*>(arg)->onTimer();
}

void OpenLoopSpin::onTimer()
{
    if (_drv.isFault())
    {
        _faulted = true;
        shutdown("FAULT, gate driver disabled");
        return;
    }

    Mode mode = _mode.load();
    uint32_t t = millis() - _modeStart;

    if (mode == Mode::Align)
    {
        mode = Mode::Ramp;
        _mode = mode;
        _modeStart = millis();
        t = 0;
    }

    float freq, duty;
    if (mode == Mode::Ramp)
    {
        const float frac = t >= RampTime_ms ? 1.0f : static_cast<float>(t) / RampTime_ms;
        freq = ramp(StartFreq_Hz, RunFreq_Hz, frac);
        duty = ramp(StartDuty, RunDuty, frac);
        if (frac >= 1.0f)
        {
            _mode = Mode::Run;
            _modeStart = millis();
        }
    }
    else if (mode == Mode::Run)
    {
        if (t >= RunTime_ms)
        {
            shutdown("run time elapsed");
            return;
        }
        freq = RunFreq_Hz;
        duty = RunDuty;
    }
    else
    {
        return; // Waiting / Stopped, nothing to do
    }

    _step = Reverse ? (_step + 5) % 6 : (_step + 1) % 6;
    applyStep(_step, duty);
    _freq = freq;
    _duty = duty;

    // 6 commutation steps per electrical revolution
    arm(static_cast<uint32_t>(1e6f / (6.0f * freq)));
}

void OpenLoopSpin::applyStep(uint8_t step, float duty)
{
    const Step& s = StepTable[step];
    // only one phase changes between consecutive steps; release first, then drive
    _drv.setPhase(s.floating, PState::Off);
    _drv.setPhase(s.low, PState::Low);
    _drv.setPhasePwm(s.high, duty);
}

void OpenLoopSpin::arm(uint32_t us)
{
    esp_timer_start_once(_timer, us);
}

void OpenLoopSpin::shutdown(const char* reason)
{
    if (_timer) esp_timer_stop(_timer);
    _drv.disable();
    _freq = 0.0f;
    _duty = 0.0f;
    _mode = Mode::Stopped;
    Serial.printf("Open-loop spin: STOP (%s)\n", reason);
}

void OpenLoopSpin::printStatus()
{
    const Mode mode = _mode.load();
    Serial.printf("[%-5s] f=%5.1fHz duty=%4.1f%%  A(io%d)=%5.2fV B(io%d)=%5.2fV C(io%d)=%5.2fV  Vsup(io%d)=%5.2fV%s\n",
                  modeName(mode),
                  _freq.load(), _duty.load() * 100.0f,
                  PinMap::Sens3, _sense.readPhase(DRV8302::Phase::A),
                  PinMap::Sens2, _sense.readPhase(DRV8302::Phase::B),
                  PinMap::Sens1, _sense.readPhase(DRV8302::Phase::C),
                  PinMap::AnIn, _sense.readSupply(),
                  (_faulted.load() || _drv.isFault()) ? "  FAULT" : "");
}

const char* OpenLoopSpin::modeName(Mode m)
{
    switch (m)
    {
        case Mode::Waiting: return "WAIT";
        case Mode::Align:   return "ALIGN";
        case Mode::Ramp:    return "RAMP";
        case Mode::Run:     return "RUN";
        case Mode::Stopped: return "STOP";
    }
    return "?";
}
