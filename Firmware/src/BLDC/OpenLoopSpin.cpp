#include "OpenLoopSpin.h"

#include <Arduino.h>
#include <cmath>

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
}

OpenLoopSpin::OpenLoopSpin() :
    _drv(PinMap::EnGate, PinMap::Fault, PinMap::OcTw, PinMap::DcCal,
         PinMap::H1, PinMap::L1,
         PinMap::H2, PinMap::L2,
         PinMap::H3, PinMap::L3),
    // SENS3 = phase A, SENS2 = phase B, SENS1 = phase C
    _sense(PinMap::Sens3, PinMap::Sens2, PinMap::Sens1, PinMap::AnIn)
{}

void OpenLoopSpin::setup()
{
    _sense.setup();
    analogSetPinAttenuation(PinMap::MosTemp, ADC_11db);
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
    // only the main loop touches the ADC; the timer task uses the cached value
    const float vsup = _sense.readSupply();
    _vsup = vsup;

    const Mode running = _mode.load();
    if ((running == Mode::Align || running == Mode::Ramp || running == Mode::Run) && vsup < MinSupply_V)
    {
        shutdown("supply below MinSupply_V, current limit hit?");
    }

    if (millis() - _lastPrintTime >= PrintTime_ms)
    {
        _lastPrintTime = millis();
        const float tempC = printStatus();

        const Mode mode = _mode.load();
        if (tempC >= OverTemp_C && mode != Mode::Stopped && mode != Mode::Waiting)
        {
            shutdown("MOSFET over temperature");
        }
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
        _faultCause = _drv.faultName();
        _faultMode = Mode::Waiting;
        shutdown("FAULT active after enable");
        return;
    }

    _vsup = _sense.readSupply();
    _vsupRef = _vsup.load();
    if (_vsup.load() < MinSupply_V)
    {
        shutdown("supply below MinSupply_V at start");
        return;
    }

    const float duty = dutyFor(AlignVoltage_V);
    _step = 0;
    _freq = 0.0f;
    _duty = duty;
    _modeStart = millis();
    _mode = Mode::Align;
    applyStep(_step, duty);
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
        // decode before shutdown: EN_GATE low may clear the fault pins
        _faulted = true;
        _faultCause = _drv.faultName();
        _faultMode = _mode.load();
        _faultFreq = _freq.load();
        _faultDuty = _duty.load();
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

    float freq;
    if (mode == Mode::Ramp)
    {
        freq = StartFreq_Hz + RampRate_HzPerS * (t / 1000.0f);
        if (freq >= RunFreq_Hz)
        {
            freq = RunFreq_Hz;
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
    }
    else
    {
        return; // Waiting / Stopped, nothing to do
    }

    const float duty = dutyFor(boostFor(freq) + BemfPerHz_V * freq);

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

float OpenLoopSpin::boostFor(float freq)
{
    if (freq >= BoostTaperFreq_Hz) return BoostRun_V;
    float t = (freq - StartFreq_Hz) / (BoostTaperFreq_Hz - StartFreq_Hz);
    if (t < 0.0f) t = 0.0f;
    return BoostStart_V + (BoostRun_V - BoostStart_V) * t;
}

float OpenLoopSpin::dutyFor(float volts) const
{
    const float vsup = _vsupRef.load();
    if (vsup < 1.0f) return 0.0f;   // no valid supply reading, don't drive
    const float duty = volts / vsup;
    return duty > MaxDuty ? MaxDuty : duty;
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
    if (_faultCause)
    {
        Serial.printf("  cause: %s  (in %s, f=%.1fHz, duty=%.1f%%)\n",
                      _faultCause, modeName(_faultMode), _faultFreq, _faultDuty * 100.0f);
    }
}

float OpenLoopSpin::printStatus()
{
    const Mode mode = _mode.load();
    const float tempC = readMosTempC();
    Serial.printf("[%-5s] f=%5.1fHz (%4.0frpm) duty=%4.1f%% (%4.2fV)  A(io%d)=%5.2fV B(io%d)=%5.2fV C(io%d)=%5.2fV  Vsup(io%d)=%5.2fV  Tmos(io%d)=%5.1fC%s\n",
                  modeName(mode),
                  _freq.load(), _freq.load() * 60.0f / PolePairs,
                  _duty.load() * 100.0f, _duty.load() * _vsup.load(), // actual, sags with supply
                  PinMap::Sens3, _sense.readPhase(DRV8302::Phase::A),
                  PinMap::Sens2, _sense.readPhase(DRV8302::Phase::B),
                  PinMap::Sens1, _sense.readPhase(DRV8302::Phase::C),
                  PinMap::AnIn, _vsup.load(),
                  PinMap::MosTemp, tempC,
                  (_faulted.load() || _drv.isFault()) ? "  FAULT" : "");
    return tempC;
}

float OpenLoopSpin::readMosTempC()
{
    // NTC on the high side of the divider: V = Vref * Rb / (Rntc + Rb)
    float mv = static_cast<float>(analogReadMilliVolts(PinMap::MosTemp));
    if (mv < 1.0f) mv = 1.0f;                          // open NTC -> very cold
    if (mv > NtcVref_mV - 1.0f) mv = NtcVref_mV - 1.0f; // shorted NTC -> very hot
    const float rNtc = NtcRBottom * (NtcVref_mV / mv - 1.0f);

    // beta equation
    constexpr float T25_K = 298.15f;
    const float invT = 1.0f / T25_K + std::log(rNtc / NtcR25) / NtcBeta;
    return 1.0f / invT - 273.15f;
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
