// Pump duty decision, independent of Arduino so it can be unit tested on a PC
// (see test/test_pump_control.cpp). Called once per MonitorInterval (100 ms) tick.
// Spec: docs/pump-control-spec.md, "펌프 제어 로직".
#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

const int MaxDutyIndex = 11;
const int Duties[MaxDutyIndex + 1] = {0, 4, 8, 16, 32, 64, 96, 128, 160, 192, 224, 255};

const int LowTemp = 88;     // below: warm-up pulses
const int TargetTemp = 92;
const int MidTemp = 95;     // high latch released below this (when not rising)
const int HighTemp = 98;    // high latch set; key-on wait upper limit
const int MaxTemp = 99;     // at or above: 100%
const int OilHotSetTemp = 103;    // oil hot latch set
const int OilHotClearTemp = 100;  // oil hot latch cleared at or below
const int OilBoostTemp = 98;      // +1 duty step per degree above this

const int FailSafeStep = 2;       // fail-safe ramps this many steps per tick
const int KeyOnDutyIndex = 0;
const int WarmUpTicks = 40;       // 4 s at the low duty
const int WarmUpPeriodTicks = 50; // then 1 s at the pulse duty (5 s period)
const int WarmUpLowIndex = 1;     // 1.56%
const int WarmUpPulseIndex = 5;   // 25%
const int BandBaseIndex = 4;      // 12.5% at or below TargetTemp
const int BandMinDelta = -3;      // keeps the band result above 0
const int TempWindow = 10;        // ticks in the 1 s rate / accumulation window

struct PumpInputs
{
  bool forceOn;        // emergency switch (OnPin LOW)
  bool tempValid;      // coolant/oil temperature is being received (IsReadBegan)
  bool rpmReceived;    // RPM received at least once (HasRpmReceived)
  bool canError;       // IsCANBusError
  bool engineStarted;  // RPM > 0 seen since power on (HasEngineStarted)
  bool engineStopped;  // recent RPM samples all 0 (IsEngineStopped)
  int coolant;         // °C, valid only with tempValid
  int oil;             // °C, valid only with tempValid
};

struct PumpState
{
  int index;           // duty index sent to the motor
  int baseIndex;       // index before the oil correction; held while the engine is stopped in warm-up
  int temps[TempWindow];
  int diffs[TempWindow];  // coolant change per tick
  int devs[TempWindow];   // coolant - TargetTemp per tick
  int head;
  bool historyValid;   // false until the first sample after (re)gaining temperature
  bool highLatched;
  bool oilHotLatched;
  int warmUpCount;
};

void PumpReset(PumpState &s);

// Called by PumpComputeIndex() every tick, in every mode: records one coolant sample while
// temperatures are received and marks the history stale when they are not, so samples from
// before a fail-safe or key-on period never feed the rate terms.
void PumpUpdateHistory(PumpState &s, bool tempValid, int coolant);

// Sum of the per-tick changes if they are monotonic over the window, else 0.
int PumpRate(const PumpState &s);
// Sum of (coolant - TargetTemp) over the window.
int PumpAccumulated(const PumpState &s);

// Decides the duty index for this tick, updates the latches/warm-up counter and s.index.
int PumpComputeIndex(const PumpInputs &in, PumpState &s);

#endif
