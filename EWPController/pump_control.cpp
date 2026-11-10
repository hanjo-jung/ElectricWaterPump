#include "pump_control.h"

// 4 + log10val[T - 92] gives the band base: 92 -> 4 (12.5%), 93 -> 7, 94 -> 8, ... 97 -> 11
static const unsigned char log10val[16] = {0, 3, 4, 5, 6, 7, 8, 8,
                                           8, 8, 8, 8, 8, 8, 8, 8};

static int Trim16(int value)
{
  return (value < 0) ? 0 : ((value > 15) ? 15 : value);
}

void PumpReset(PumpState &s)
{
  s.index = MaxDutyIndex;
  s.baseIndex = MaxDutyIndex;
  for (int i = 0; i < TempWindow; ++i)
  {
    s.temps[i] = 0;
    s.diffs[i] = 0;
    s.devs[i] = 0;
  }
  s.head = 0;
  s.historyValid = false;
  s.highLatched = false;
  s.oilHotLatched = false;
  s.warmUpCount = 0;
}

void PumpUpdateHistory(PumpState &s, bool tempValid, int coolant)
{
  if (!tempValid)
  {
    s.historyValid = false;
    return;
  }
  if (!s.historyValid)
  {
    // no usable past samples: start as if the temperature had been steady
    for (int i = 0; i < TempWindow; ++i)
    {
      s.temps[i] = coolant;
      s.diffs[i] = 0;
      s.devs[i] = coolant - TargetTemp;
    }
    s.head = 0;
    s.historyValid = true;
    return;
  }
  int prev = (s.head + TempWindow - 1) % TempWindow;
  s.diffs[s.head] = coolant - s.temps[prev];
  s.temps[s.head] = coolant;
  s.devs[s.head] = coolant - TargetTemp;
  s.head = (s.head + 1) % TempWindow;
}

int PumpRate(const PumpState &s)
{
  int sum = 0;
  bool monoInc = true;
  bool monoDec = true;
  for (int i = 0; i < TempWindow; ++i)
  {
    int d = s.diffs[i];
    if (d < 0) monoInc = false;
    else if (d > 0) monoDec = false;
    sum += d;
  }
  return (monoInc || monoDec) ? sum : 0;
}

int PumpAccumulated(const PumpState &s)
{
  int accu = 0;
  for (int i = 0; i < TempWindow; ++i)
  {
    accu += s.devs[i];
  }
  return accu;
}

// band control (LowTemp <= T < MaxTemp, no latch): base step + rate/accumulation correction
static int BandIndex(int t, int diff, int accu)
{
  int diverge = Trim16(t - TargetTemp);
  int bonus = Trim16(t - (TargetTemp - 2));
  int delta;

  if (t < TargetTemp)
  {
    if (diff < 0) delta = -3;
    else if (diff == 0)
    {
      // keep below the TargetTemp steady case (+2) so duty rises with temperature:
      // steady 90 -> 12.5%, 91 -> 25%, 92 -> 37.5%
      if (accu >= -10) delta = 1;
      else if (accu >= -20) delta = 0;
      else delta = -1;
    }
    else delta = (accu > -20) ? log10val[bonus] : 0;
  }
  else if (t == TargetTemp)
  {
    if (diff < 0) delta = 0;
    else if (diff == 0) delta = (accu >= 0) ? 2 : 0;
    else delta = (accu > -10) ? (10 + accu) : 4;
  }
  else
  {
    if (diff < 0) delta = diverge;
    else if (diff == 0) delta = log10val[diverge];
    else delta = log10val[bonus];
  }

  if (delta < BandMinDelta) delta = BandMinDelta;
  return BandBaseIndex + log10val[diverge] + delta;
}

int PumpComputeIndex(const PumpInputs &in, PumpState &s)
{
  PumpUpdateHistory(s, in.tempValid, in.coolant);

  /*
   * tempValid canError engineStarted
   *    F         F         F         while ECU reflashing possibly
   *    F         T         F         while ECU reflashing possibly
   *    T         F         F         before cranking (key on); treat as normal to check pump
   *    T         T         F         hardly
   *    F         F         T         error; after this controller reset, temp not yet received
   *    F         T         T         error
   *    T         F         T         normal
   *    T         T         T         hardly
   *
   * Before the engine has started, a CAN error also means fail-safe: otherwise a bus
   * lost after the first RPM packet would keep the key-on 0% duty, and since no RPM
   * arrives engineStarted would never be set even after the engine is cranked.
   */
  bool failSafe = in.forceOn ||
                  (in.engineStarted && !in.tempValid) ||
                  (!in.engineStarted && (!in.rpmReceived || in.canError));
  // RPM may not arrive for a while after start, so key-on is judged by engineStarted only.
  bool keyOnWait = !in.engineStarted && (!in.tempValid || in.coolant < HighTemp);

  if (failSafe)
  {
    if (s.index < MaxDutyIndex) s.index += FailSafeStep;
    if (s.index <= 0 || MaxDutyIndex < s.index) s.index = MaxDutyIndex;
    s.baseIndex = s.index;
    return s.index;
  }
  if (keyOnWait)
  {
    s.index = s.baseIndex = KeyOnDutyIndex;
    return s.index;
  }

  // here tempValid is true: either engineStarted, or before start with coolant >= HighTemp
  int t = in.coolant;
  int diff = PumpRate(s);
  int accu = PumpAccumulated(s);

  if (t >= HighTemp) s.highLatched = true;
  else if (t < MidTemp && diff <= 0) s.highLatched = false;

  if (in.oil >= OilHotSetTemp) s.oilHotLatched = true;
  else if (in.oil <= OilHotClearTemp) s.oilHotLatched = false;

  int base;
  if (t < LowTemp)
  {
    if (in.engineStopped)
    {
      base = s.baseIndex; // hold; the oil correction below is applied once, not accumulated
    }
    else
    {
      // pulse 1 second per 5 seconds (warmUpCount counts 100 ms ticks)
      // 1..40  (4 s) -> low duty to warm up coolant
      // 41..50 (1 s) -> pulse duty to circulate coolant
      s.warmUpCount++;
      if (s.warmUpCount > WarmUpTicks)
      {
        base = WarmUpPulseIndex;
        if (s.warmUpCount >= WarmUpPeriodTicks) s.warmUpCount = 0;
      }
      else
      {
        base = WarmUpLowIndex;
      }
    }
  }
  else if (t >= MaxTemp || s.oilHotLatched || s.highLatched)
  {
    base = MaxDutyIndex;
  }
  else
  {
    base = BandIndex(t, diff, accu);
  }

  int idx = base;
  if (in.oil >= OilBoostTemp) idx += in.oil - OilBoostTemp;
  if (idx <= 0 || idx > MaxDutyIndex) idx = MaxDutyIndex;

  s.baseIndex = base;
  s.index = idx;
  return idx;
}
