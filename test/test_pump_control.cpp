// Host-side tests for EWPController/pump_control.cpp.
//   g++ -std=c++11 -Wall -Wextra -I EWPController test/test_pump_control.cpp EWPController/pump_control.cpp -o test_pump && ./test_pump
// The expectations follow docs/pump-control-spec.md. RefPumpDuty() is the
// SetPumpDuty() of commit 4a0d579 (before the extraction), kept to show that the
// extraction did not change the control outside the intended fixes.
#include <cstdio>
#include <cstdlib>
#include "pump_control.h"

static int failures = 0;
#define CHECK_EQ(actual, expected, what) do { \
  int a_ = (actual), e_ = (expected); \
  if (a_ != e_) { ++failures; std::printf("FAIL %s:%d %s: got %d, expected %d\n", __FILE__, __LINE__, what, a_, e_); } \
} while (0)

// ---------------------------------------------------------------- reference (commit 4a0d579)
struct Ref
{
  int Diff1sec[16], Accu1sec[16], Temp1sec[16];
  int TempIndex, IndexDuty, WarmUpCnt;
  bool IsHighReached, IsOilHot;
};

static int RefTrim16(int v) { return (v < 0) ? 0 : ((v > 15) ? 15 : v); }

static int RefSumOfDifferentials(const Ref &r)
{
  const unsigned char idx[21] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0,};
  int sum = 0; bool monoInc = true, monoDec = true;
  for (int i = 0; i < 10; ++i)
  {
    int d = r.Diff1sec[idx[r.TempIndex + i]];
    if (d < 0) monoInc = false; else if (d > 0) monoDec = false;
    sum += d;
  }
  return (monoInc || monoDec) ? sum : 0;
}

static int RefSumOfDelta(const Ref &r)
{
  const unsigned char idx[21] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0,};
  int accu = 0;
  for (int i = 0; i < 10; ++i) accu += r.Accu1sec[idx[r.TempIndex + i]];
  return accu;
}

static void RefPumpDuty(Ref &r, const PumpInputs &in)
{
  const unsigned char nextIdx[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6};
  const unsigned char prevIdx[16] = {9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4};
  const bool IsReadBegan = in.tempValid, HasEngineStarted = in.engineStarted;
  const bool HasRpmReceived = in.rpmReceived, IsCANBusError = in.canError;
  const int CurTemp = in.coolant, OilTemp = in.oil;
  if (in.forceOn || (!IsReadBegan && HasEngineStarted) ||
      (!HasEngineStarted && (!HasRpmReceived || IsCANBusError)))
  {
    if (r.IndexDuty < MaxDutyIndex) r.IndexDuty += 2;
    if (r.IndexDuty <= 0 || MaxDutyIndex < r.IndexDuty) r.IndexDuty = MaxDutyIndex;
  }
  else if ((!IsReadBegan || (IsReadBegan && CurTemp < HighTemp)) && (!HasEngineStarted && HasRpmReceived))
  {
    r.IndexDuty = 0;
  }
  else
  {
    const unsigned char log10val[16] = {0, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
    if (r.TempIndex < 0 || 10 <= r.TempIndex) r.TempIndex = 0;
    r.Temp1sec[r.TempIndex] = CurTemp;
    r.Diff1sec[r.TempIndex] = CurTemp - r.Temp1sec[prevIdx[r.TempIndex]];
    int diverge = CurTemp - TargetTemp;
    r.Accu1sec[r.TempIndex] = diverge;
    diverge = RefTrim16(diverge);
    int defaultDutyIdx = 4 + log10val[diverge];
    r.TempIndex = nextIdx[r.TempIndex];
    int diff = RefSumOfDifferentials(r);
    int accu = RefSumOfDelta(r);
    if (CurTemp >= HighTemp) r.IsHighReached = true;
    else if (CurTemp < MidTemp && diff <= 0) r.IsHighReached = false;
    if (OilTemp >= 103) r.IsOilHot = true;
    else if (OilTemp <= 100) r.IsOilHot = false;
    if (CurTemp < LowTemp)
    {
      if (IsReadBegan && !in.engineStopped)
      {
        r.WarmUpCnt++;
        if (r.WarmUpCnt > 40) { r.IndexDuty = 5; if (r.WarmUpCnt >= 50) r.WarmUpCnt = 0; }
        else r.IndexDuty = 1;
      }
    }
    else if (CurTemp >= MaxTemp || r.IsOilHot || r.IsHighReached)
    {
      r.IndexDuty = MaxDutyIndex;
    }
    else
    {
      int delta;
      int bonus = RefTrim16(CurTemp - (TargetTemp - 2));
      if (CurTemp < TargetTemp)
      {
        if (diff < 0) delta = -3;
        else if (diff == 0) delta = (accu >= -10) ? 1 : ((accu >= -20) ? 0 : -1);
        else delta = (accu > -20) ? log10val[bonus] : 0;
      }
      else if (CurTemp == TargetTemp)
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
      if (delta < -3) delta = -3;
      r.IndexDuty = defaultDutyIdx + delta;
    }
    if (OilTemp >= 98) r.IndexDuty += OilTemp - 98;
    if (r.IndexDuty <= 0 || r.IndexDuty > MaxDutyIndex) r.IndexDuty = MaxDutyIndex;
  }
}

// reference seeded the way the new code seeds its history (steady at the first sample)
static void RefSeed(Ref &r, int coolant)
{
  for (int i = 0; i < 16; ++i) { r.Temp1sec[i] = coolant; r.Diff1sec[i] = 0; r.Accu1sec[i] = coolant - TargetTemp; }
  r.TempIndex = 0; r.IndexDuty = MaxDutyIndex; r.WarmUpCnt = 0; r.IsHighReached = false; r.IsOilHot = false;
}

// ---------------------------------------------------------------- helpers
static PumpInputs Running(int coolant, int oil = 80)
{
  PumpInputs in;
  in.forceOn = false; in.tempValid = true; in.rpmReceived = true; in.canError = false;
  in.engineStarted = true; in.engineStopped = false; in.coolant = coolant; in.oil = oil;
  return in;
}

static PumpInputs KeyOn(int coolant)
{
  PumpInputs in = Running(coolant);
  in.engineStarted = false; in.engineStopped = true;
  return in;
}

static int RunTicks(PumpState &s, const PumpInputs &in, int ticks)
{
  int idx = -1;
  for (int i = 0; i < ticks; ++i) idx = PumpComputeIndex(in, s);
  return idx;
}

// ---------------------------------------------------------------- spec tables
static void TestSteadyTable()
{
  // spec "수온이 일정할 때" table (oil correction excluded)
  const int temps[] = {88, 89, 90, 91, 92, 93, 94, 95, 96, 97};
  const int expect[] = {3, 3, 4, 5, 6, 10, 11, 11, 11, 11};
  for (int i = 0; i < 10; ++i)
  {
    PumpState s; PumpReset(s);
    CHECK_EQ(RunTicks(s, Running(temps[i]), 30), expect[i], "steady duty");
  }
}

static void TestBandBase()
{
  // falling at 92 -> delta 0 -> base 4; falling at 95 -> base 9 + (95-92) -> 11 (clamped... 12 -> 11)
  PumpState s; PumpReset(s);
  RunTicks(s, Running(93), 20);
  CHECK_EQ(PumpComputeIndex(Running(92), s), 4, "falling to 92");
  PumpReset(s);
  RunTicks(s, Running(90), 20);
  CHECK_EQ(PumpComputeIndex(Running(89), s), 1, "falling below 92: 4 - 3");
}

static void TestModes()
{
  PumpState s; PumpReset(s);
  // power on, nothing received: 100% from the first tick
  PumpInputs in = KeyOn(0); in.rpmReceived = false; in.tempValid = false;
  CHECK_EQ(PumpComputeIndex(in, s), 11, "power on fail-safe");

  // key on with RPM 0 received, coolant < 98 or unknown: 0%
  in = KeyOn(60); in.tempValid = false;
  CHECK_EQ(PumpComputeIndex(in, s), 0, "key on, no temp");
  CHECK_EQ(PumpComputeIndex(KeyOn(97), s), 0, "key on 97");
  // key on, already hot: normal control (high latch -> 100%)
  CHECK_EQ(PumpComputeIndex(KeyOn(98), s), 11, "key on 98");

  // fail-safe ramps by 2 steps
  PumpReset(s);
  RunTicks(s, Running(70), 1); // warm-up low duty = 1
  CHECK_EQ(s.index, 1, "warm-up low");
  in = Running(70); in.forceOn = true;
  CHECK_EQ(PumpComputeIndex(in, s), 3, "ramp 1");
  CHECK_EQ(PumpComputeIndex(in, s), 5, "ramp 2");
  RunTicks(s, in, 2);
  CHECK_EQ(PumpComputeIndex(in, s), 11, "ramp end");

  // fail-safe from 0 also ramps: 2, 4, ... 10, 11
  PumpReset(s);
  PumpComputeIndex(KeyOn(60), s);
  in = Running(60); in.tempValid = false;
  CHECK_EQ(PumpComputeIndex(in, s), 2, "engine started, temp lost: ramp from 0");
  CHECK_EQ(RunTicks(s, in, 5), 11, "ramp from 0 reaches 11 after 6 ticks");

  // CAN lost before start (fix in 4a0d579): fail-safe, not key-on 0%
  PumpReset(s);
  PumpComputeIndex(KeyOn(60), s);
  in = KeyOn(60); in.tempValid = false; in.canError = true;
  CHECK_EQ(PumpComputeIndex(in, s), 2, "CAN lost before start: fail-safe ramp, not key-on 0%");
  CHECK_EQ(RunTicks(s, in, 5), 11, "CAN lost before start reaches 100%");
}

static void TestWarmUp()
{
  PumpState s; PumpReset(s);
  for (int cycle = 0; cycle < 2; ++cycle)
  {
    for (int i = 0; i < 40; ++i) CHECK_EQ(PumpComputeIndex(Running(70), s), 1, "warm-up 4 s low");
    for (int i = 0; i < 10; ++i) CHECK_EQ(PumpComputeIndex(Running(70), s), 5, "warm-up 1 s pulse");
  }
  // engine stopped: hold the previous step
  PumpInputs in = Running(70); in.engineStopped = true;
  CHECK_EQ(RunTicks(s, in, 20), 5, "warm-up hold while stopped (last was the pulse)");
}

static void TestOil()
{
  PumpState s; PumpReset(s);
  CHECK_EQ(RunTicks(s, Running(90, 100), 30), 6, "oil 100 -> +2");  // steady 90 = 4
  PumpReset(s);
  CHECK_EQ(RunTicks(s, Running(92, 104), 30), 11, "oil hot latch");
  CHECK_EQ(RunTicks(s, Running(92, 101), 30), 11, "oil latch holds at 101");
  CHECK_EQ(RunTicks(s, Running(92, 100), 30), 8, "oil latch cleared at 100: 6 + 2");
  // hold while stopped + hot oil must not accumulate (fix ④)
  PumpReset(s);
  RunTicks(s, Running(70, 100), 1); // 1 + 2 = 3
  PumpInputs in = Running(70, 100); in.engineStopped = true;
  CHECK_EQ(RunTicks(s, in, 50), 3, "hold does not accumulate oil correction");
}

static void TestHighLatch()
{
  PumpState s; PumpReset(s);
  RunTicks(s, Running(98), 5);
  CHECK_EQ(RunTicks(s, Running(95), 30), 11, "latched at 95");
  CHECK_EQ(RunTicks(s, Running(94), 30), 11, "released at 94 steady -> 11 anyway");
  CHECK_EQ(s.highLatched ? 1 : 0, 0, "latch released below 95");
  CHECK_EQ(RunTicks(s, Running(92), 30), 6, "back to 92 steady");
}

static void TestHistoryAfterGap()
{
  // fix ③: the first band tick after fail-safe uses a fresh (steady) history
  PumpState s; PumpReset(s);
  RunTicks(s, Running(80), 30);
  PumpInputs lost = Running(92); lost.tempValid = false;
  RunTicks(s, lost, 30);
  CHECK_EQ(PumpComputeIndex(Running(92), s), 6, "92 after temp loss treated as steady");

  // key-on wait now feeds the history, so start-up at 92 is steady as well
  PumpReset(s);
  RunTicks(s, KeyOn(92), 30);
  CHECK_EQ(PumpComputeIndex(Running(92), s), 6, "92 right after start");
}

// ---------------------------------------------------------------- equivalence with the reference
static unsigned long rng = 12345;
static int Rand(int n) { rng = rng * 1103515245UL + 12345UL; return (int)((rng >> 16) % (unsigned long)n); }

static void TestEquivalence(bool allowStops, int oilMax, int runs)
{
  for (int run = 0; run < runs; ++run)
  {
    int t = 75 + Rand(25), oil = 80 + Rand(oilMax - 80);
    PumpState s; PumpReset(s);
    Ref r;
    bool stopped = false;
    for (int tick = 0; tick < 600; ++tick)
    {
      int step = Rand(10);
      if (step == 0 && t > 70) --t; else if (step == 1 && t < 105) ++t;
      int o = Rand(10);
      if (o == 0 && oil > 80) --oil; else if (o == 1 && oil < oilMax) ++oil;
      if (allowStops && Rand(100) == 0) stopped = !stopped;
      PumpInputs in = Running(t, oil); in.engineStopped = stopped;
      if (tick == 0) RefSeed(r, t);
      RefPumpDuty(r, in);
      int got = PumpComputeIndex(in, s);
      if (got != r.IndexDuty)
      {
        ++failures;
        std::printf("FAIL equivalence run %d tick %d T=%d oil=%d stopped=%d: got %d, ref %d\n",
                    run, tick, t, oil, stopped, got, r.IndexDuty);
        break;
      }
    }
  }
}

int main()
{
  TestSteadyTable();
  TestBandBase();
  TestModes();
  TestWarmUp();
  TestOil();
  TestHighLatch();
  TestHistoryAfterGap();
  TestEquivalence(false, 110, 300); // engine running, any oil temperature
  TestEquivalence(true, 98, 300);   // with engine stops, oil below the correction (fix ④ excluded)
  if (failures) { std::printf("%d failure(s)\n", failures); return 1; }
  std::printf("all pump control tests passed\n");
  return 0;
}
