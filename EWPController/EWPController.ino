#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>
#include <CytronMotorDriver.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <rgb_lcd.h>
#include "pump_control.h"

//#define TEST
//#define DEBUG
#ifdef DEBUG
#define DBG(s)  do{Serial.print(s);}while(0)
#define DBGLN(s)  do{Serial.println(s);}while(0)
#else
#define DBG(s)
#define DBGLN(s)
#endif
//#define DEBUG1
#ifdef DEBUG1
#define DBG1(s)  do{Serial.print(s);}while(0)
#define DBG1LN(s)  do{Serial.println(s);}while(0)
#else
#define DBG1(s)
#define DBG1LN(s)
#endif
//#define DEBUG2
#ifdef DEBUG2
#define DBG2(s)  do{Serial.print(s);}while(0)
#define DBG2H(s)  do{Serial.print(s, HEX);}while(0)
#define DBG2LN(s)  do{Serial.println(s);}while(0)
#else
#define DBG2(s)
#define DBG2H(s)
#define DBG2LN(s)
#endif
//#define DEBUG3
#ifdef DEBUG3
#define DBG3LN(s)  do{Serial.println(s);}while(0)
#else
#define DBG3LN(s)
#endif
//#define DEBUG4
#ifdef DEBUG4
#define DBG4(s)  do{Serial.print(s);}while(0)
#define DBG4LN(s)  do{Serial.println(s);}while(0)
#else
#define DBG4(s)
#define DBG4LN(s)
#endif

/*
* Arduino Leonardo PWM : 3, 5, 6, 9, 10, 11, and 13
D3:  8-bit timer0
D5:  16-bit timer1&3
D6:  10-bit timer4
D9:  16-bit timer1&3
D10:  16-bit timer1&3
D11:  8-bit timer0
D13:  10-bit timer4
*
* MsTimer2 uses timer4
* msTask uses timer1
*
* Arduino D9  - Motor Driver PWM Input
* Arduino D4  - Motor Driver DIR Input -> actually pull down to GND
* Arduino GND - Motor Driver GND
*
* Interrupt
* PIN 3 INT 0 SCL OC0B
* PIN 2 INT 1 SDA
* PIN 1 INT 3 TXD1
* PIN 0 INT 2 RXD1
* PIN 7 INT 4
* attachInterrupt(digitalPinToInterrupt(<pin num>), ISR, FALLING)
*/

const int SPI_CS_PIN = 17;
const int MotorPwmPin = 9;
const int MotorDirPin = 4; // actually pulled down to GND
const int OnPin = 12;
const int OilPressurePin = A0;
const int MonitorInterval = 100; // ms
const int BeatsPerSecond = 1000 / MonitorInterval;
const unsigned long CanIdTemps = 0x360; // byte 2 oil, byte 3 coolant
const unsigned long CanIdRpm = 0x140;   // bytes 2-3 RPM (14 bits)
const int CanTempOffset = 40;           // °C = raw - 40
const int RpmWindow = 8; // engine stopped when this many consecutive RPM packets are 0
// duty steps and temperature thresholds: pump_control.h
// lcd row 0
const int PressureNowRow = 0; // current psi at PressureOffset, calibration mark at +3
const int OilOffset = 6;
const int OilRow = 0;
const int CoolantOffset = 11;
const int CoolantRow = 0;
// lcd row 1
const int RpmOffset = 7;
const int RpmRow = 1;
const int PressureOffset = 0;
const int PressureRow = 1;
const int ResetOffset = 9;
const int ResetRow = 0;
const int HeartbeatOffset = 10;
const int HeartbeatRow = 1;
const int PumpOffset = 11;
const int PumpRow = 1;
const int DipMarkOffset = 4; // oil pressure dip indicator
const int DipMarkRow = 0;
const int OPLogSize = 150; // 20 Hz (OPSampleMs) x 150 = ~7.5 sec
const int OPLogRpmRange = 50; // max psi is taken within this many rpm of the logged max rpm
// Oil pressure filter: oversampled ADC -> median (spike rejection) -> EMA (smoothing)
const int OPOversample = 4;          // analogRead() calls averaged per sample
const int OPMedianSize = 5;          // rejects up to 2 consecutive spikes
const float OPEmaAlpha = 0.3;        // ~0.15 s time constant at 20 Hz
const int OPCalibMinSamples = 20;    // ~1 s of samples before trusting the 0 psi voltage
const unsigned long OPSampleMs = 50; // 20 Hz, independent of CAN reception
// Oil pressure warning
const int BuzzerPin = -1;            // set to a free digital pin (e.g. 8) to enable an active-high buzzer
const unsigned long RpmStaleMs = 2000;     // RPM older than this is treated as unknown
const int OPMonitorMinRpm = 500;           // below this the engine is treated as not running
const unsigned long OPStartDelayMs = 3000; // ignore pressure build-up right after start
const unsigned long OPDipMs = 300;         // low this long -> DIP
const unsigned long OPLossMs = 2000;       // low this long -> LOSS
const unsigned long OPRecoverMs = 1000;    // above min + hysteresis this long -> NORMAL
const int OPHysteresisPsi = 3;
const int OPNoRpmMinPsi = 5;               // min psi when RPM is unknown (CAN lost)
const float OPSensorMinVolt = 0.1;         // below: open circuit / short to GND
const float OPSensorMaxVolt = 4.8;         // above: short to 5V
const unsigned char OP_NORMAL = 0;
const unsigned char OP_DIP = 1;
const unsigned char OP_LOSS = 2;
const unsigned char OP_SENSOR_FAULT = 3;
// HD44780 ROM font glyphs; stored inverted into CGRAM 0..4 for reverse video
const unsigned char InvGlyphSrc[5][8] = {
  {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E, 0x00}, // O
  {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E, 0x00}, // I
  {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F, 0x00}, // L
  {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A, 0x00}, // W
  {0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x04, 0x00}, // !
};
const unsigned char InvO = 0, InvI = 1, InvL = 2, InvW = 3, InvExcl = 4;
const unsigned char InvSpace = 0xFF; // ROM full block

rgb_lcd lcd;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

// Configure the motor driver.
CytronMD motor(PWM_DIR, MotorPwmPin, MotorDirPin);

bool CanResetRequested = false; // set by RequestCanReset(), handled by ResetCanIfRequested()
int LcdUpdateCount = 0;
#ifdef TEST
int CurTemp = 70; //test
#else
int CurTemp;
#endif
PumpState Pump;
int RpmHistory[RpmWindow] = {0, };
int RpmIndex = 0;
int CurRPM = 0;
int OilTemp;
bool IsEngineStopped = true;
bool HasEngineStarted = false;
bool HasRpmReceived = false;
bool IsCANBusError = false;
bool IsReadBegan = false;
int IndexDuty = MaxDutyIndex;
unsigned int CANBusResetCount = 0;

int IntervalCount = 0;
bool RpmStop = false;
bool TempStop = false;

int PrevTemp = -40;
int PrevOil = -40;
int PrevDuty = -1;
unsigned int PrevRpmCnt = 0;
unsigned int PrevTempCnt = 0;
unsigned int RpmCnt = 0;
unsigned int TempCnt = 0;
unsigned int Heartbeat = 0;

struct PressureLogT
{
  int rpm;
  int psi;
} OilPressureLogs[OPLogSize];
int OPLogHead = 0;
int OPLogTail = 0;
struct PressureLogT OPToDisp;
float VoltOffset = 0.0;
float OPVoltRaw[OPMedianSize];
int OPVoltIdx = 0;
int OPVoltSamples = 0;
float OPVoltEma = 0.0;
unsigned char OPAlarm = OP_NORMAL;
unsigned int OPDipCount = 0;
unsigned long LastRpmMs = 0;
unsigned long LastOPSampleMs = 0;
unsigned long OPRunningSince = 0;
unsigned long OPLowSince = 0;
unsigned long OPOkSince = 0;
unsigned long BuzzerUntil = 0;

volatile bool Tick = false; // set every MonitorInterval by MsTimer2

// Restricts reception to the two IDs used. Each register write is tried up to 3 times;
// on failure the bus is flagged and CheckCount() schedules another reset.
bool SetMaskFilt()
{
  struct Reg { bool mask; unsigned char num; unsigned long id; };
  const Reg regs[] = {
    {true, 0, CanIdTemps}, {true, 1, CanIdRpm},
    {false, 0, CanIdTemps}, {false, 1, CanIdTemps}, {false, 2, CanIdTemps},
    {false, 3, CanIdRpm}, {false, 4, CanIdRpm}, {false, 5, CanIdRpm},
  };
  for (unsigned int i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i)
  {
    bool ok = false;
    for (int retry = 0; retry < 3 && !ok; ++retry)
    {
      ok = MCP2515_OK == (regs[i].mask ? CAN.init_Mask(regs[i].num, 0, regs[i].id)
                                       : CAN.init_Filt(regs[i].num, 0, regs[i].id));
    }
    if (!ok)
    {
      IsCANBusError = true;
      DBG("init failed "); DBG(regs[i].mask ? "mask " : "filt "); DBGLN(regs[i].num);
      return false;
    }
  }
  return true;
}

void SetPumpDuty()
{
  PumpInputs in;
  in.forceOn = digitalRead(OnPin) == LOW;
  in.tempValid = IsReadBegan;
  in.rpmReceived = HasRpmReceived;
  in.canError = IsCANBusError;
  in.engineStarted = HasEngineStarted;
  in.engineStopped = IsEngineStopped;
  in.coolant = CurTemp;
  in.oil = OilTemp;
  IndexDuty = PumpComputeIndex(in, Pump);
  DBG1(CurTemp); DBG1(" diff "); DBG1(PumpRate(Pump)); DBG1(" accu "); DBG1(PumpAccumulated(Pump));
  DBG1(" -> "); DBG1LN(IndexDuty);
}

void CheckEngineStop()
{
#ifndef TEST
  bool allZero = true;
  for (int i = 0; i < RpmWindow; ++i)
  {
    if (RpmHistory[i] > 0)
    {
      allZero = false;
      break;
    }
  }
  if (allZero)
  {
    DBGLN("Eng Stop");
    IsEngineStopped =  true;
  }
#endif
}

void SetMotorSpeed()
{
  static int prevIndex = -1;

  if (prevIndex != IndexDuty)
  {
    int duty;
    prevIndex = IndexDuty;
    duty = Duties[IndexDuty];
    DBG1("motor speed: "); DBG1LN(duty);
    motor.setSpeed(duty);
  }
}

void timerFunc()
{
  Tick = true;
}

void SetBusError()
{
  IsCANBusError = true;
  IsReadBegan = false;
}

// '?' over the values that stopped arriving (TempStop/RpmStop from CheckCount())
void MarkCanLost()
{
  if (TempStop)
  {
    lcd.setCursor(OilOffset + 2, OilRow); lcd.print('?');
    lcd.setCursor(CoolantOffset + 2, CoolantRow); lcd.print('?');
  }
  if (RpmStop)
  {
    lcd.setCursor(RpmOffset + 1, RpmRow); lcd.print('?');
  }
}

void RequestCanReset()
{
  CanResetRequested = true;
  SetBusError();
  MarkCanLost();
}

void SetBusSuccess()
{
  IsCANBusError = false;
}

void setup()
{
  // full duty until the controller can decide otherwise (the driver constructor leaves PWM LOW)
  motor.setSpeed(Duties[MaxDutyIndex]);

  // switch input
  pinMode(OnPin, INPUT);
  if (BuzzerPin >= 0)
  {
    pinMode(BuzzerPin, OUTPUT);
    digitalWrite(BuzzerPin, LOW);
  }

  PumpReset(Pump);
  
#ifdef DEBUG
  Serial.begin(115200);
#endif
  LcdInit();

  // Do not block here: if the MCP2515 fails, the control loop still runs (fail-safe 100%,
  // oil pressure monitoring) and CheckCount()/ResetCanIfRequested() retry the init every second.
  if (CAN_OK == CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
  {
    DBGLN("CANBus initialized");
    SetBusSuccess();
  }
  else
  {
    DBGLN("CANBus init failed");
    SetBusError();
  }

  MsTimer2::set(MonitorInterval, timerFunc);
  MsTimer2::start();
}

void loop()
{
  taskCanRecv();
  SampleOilPressure();
  if (Tick)
  {
    Tick = false;
    CheckCount();
    ResetCanIfRequested();
    CheckEngineStop();
    SetPumpDuty();
    SetMotorSpeed();
    Display();
  }
}

// median of the first n entries of buf (n <= OPMedianSize)
float MedianOf(const float *buf, int n)
{
  float t[OPMedianSize];
  for (int i = 0; i < n; ++i) // insertion sort
  {
    float v = buf[i];
    int j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; --j; }
    t[j + 1] = v;
  }
  return t[n / 2];
}

// Reads the sensor and returns the filtered voltage (offset not applied).
float ReadOilPressureVoltage()
{
  long sum = 0;
  for (int i = 0; i < OPOversample; ++i)
  {
    sum += analogRead(OilPressurePin);
  }
  float voltage = sum * (5.0 / 1023.0) / OPOversample;

  OPVoltRaw[OPVoltIdx] = voltage;
  OPVoltIdx = (OPVoltIdx + 1) % OPMedianSize;
  if (OPVoltSamples < 32767) { OPVoltSamples++; }
  int n = (OPVoltSamples < OPMedianSize) ? OPVoltSamples : OPMedianSize;
  float median = MedianOf(OPVoltRaw, n);

  if (OPVoltSamples == 1) { OPVoltEma = median; }
  else { OPVoltEma += OPEmaAlpha * (median - OPVoltEma); }

  DBG4("Pr "); DBG4(voltage); DBG4(" med "); DBG4(median); DBG4(" ema "); DBG4LN(OPVoltEma);
  return OPVoltEma;
}

// Called while RPM == 0 is reported before the engine has started (key on).
// Oil pressure is truly 0 psi then, so the filtered voltage is the sensor's 0 psi point.
void CalibrateOilPressureOffset(float voltage)
{
  if (OPVoltSamples < OPCalibMinSamples) { return; }
  // 0.1 <= voltage < 0.5 : adjust offset (below 0.1 V the sensor is likely disconnected)
  if (0.1 <= voltage && voltage < 0.5)
  {
    VoltOffset = 0.5 - voltage;
  }
}

// filtered sensor voltage (offset not applied) -> psi
int OilPsiFromVoltage(float voltage)
{
  /*       Linear scale
   * Bar  0    2    5    8    10
   * PSI  0    29   72.5 116  145
   * V    0.5  1.3  2.5  3.7  4.5
   */
  float psi = (voltage + VoltOffset - 0.5) * (145 / 4.0);
  if (psi < 0) { psi = 0; }
  return (int)(psi + 0.5); // round
}

// minimum healthy oil pressure for the RPM
int MinOilPsi(int rpm, bool rpmValid)
{
  if (!rpmValid) { return OPNoRpmMinPsi; }
  if (rpm < 1500) { return 6; }
  if (rpm < 3000) { return 15; }
  if (rpm < 4500) { return 25; }
  return 35;
}

/*
 * NORMAL -> DIP : below the minimum for OPDipMs (e.g. oil starvation in a corner), latched in OPDipCount
 * DIP -> LOSS   : still below the minimum after OPLossMs
 * -> NORMAL     : at least minimum + hysteresis for OPRecoverMs
 * SENSOR_FAULT  : sensor voltage out of range, overrides the others
 */
void CheckOilPressure(unsigned long now, bool sensorOk, int psi, bool rpmValid)
{
  unsigned char prev = OPAlarm;

  if (!sensorOk)
  {
    OPAlarm = OP_SENSOR_FAULT;
    OPLowSince = 0; OPOkSince = 0;
  }
  else
  {
    if (OPAlarm == OP_SENSOR_FAULT) { OPAlarm = OP_NORMAL; }

    // without RPM (CAN lost) assume running once the engine has started
    bool running = rpmValid ? (CurRPM >= OPMonitorMinRpm) : HasEngineStarted;
    if (!running)
    {
      OPRunningSince = 0; OPLowSince = 0; OPOkSince = 0;
      OPAlarm = OP_NORMAL;
    }
    else
    {
      if (OPRunningSince == 0) { OPRunningSince = now; }
      if (now - OPRunningSince >= OPStartDelayMs)
      {
        int minPsi = MinOilPsi(CurRPM, rpmValid);
        if (psi < minPsi)
        {
          OPOkSince = 0;
          if (OPLowSince == 0) { OPLowSince = now; }
          unsigned long low = now - OPLowSince;
          if (low >= OPLossMs) { OPAlarm = OP_LOSS; }
          else if (low >= OPDipMs && OPAlarm == OP_NORMAL)
          {
            OPAlarm = OP_DIP;
            OPDipCount++;
          }
        }
        else
        {
          OPLowSince = 0;
          if (psi >= minPsi + OPHysteresisPsi)
          {
            if (OPOkSince == 0) { OPOkSince = now; }
            if (now - OPOkSince >= OPRecoverMs) { OPAlarm = OP_NORMAL; }
          }
          else
          {
            OPOkSince = 0;
          }
        }
      }
    }
  }

  if (OPAlarm != prev)
  {
    if (OPAlarm == OP_DIP) { BuzzerUntil = now + 200; }               // one beep
    else if (OPAlarm == OP_SENSOR_FAULT) { BuzzerUntil = now + 100; } // short beep
    DBG4("OP alarm "); DBG4(prev); DBG4("->"); DBG4(OPAlarm); DBG4(" psi "); DBG4(psi);
    DBG4(" rpm "); DBG4(rpmValid ? CurRPM : -1); DBG4(" dips "); DBG4LN(OPDipCount);
  }
}

void UpdateBuzzer(unsigned long now)
{
  if (BuzzerPin < 0) { return; }
  bool on;
  if (OPAlarm == OP_LOSS) { on = ((now / 250) & 1) == 0; } // 2 Hz beeping
  else { on = (long)(BuzzerUntil - now) > 0; }
  digitalWrite(BuzzerPin, on ? HIGH : LOW);
}

// Called from loop(); samples every OPSampleMs independent of CAN reception
// so that oil pressure is still monitored when the CAN bus is lost.
void SampleOilPressure()
{
  unsigned long now = millis();
  if (now - LastOPSampleMs < OPSampleMs) { return; }
  LastOPSampleMs = now;

  bool rpmValid = HasRpmReceived && (now - LastRpmMs) < RpmStaleMs;
  float voltage = ReadOilPressureVoltage();

  if (rpmValid && CurRPM == 0 && !HasEngineStarted)
  {
    // key on, engine not yet started: learn the 0 psi sensor offset
    CalibrateOilPressureOffset(voltage);
  }

  bool sensorOk = (OPSensorMinVolt <= voltage && voltage <= OPSensorMaxVolt);
  int psi = OilPsiFromVoltage(voltage);
  if (sensorOk && rpmValid && CurRPM > 0)
  {
    AddToOPLog(CurRPM, psi);
  }
  CheckOilPressure(now, sensorOk, psi, rpmValid);
  UpdateBuzzer(now);
}

void AddToOPLog(int rpm, int psi)
{
  OilPressureLogs[OPLogTail].rpm = rpm;
  OilPressureLogs[OPLogTail].psi = psi;
  OPLogTail = (OPLogTail + 1) % OPLogSize;
  if (OPLogTail == OPLogHead)
  {
    OPLogHead = (OPLogHead + 1) % OPLogSize;
  }
}

int GetCurrentOP()
{
  if (OPLogHead == OPLogTail) { return -1; }
  return OilPressureLogs[(OPLogTail + OPLogSize - 1) % OPLogSize].psi;
}

// max rpm in the log and the max psi within OPLogRpmRange of it -> OPToDisp
void GetMaxRecentOPLog()
{
  int maxRpm = 0;
  int psiAtMaxRpm = 0;
  for (int i = OPLogHead; i != OPLogTail; i = (i + 1) % OPLogSize)
  {
    if (maxRpm < OilPressureLogs[i].rpm) { maxRpm = OilPressureLogs[i].rpm; }
  }
  for (int i = OPLogHead; i != OPLogTail; i = (i + 1) % OPLogSize)
  {
    if (maxRpm - OPLogRpmRange < OilPressureLogs[i].rpm && psiAtMaxRpm < OilPressureLogs[i].psi)
    {
      psiAtMaxRpm = OilPressureLogs[i].psi;
    }
  }
  OPToDisp.rpm = maxRpm;
  OPToDisp.psi = psiAtMaxRpm;
}

void CheckCount()
{
  IntervalCount++;
  if (IntervalCount >= BeatsPerSecond) // 1 seconds
  {
    IntervalCount = 0;
    RpmStop = (PrevRpmCnt == RpmCnt) ? true : false;
    TempStop = (PrevTempCnt == TempCnt) ? true : false;
    PrevRpmCnt = RpmCnt;
    PrevTempCnt = TempCnt;
    if (RpmStop || TempStop)
    {
      RequestCanReset();
    }
    else
    {
      SetBusSuccess();
    }
  }
}

// Re-initializes the MCP2515 after CheckCount() saw no packets for a second.
// Never blocks, so oil pressure sampling and the pump control keep running.
void ResetCanIfRequested()
{
#ifdef TEST
  static unsigned char dir = 0;
  static char step = 0;
#endif
  if (CanResetRequested)
  {
    CanResetRequested = false;
#ifdef TEST
    if (dir == 0)
      CurTemp += 5;
    else if (dir == 1)
    {
      if (CurTemp <= TargetTemp)
        dir = 2;
      else
      {
        step++;
        if (step >= 2)
        {
          step = 0;
          CurTemp -= 1;
        }
      }
    }
    else if (dir == 2)
    {
      if (CurTemp <= HighTemp)
      {
        step++;
        if (step >= 2)
        {
          step = 0;
          CurTemp++;
        }
      }
      else
        dir = 1;
    }
    if (CurTemp > HighTemp)
      dir = 1;
    else if (CurTemp < -20)
      dir = 0;
    DBG("dir = "); DBGLN(dir);
#else
    SetBusError();
    DBGLN("CANBus error!");

    CANBusResetCount++;
    if (CAN_OK == CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
      DBGLN("CANBus initialized");
      SetMaskFilt();
    }
#endif
  }
}

void taskCanRecv()
{
  unsigned char len = 0;
  unsigned char buf[MAX_CHAR_IN_MESSAGE];
  unsigned long canId;

  if(CAN_MSGAVAIL == CAN.checkReceive())                   // check if get data
  {
    CAN.readMsgBuf(&len, buf); // read data,  len: data length, buf: data buf
    canId = CAN.getCanId();
    DBG2H(canId); DBG2(","); DBG2(len); DBG2(":");
    DBG2H(buf[0]); DBG2(" "); DBG2H(buf[1]); DBG2(" "); DBG2H(buf[2]); DBG2(" "); DBG2H(buf[3]); DBG2LN("");
    if (canId == CanIdTemps && len >= 4)
    {
      TempCnt++;
      // 2 byte OilTemp, 3 byte CoolantTemp
      CurTemp = buf[3] - CanTempOffset;
      OilTemp = buf[2] - CanTempOffset;
      DBG("Water "); DBG(CurTemp); DBG(" Oil "); DBGLN(OilTemp);
      if (!IsReadBegan)
      {
        IsReadBegan = true;
        DBGLN("Read began!");
      }
    }
    else if (canId == CanIdRpm && len >= 4)
    {
      RpmCnt++;
      HasRpmReceived = true;
      LastRpmMs = millis();
      /*
      Get data from ID: 140
      0  4 0 40  0 0 60  1 
      */
      // 0 byte AccelPedalPositionA, 1 byte Flags (8 bit clutch), 2 long RPM, 4 byte AccelPedalPositionB, 5 byte AccelPedalPositionC, ?
      CurRPM = (((buf[3] & 0x3f) << 8) + buf[2]);
      RpmHistory[RpmIndex] = CurRPM;
      RpmIndex = (RpmIndex + 1) % RpmWindow;
      if (CurRPM > 0)
      {
        IsEngineStopped = false;
        HasEngineStarted = true;
        DBGLN("Engine running");
      }
      DBG("RPM "); DBGLN(CurRPM);
    }
  }
}

void LcdInit()
{
  lcd.begin(16, 2);
  for (int i = 0; i < 5; ++i)
  {
    uint8_t glyph[8];
    for (int r = 0; r < 8; ++r) { glyph[r] = InvGlyphSrc[i][r] ^ 0x1F; }
    lcd.createChar(i, glyph);
  }
  lcd.setCursor(0, 0);
  lcd.print("---  O--- ");
  lcd.print("W---");
  lcd.print((char)223);
  lcd.print("C ");
  lcd.setCursor(0, 1);
  lcd.print("---psi@-- P    %");
}

void Display()
{
  const char dig[MaxDutyIndex + 1][5] = {"   0", "1.56", "3.13", "6.25", "12.5", "  25", "37.5", "  50", "62.5", "  75", "87.5", " 100"};
  bool perSecond = false;
  //           0123456789012345
  //row 0:    "---  O--- W---°C"
  //row 1:    "---psi@-- P    %"

  if (++LcdUpdateCount >= BeatsPerSecond)
  {
    perSecond = true;
    LcdUpdateCount = 0;
    lcd.setCursor(HeartbeatOffset, HeartbeatRow);
    if (++Heartbeat & 1) { lcd.print("p"); }
    else { lcd.print("P"); }
  }

#ifndef TEST
  if (IsCANBusError || !IsReadBegan)
  {
    MarkCanLost();
    if (IsCANBusError)
    {
      const char spinner[4] = {'-', '/', '-', '\\'};
      lcd.setCursor(ResetOffset, ResetRow);
      lcd.print(spinner[CANBusResetCount % 4]);
    }
  }
  else
#endif
  {
    lcd.setCursor(ResetOffset, ResetRow); lcd.print(' ');

    if (perSecond)
    {
      DispOilPressure();
    }
    if (PrevTemp != CurTemp)
    {
      PrevTemp = CurTemp;
      DispTemp(PrevTemp, CoolantOffset, CoolantRow);
    }
    if (PrevOil != OilTemp)
    {
      PrevOil = OilTemp;
      DispTemp(PrevOil, OilOffset, OilRow);
    }
  }

  // shown regardless of CAN state; oil pressure is sampled independently
  DispOilWarning();

  if (PrevDuty != IndexDuty)
  {
    PrevDuty = IndexDuty;
    lcd.setCursor(PumpOffset, PumpRow);
    if (PrevDuty < 0 || MaxDutyIndex < PrevDuty)
    {
      lcd.print("err");
    }
    else
    {
      DBG1("idx "); DBG1(PrevDuty); DBG1(" duty "); DBG1LN(dig[PrevDuty]);
      lcd.print(dig[PrevDuty]);
    }
  }
}

void DispPsi(int psi)
{
  if (psi < 0) { psi = 0; }
  if (psi < 10) { lcd.print("  "); }
  else if (psi < 100) { lcd.print(" "); }
  lcd.print(psi);
}

void DispOilPressure()
{
  GetMaxRecentOPLog();
  int rpm = OPToDisp.rpm / 100; // only two digits
  int localMax = OPToDisp.psi;
  int cur = GetCurrentOP();
  DBG4("Pr cur "); DBG4(cur); DBG4(" max "); DBG4(localMax); DBG4(" @ "); DBG4LN(rpm); 
  //           0123456789012345
  //lcd.print("---psi@-- P    %");
  // current oil pressure
  lcd.setCursor(PressureOffset, PressureNowRow);
  if (OPAlarm == OP_SENSOR_FAULT) { lcd.print("---"); }
  else { DispPsi(cur); }
  lcd.setCursor(PressureOffset + 3, PressureNowRow);
  if (VoltOffset == 0.0) { lcd.print(' '); }
  else if (VoltOffset < 0.1) { lcd.print('_'); }
  else if (VoltOffset < 0.3) { lcd.print('+'); }
  else if (VoltOffset < 0.5) { lcd.print('^'); }
  else { lcd.print('?'); }
  // row 1 is taken by the warning message
  if (OPAlarm == OP_LOSS || OPAlarm == OP_SENSOR_FAULT) { return; }
  // maximum in the log (OPLogSize samples, ~7.5 seconds)
  lcd.setCursor(PressureOffset, PressureRow);
  DispPsi(localMax);
  // RPM at max pressure
  lcd.setCursor(RpmOffset, RpmRow);
  if (rpm < 10) { lcd.print(' '); }
  lcd.print(rpm);
}

void PrintOilLow(bool inverse)
{
  const char text[] = "OIL LOW!";
  const unsigned char inv[8] = {InvO, InvI, InvL, InvSpace, InvL, InvO, InvW, InvExcl};
  lcd.setCursor(PressureOffset, PressureRow);
  for (int i = 0; i < 8; ++i)
  {
    lcd.write(inverse ? inv[i] : (uint8_t)text[i]);
  }
}

/*
 * row 0 col 4 : ' ' no dip, '!' dip happened since key on, inverted '!' dip/loss now
 * row 1 col 0-7 : LOSS -> "OIL LOW!" blinking normal/inverted, SENSOR_FAULT -> "OP SENS?"
 * Called every Display() tick (100 ms).
 */
void DispOilWarning()
{
  static unsigned char shown = OP_NORMAL;
  static int shownPhase = -1;
  static char shownMark = ' ';

  char mark = ' ';
  if (OPDipCount > 0)
  {
    mark = (OPAlarm == OP_DIP || OPAlarm == OP_LOSS) ? 'i' : '!';
  }
  if (mark != shownMark)
  {
    shownMark = mark;
    lcd.setCursor(DipMarkOffset, DipMarkRow);
    lcd.write(mark == 'i' ? InvExcl : (uint8_t)mark);
  }

  if (OPAlarm == OP_LOSS)
  {
    int phase = (millis() / 500) & 1;
    if (shown != OP_LOSS)
    {
      shownPhase = -1;
      lcd.setCursor(RpmOffset + 1, RpmRow); lcd.print(' '); // leave col 8 for the CAN '?' mark
    }
    if (phase != shownPhase)
    {
      shownPhase = phase;
      PrintOilLow(phase == 0);
    }
  }
  else if (OPAlarm == OP_SENSOR_FAULT)
  {
    if (shown != OP_SENSOR_FAULT)
    {
      lcd.setCursor(PressureOffset, PressureRow); lcd.print("OP SENS? ");
    }
  }
  else if (shown == OP_LOSS || shown == OP_SENSOR_FAULT)
  {
    // restore the template; DispOilPressure() fills the digits
    lcd.setCursor(PressureOffset, PressureRow); lcd.print("---psi@--");
  }
  shown = OPAlarm;
}

void DispTemp(int temp, int col, int row)
{
  lcd.setCursor(col, row);
  if (temp <= -10) { /*-10*/ }
  else if (temp < 0) { /* -1*/ lcd.print(" "); }
  else if (temp < 10) { /*  9*/ lcd.print("  "); }
  else if (temp < 100) { /* 99*/ lcd.print(" "); }
  else { /*100*/ }
  lcd.print(temp);
}

// END FILE
