#include <SimpleKalmanFilter.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>
#include <CytronMotorDriver.h>
#include <MsTimer2.h>
#include <Wire.h>
#include <rgb_lcd.h>

//#define NO_CAN
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
const int OnPin = 12;
const int OilPressurePin = A0;
const int MonitorInterval = 100; // ms
const int BeatsPerSecond = 1000 / MonitorInterval;
const int MonitorWindow = 10;
const int EmptyThreshold = BeatsPerSecond;
const int MaxDutyIndex = 11;
const int Duties[12] = {0, 4, 8, 16, 32, 64, 96, 128, 160, 192, 224, 255};
const int WarmUpTemp = 80;
const int LowTemp = 88;
const int TargetTemp = 92;
const int MidTemp = 95;
const int HighTemp = 98;
const int MaxTemp = 99;
// lcd row 0
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

rgb_lcd lcd;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

// Configure the motor driver.
CytronMD motor(PWM_DIR, 9, 4);  // PWM = Pin 9, DIR = Pin 4.
/*
 SimpleKalmanFilter(e_mea, e_est, q);
 e_mea: Measurement Uncertainty 
 e_est: Estimation Uncertainty 
 q: Process Noise
 */
SimpleKalmanFilter PressureKalmanFilter(1, 1, 0.01);

void timerFunc();

int EmptyCnt = 0;
int LcdUpdateCount = 0;
int Diff1sec[16];
int Accu1sec[16];
int TempIndex = 0;
#ifdef TEST
int Temp1sec[16] = {70,70,70,70,70,70,70,70,70,70,};
int CurTemp = 70; //test
#else
int Temp1sec[16];
int CurTemp;
#endif
int RpmHistory[16] = {0, };
int RpmIndex = 0;
int CurRPM = 0;
int CurVolt;
int OilTemp;
bool IsEngineStopped = true;
bool HasEngineStarted = false;
bool HasRpmReceived = false;
bool IsWarmedUp = false;
bool IsHighReached = false;
bool IsOilHot = false;
bool IsCANBusError = false;
bool IsReadBegan = false;
volatile bool ForceCanTimer = false;
volatile int TimerCount = 0;
int WarmUpCnt = 0;
int IndexDuty = MaxDutyIndex;
unsigned int CANBusResetCount = 0;
char MaskSet = '.';
char FilterSet = '.';

int PrevTemp = -40;
int PrevOil = -40;
int PrevDuty = -1;
int PrevVolt = -1;
int PrevRpm = -1;
unsigned int PrevRpmCnt = 0;
unsigned int PrevTempCnt = 0;
unsigned int RpmCnt = 0;
unsigned int TempCnt = 0;
unsigned int Heartbeat = 0;

volatile unsigned char flagRecv = 0;
volatile unsigned char checkBus = 0;
volatile unsigned char setPwm = 0;
volatile unsigned char dispLcd = 0;

void init_arrays()
{
  int i;
  for (i = 0; i < 16; ++i)
  {
    Diff1sec[i] = 0;
#ifndef TEST
    Temp1sec[i] = 0;
#endif
    RpmHistory[i] = 0;
  }
}

void init_vars()
{
  EmptyCnt = 0;
  TempIndex = 0;
  RpmIndex = 0;
  CurRPM = 0;
  IsEngineStopped = true;
  HasEngineStarted = false;
  HasRpmReceived = false;
  IsWarmedUp = false;
  IsHighReached = false;
  IsOilHot = false;
  IsCANBusError = false;
  IsReadBegan = false;
  ForceCanTimer = false;
  TimerCount = 0;
  WarmUpCnt = 0;
  IndexDuty = MaxDutyIndex;
  CANBusResetCount = 0;
  MaskSet = '.';
  FilterSet = '.';

  PrevTemp = -40;
  PrevOil = -40;
  PrevDuty = -1;
  PrevVolt = -1;
  PrevRpm = -1;
  Heartbeat = 0;
}

void init_flags()
{
  flagRecv = 0;
  checkBus = 0;
  setPwm = 0;
  dispLcd = 0;  
}

#define TRY(a, l)  \
do {\
  int retry = 0;\
  while (MCP2515_OK != (a))\
  {\
    retry++;\
    if (retry > 2)\
    {\
      IsCANBusError = true; MaskSet = '.'; FilterSet = '.';\
      DBGLN("init failed " #a " " #l);\
      return;\
    }\
  }\
} while(0)

void set_mask_filt()
{
  /*
  * set mask
  */
  TRY(CAN.init_Mask(0, 0, 0x360), __LINE__);
  MaskSet = 'a';
  TRY(CAN.init_Mask(1, 0, 0x140), __LINE__);
  MaskSet = 'b';
  
  /*
  * set filter
  */
  TRY(CAN.init_Filt(0, 0, 0x360), __LINE__);
  FilterSet = '0';
  TRY(CAN.init_Filt(1, 0, 0x360), __LINE__);
  FilterSet = '1';
  TRY(CAN.init_Filt(2, 0, 0x360), __LINE__);
  FilterSet = '2';
  TRY(CAN.init_Filt(3, 0, 0x140), __LINE__);
  FilterSet = '3';
  TRY(CAN.init_Filt(4, 0, 0x140), __LINE__);
  FilterSet = '4';
  TRY(CAN.init_Filt(5, 0, 0x140), __LINE__);
  FilterSet = '5';
}

int SumOfDifferentials()
{
  const unsigned char idx[21] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0,};
  int i;
  int sum = 0;
  bool monoInc = true;
  bool monoDec = true;
  for (i = 0; i < 10; ++i)
  {
    int d = Diff1sec[idx[TempIndex + i]];
    if (d < 0) monoInc = false;
    else if (d > 0) monoDec = false;
    sum += d;
  }
  DBG1(" ("); DBG1(monoInc); DBG1(monoDec); DBG1(" delta "); DBG1(sum); DBG1(") ");
  if (monoInc || monoDec)
  {
    return sum;
  }
  return 0;
}

int SumOfDelta()
{
  const unsigned char idx[21] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0,};
  int i;
  int accu = 0;
  for (i = 0; i < 10; ++i)
  {
    int d = Accu1sec[idx[TempIndex + i]];
    accu += d;
  }
  DBG1("("); DBG1("accu "); DBG1(accu); DBG1(") ");
  return accu;
}

int Trim16(int value)
{
  return (value < 0) ? 0 : ((value > 15) ? 15: value);
}

void SetPumpDuty()
{
  const unsigned char nextIdx[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6};
  const unsigned char prevIdx[16] = {9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4};
  int diverge;
  int defaultDutyIdx;
  bool pumpOn = digitalRead(OnPin) == LOW;
  /*
   * IsCANBusError IsReadBegan HasEngineStarted
   *       F           F             F         while ECU reflashing possibly
   *       T           F             F         while ECU reflashing possibly
   *       F           T             F         before cranking (key on); treat as normal to check pump
   *       T           T             F         hardly
   *       F           F             T         error; after this water pump controller reset, temp not yet received
   *       T           F             T         error
   *       F           T             T         normal
   *       T           T             T         hardly
   */
  if (pumpOn || (!IsReadBegan && HasEngineStarted) || (!HasEngineStarted && !HasRpmReceived))
  {
    if (IndexDuty < MaxDutyIndex)
      IndexDuty += 2;
    if (IndexDuty <= 0 || MaxDutyIndex < IndexDuty)
      IndexDuty = MaxDutyIndex;
    DBG("Error ");
    DBGLN(IndexDuty);
  }
  // there is a possibility that RPM packets are not received for a while after engine started.
  else if ((!IsReadBegan || (IsReadBegan && CurTemp < HighTemp)) && (!HasEngineStarted && HasRpmReceived))
  {
    IndexDuty = 0;
    DBG("Key On");
    DBGLN(IndexDuty);
  }
  else
  {
    const unsigned char log10val[16] = {0, 3, 4, 5, 6, 7, 8, 8, 
                                        8, 8, 8, 8, 8, 8, 8, 8};
    int diff;
    int accu;
#ifdef TEST
    {
      int i;
      for (i = 0; i < MonitorWindow; i++) { DBG1(Temp1sec[i]); DBG1(" "); } DBG1LN(TempIndex);
      for (i = 0; i < MonitorWindow; i++) { DBG1(Diff1sec[i]); DBG1(" "); } DBG1LN(TempIndex);
    }
#endif
    if (TempIndex < 0 || 10 <= TempIndex)
    {
      TempIndex = 0;
    }
    Temp1sec[TempIndex] = CurTemp;
    Diff1sec[TempIndex] = CurTemp - Temp1sec[prevIdx[TempIndex]];
    diverge = CurTemp - TargetTemp;
    Accu1sec[TempIndex] = diverge;
    diverge = Trim16(diverge);
    defaultDutyIdx = 4 + log10val[diverge];
    DBG1(TempIndex); DBG1(", "); DBG1(CurTemp); DBG1(" D"); DBG1(diverge); DBG1(", ");
    TempIndex = nextIdx[TempIndex];

    DBG1(defaultDutyIdx); DBG1("->");
    diff = SumOfDifferentials();
    accu = SumOfDelta();

    if (CurTemp >= HighTemp)
      IsHighReached = true;
    else if (CurTemp < MidTemp && diff <= 0)
      IsHighReached = false; 

    if (OilTemp >= 103)
      IsOilHot = true;
    else if (OilTemp <= 100)
    {
      IsOilHot = false;
    }

    if (CurTemp >= WarmUpTemp)
      IsWarmedUp = true;

    if (CurTemp < LowTemp) // Low=88, WarmUp=80
    {
      if (IsReadBegan && !IsEngineStopped)
      {
        WarmUpCnt++;
        // try to pulse 2 second per 10 seconds
        // 1..40 -> 1 to warm up coolant
        // 41..50 -> 5 to circulate coolant
        if (WarmUpCnt > 40)
        {
          IndexDuty = 5; // 25%
          if (WarmUpCnt >= 50)
            WarmUpCnt = 0;
        }
        else
        {
          IndexDuty = 1; // 1.25%
        }
      }
    }
    else if (CurTemp >= MaxTemp || IsOilHot || IsHighReached)
    {
      IndexDuty = MaxDutyIndex;
    }
    else
    {
      int delta;
      int bonus = Trim16(CurTemp - (TargetTemp - 2));

      if (CurTemp < TargetTemp)
      {
        if (diff < 0) delta = -3;
        else if (diff == 0) delta = (accu > -10) ? log10val[bonus] : (bonus == 0 ? -2 : -1);
        else delta = (accu > -20) ? log10val[bonus] : 0;
      }
      else if (CurTemp == TargetTemp)
      {
        if (diff < 0) delta = 0;
        else if (diff == 0) delta = (accu > 0) ? 2 : 1;
        else delta = 4;
      }
      else
      {
        if (diff < 0) delta = diverge;
        else if(diff == 0) delta = log10val[diverge];
        else delta = log10val[bonus];
      }

      if (delta < -3) delta = -3; // to avoid IndexDuty == 0 case

      IndexDuty = defaultDutyIdx + delta;
    }
   
    if (OilTemp >= 98)
    {
      IndexDuty += OilTemp - 98;
    }

    if (IndexDuty <= 0 || IndexDuty > MaxDutyIndex)
      IndexDuty = MaxDutyIndex;

    DBG1LN(IndexDuty);
  }
}

void CheckEngineStop()
{
#ifndef TEST
  bool allZero = true;
  for (int i = 0; i < 8; ++i)
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
  setPwm = 1;
  dispLcd = 1;
  checkBus = 1;
  flagRecv = 1;
}

/*void MCP2515_ISR()
{
  flagRecv = 1;
}*/

void SetBusError(char c, int line)
{
  IsCANBusError = true; MaskSet = '.'; FilterSet = '.';
  IsReadBegan = false;
}

void SetBusErrorRetry(char c, int line)
{
  EmptyCnt = EmptyThreshold;
  IsCANBusError = true; MaskSet = '.'; FilterSet = '.';
  IsReadBegan = false;

  if (c == 't' || c == 'x')
  {
    lcd.setCursor(OilOffset + 2, OilRow); lcd.print('?');
    lcd.setCursor(CoolantOffset + 2, CoolantRow); lcd.print('?');
  }
  if (c == 'r' || c == 'x')
  {
    lcd.setCursor(RpmOffset + 1, RpmRow); lcd.print('?');
  }
}

void SetBusSuccess(char c, int line)
{
  IsCANBusError = false;
}

/*
* Interrupt
* PIN 3 INT 0 SCL OC0B(Timer 0)
* PIN 2 INT 1 SDA
* PIN 1 INT 3 TXD1
* PIN 0 INT 2 RXD1
* PIN 7 INT 4
* attachInterrupt(digitalPinToInterrupt(<pin num>), ISR, FALLING)
 */
void setup()
{
  // switch input
  pinMode(OnPin, INPUT);
  
  init_arrays();
  init_vars();
  init_flags();
  
#ifdef DEBUG
  Serial.begin(115200);
#endif
  LcdInit();

  //attachInterrupt(digitalPinToInterrupt(7), MCP2515_ISR, FALLING); // start interrupt

  while (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
  {
    SetBusError('E', __LINE__);
    delay(100);
  }
  DBGLN("CANBus initialized");
  SetBusSuccess('S', __LINE__);

  MsTimer2::set(MonitorInterval, timerFunc);
  MsTimer2::start();
}

void loop()
{
  taskCanRecv();
  if (flagRecv)
  {
    flagRecv = 0;
    CheckCount();
    CheckEmpty();
    CheckEngineStop();
  }

  if (setPwm)
  {
    setPwm = 0;
    SetPumpDuty();
    SetMotorSpeed();
  }
  if (dispLcd)
  {
    dispLcd = 0;
    Display();
  }
}

int ReadOilPressureSensor()
{
  int value = analogRead(OilPressurePin);
  float voltage = value * (5.0 / 1023.0);
  /*       Linear scale
   * Bar  0    2    5    8    10
   * PSI  0    29   72.5 116  145
   * V    0.5  1.3  2.5  3.7  4.5
   */
  float psi = (voltage - 0.5) * (145 / 4.0);

  if (psi < 0)
    return (int)(psi + 0.5); // round

  float estimatedPsi = PressureKalmanFilter.updateEstimate(psi);
  DBG4("Pr "); DBG4(value); DBG4(" "); DBG4(voltage); DBG4(" V ");
  DBG4(psi); DBG4(" "); DBG4(estimatedPsi); DBG4LN(" psi");
  return (int)(estimatedPsi + 0.5); // round
}

const int OPLogSize = 150; // 5Hz 30sec maybe
struct PressureLogT
{
  int rpm;
  int psi;
} OilPressureLogs[OPLogSize];
int OPLogHead = 0;
int OPLogTail = 0;
struct PressureLogT OPToDisp;

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

// read per RPM packet, so 20Hz
void LogRpmAndOilPressure()
{
  int psi = ReadOilPressureSensor();
  int rpm = CurRPM;
  if (psi < 0) return;
  AddToOPLog(rpm, psi);
}

int GetCurrentOP()
{
  if (OPLogHead == OPLogTail)
  {
    return -1;
  }
  
  return OilPressureLogs[(OPLogTail + OPLogSize - 1) % OPLogSize].psi;
}

int GetMaxRpm()
{
  int maxRpm = 0;
  if (OPLogHead < OPLogTail)
  {
    for (int i = OPLogHead; i < OPLogTail; ++i)
    {
      if (maxRpm < OilPressureLogs[i].rpm)
      {
        maxRpm = OilPressureLogs[i].rpm;
      }
    }
  }
  else if (OPLogTail < OPLogHead)
  {
    for (int i = OPLogHead; i < OPLogSize; ++i)
    {
      if (maxRpm < OilPressureLogs[i].rpm)
      {
        maxRpm = OilPressureLogs[i].rpm;
      }
    }
    for (int i = 0; i < OPLogTail; ++i)
    {
      if (maxRpm < OilPressureLogs[i].rpm)
      {
        maxRpm = OilPressureLogs[i].rpm;
      }
    }
  }
  return maxRpm;
}

void GetMaxRecentOPLog()
{
  const int range = 50;
  int maxRpm = GetMaxRpm();
  int psiAtMaxRpm = 0;
  if (OPLogHead < OPLogTail)
  {
    for (int i = OPLogHead; i < OPLogTail; ++i)
    {
      if (maxRpm - range < OilPressureLogs[i].rpm && psiAtMaxRpm < OilPressureLogs[i].psi)
      {
        psiAtMaxRpm = OilPressureLogs[i].psi;
      }
    }
  }
  else if (OPLogTail < OPLogHead)
  {
    for (int i = OPLogHead; i < OPLogSize; ++i)
    {
      if (maxRpm - range < OilPressureLogs[i].rpm && psiAtMaxRpm < OilPressureLogs[i].psi)
      {
        psiAtMaxRpm = OilPressureLogs[i].psi;
      }
    }
    for (int i = 0; i < OPLogTail; ++i)
    {
      if (maxRpm - range < OilPressureLogs[i].rpm && psiAtMaxRpm < OilPressureLogs[i].psi)
      {
        psiAtMaxRpm = OilPressureLogs[i].psi;
      }
    }
  }
  else
  {
    DBG4LN("maxOP h==t");
  }
  OPToDisp.rpm = maxRpm;
  OPToDisp.psi = psiAtMaxRpm;
}

int IntervalCount = 0;
bool RpmStop = false;
bool TempStop = false;

void CheckCount()
{
  IntervalCount++;
  if (IntervalCount >= BeatsPerSecond) // 1 seconds
  {
    char c = 'x';
    IntervalCount = 0;
    RpmStop = (PrevRpmCnt == RpmCnt) ? true : false;
    TempStop = (PrevTempCnt == TempCnt) ? true : false;
    PrevRpmCnt = RpmCnt;
    PrevTempCnt = TempCnt;
    if (!RpmStop) c = 't'; 
    if (!TempStop) c = 'r';
    if (RpmStop || TempStop)
    {
      SetBusErrorRetry(c, __LINE__);
    }
    else
    {
      SetBusSuccess('s', __LINE__);
    }
  }
}

void CheckEmpty()
{
#ifdef TEST
  static unsigned char dir = 0;
  static char step = 0;
#endif
  if (EmptyCnt >= EmptyThreshold)
  {
    EmptyCnt = 0;
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
    SetBusError('R', __LINE__);
    DBGLN("CANBus error!");

    CANBusResetCount++;
    if (CAN_OK != CAN.begin(CAN_500KBPS))    // init can bus : baudrate = 500k
    {
      SetBusError('e', __LINE__);
      delay(100);
    }
    else
    {
      DBGLN("CANBus initialized");
      set_mask_filt();
    }
#endif
  }
}

void taskCanRecv()
{
  unsigned char len = 0;
  unsigned char buf[128];
  unsigned long canId;
  unsigned char svc;
  unsigned char pid;

  if(CAN_MSGAVAIL == CAN.checkReceive())                   // check if get data
  {
    CAN.readMsgBuf(&len, buf); // read data,  len: data length, buf: data buf
    canId = CAN.getCanId();
    DBG2H(canId); DBG2(","); DBG2(len); DBG2(":");
    DBG2H(buf[0]); DBG2(" "); DBG2H(buf[1]); DBG2(" "); DBG2H(buf[2]); DBG2(" "); DBG2H(buf[3]); DBG2LN("");
    if (canId == 0x360)
    {
      TempCnt++;
      // 2 byte OilTemp, 3 byte CoolantTemp
      CurTemp = buf[3] - 40;
      OilTemp = buf[2] - 40;
      DBG("Water "); DBG(CurTemp); DBG(" Oil "); DBGLN(OilTemp);
      if (!IsReadBegan)
      {
        IsReadBegan = true;
        DBGLN("Read began!");
      }
    }
    else if (canId == 0x140)
    {
      RpmCnt++;
      HasRpmReceived = true;
      if (RpmIndex < 0 || 10 <= RpmIndex)
      {
        RpmIndex = 0;
      }
      if (0 <= RpmIndex && RpmIndex < 10)
      {
        RpmHistory[RpmIndex] = CurRPM;
      }
      if (++RpmIndex >= 10)
      {
        RpmIndex = 0;
      }
      /*
      Get data from ID: 140
      0  4 0 40  0 0 60  1 
      */
      // 0 byte AccelPedalPositionA, 1 byte Flags (8 bit clutch), 2 long RPM, 4 byte AccelPedalPositionB, 5 byte AccelPedalPositionC, ?
      CurRPM = (((buf[3] & 0x3f) << 8) + buf[2]);
      if (CurRPM > 0)
      {
        IsEngineStopped = false;
        HasEngineStarted = true;
        DBGLN("Engine running");
        LogRpmAndOilPressure();
      }
      DBG("RPM "); DBGLN(CurRPM);
    }
  }
}

void LcdInit()
{
  lcd.begin(16, 2);
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
  const char dig[16][8] = {"   0", "1.56", "3.13", "6.25", "12.5", "  25", "37.5","  50", "62.5", "  75", "87.5", " 100"};
  bool perSecond = false;
  //           0123456789012345
  //lcd.print("--.-V Pump     %");
  //lcd.print("O    °C Wt    °C");
  //lcd.print("R ---- Pmp     %");
  //lcd.print("---  O--- W---°C");
  //lcd.print("---psi@-- P    %");

  if (++LcdUpdateCount >= BeatsPerSecond)
  {
    perSecond = true;
    LcdUpdateCount = 0;
    lcd.setCursor(HeartbeatOffset, HeartbeatRow);
    if (++Heartbeat & 1)
    {
      lcd.print("p");
    }
    else
    {
      lcd.print("P");
    }
  }

#ifndef TEST
  if (IsCANBusError || !IsReadBegan)
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
    /*PrevTemp = -40;
    PrevOil = -40;
    PrevDuty = -1;
    PrevVolt = -1;
    PrevRpm = -1;*/
    if (!IsCANBusError && !IsReadBegan)
    {
      /*lcd.setCursor(3, 0);
      lcd.print(MaskSet);
      lcd.setCursor(4, 0);
      lcd.print(FilterSet);*/
    }
    else if (IsCANBusError)
    {
      /*lcd.setCursor(OilOffset, OilRow);
      lcd.print("---");
      lcd.setCursor(CoolantOffset, CoolantRow);
      lcd.print("---");*/
      lcd.setCursor(ResetOffset, ResetRow);
      int pos = CANBusResetCount % 4;
      switch (pos)
      {
        case 0: lcd.print("-"); break;
        case 1: lcd.print("/"); break;
        case 2: lcd.print("-"); break;
        case 3: lcd.print("\\"); break;
        default: break;
      }
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
    /*if (PrevRpm != CurRPM)
    {
      PrevRpm = CurRPM;
      DispRpm(PrevRpm, RpmOffset, 1);
    }*/
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
    /*if (PrevVolt != CurVolt)
    {
      PrevVolt = CurVolt;
      DispVolt(PrevVolt, 0, 1);
    }*/
  }

  if (PrevDuty != IndexDuty)
  {
    PrevDuty = IndexDuty;
    DBG1("idx "); DBG1(PrevDuty); DBG1(" duty "); DBG1LN(dig[PrevDuty]);
    lcd.setCursor(PumpOffset, 1);
    if (PrevDuty < 0 || MaxDutyIndex < PrevDuty)
    {
      lcd.print("err");
    }
    else
    {
      lcd.print(dig[PrevDuty]);
    }
  }
}

void DispPsi(int psi)
{
  if (psi < 0)
  {
    psi = 0;
  }
  if (psi < 10)
  {
    lcd.print("  ");
  }
  else if (psi < 100)
  {
    lcd.print(" ");
  }
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
  lcd.setCursor(PressureOffset, 0);
  DispPsi(cur);
  // maximum during past 10 seconds
  lcd.setCursor(PressureOffset, PressureRow);
  DispPsi(localMax);
  // RPM at max pressure
  lcd.setCursor(RpmOffset, RpmRow);
  if (rpm < 10)
  {
    lcd.print(" ");
  }
  lcd.print(rpm);
}

/*void DispRpm(int rpm, int col, int row)
{
  int base = rpm / 50;
  rpm = base * 50 + (rpm - base >= 25 ? 50 : 0);
  lcd.setCursor(col, row);
  if (rpm < 0 || rpm > 9000)
  {
    lcd.print("err");
    return;
  }
  if (rpm < 10)
  {
    lcd.print("   ");
  }
  else if (rpm < 100)
  {
    lcd.print("  ");
  }
  else if (rpm < 1000)
  {
    lcd.print(" ");
  }
  lcd.print(rpm);
}*/

void DispTemp(int temp, int col, int row)
{
  lcd.setCursor(col, row);
  if (temp <= -10)
  { //-10
  }
  else if (temp < 0)
  { // -1
    lcd.print(" ");
  }
  else if (temp < 10)
  { //  9
    lcd.print("  ");
  }
  else if (temp < 100)
  { // 99
    lcd.print(" ");
  }
  else
  { //100
  }
  lcd.print(temp);
}

/*void DispVolt(int volt, int col, int row)
{
  lcd.setCursor(col, row);
  if (volt < 10)
  {
    lcd.print(" ");
    lcd.print(volt / 10);
  }
  else
    lcd.print(volt / 10);
  lcd.print(".");
  lcd.print(volt % 10);  
}*/
// END FILE
