/*
  stepper_serial.ino
  Arduino Mega 2축 시리얼 제어
    X축: DM542 + STM-4248 (리니어 축, mm 단위, 원점/끝 리밋)
    R축: TMC2209 + OK35STH13 (소형 회전 축, 도(deg) 단위)

  - 라이브러리: FastAccelStepper, TMCStepper (라이브러리 관리자에서 설치)
  - 시리얼 모니터: 115200 baud, 줄 끝 "Newline"
  - 명령 목록은 "help". 명령 앞에 축 이름(x / r)을 붙이면 그 축, 생략하면 X축
      예) go 300      -> X축 300mm로 이동
          r go 90     -> R축 90도로 이동

  배선 (wiring/wiring_diagram.html 참고)
    DM542   : PUL+ D6, DIR+ D5, ENA+ D4, PUL-/DIR-/ENA- -> GND
    TMC2209 : STEP D7, DIR D22, EN D23, PDN_UART <- TX1(D18, 1kΩ 경유) + RX1(D19),
              VIO 5V, MS1/MS2 GND
    리밋    : 원점 NC -> D2, 끝 NC -> D3, COM -> GND  (평소 LOW, 눌림/단선 HIGH)

  DM542 DIP (HLTNC 표 기준)
    전류 1.91A peak : SW1 on, SW2 off, SW3 on   (첫 시운전은 1.00A: SW1~3 on)
    SW4 off         : 정지 시 전류 절반
    3200 pulse/rev  : SW5 on, SW6 on, SW7 off, SW8 on
*/

#include <FastAccelStepper.h>
#include <TMCStepper.h>
#include <math.h>

// ================= 하드웨어 설정 =================
const uint8_t X_STEP_PIN = 6;   // Mega + FastAccelStepper: STEP은 6, 7, 8 중 하나
const uint8_t X_DIR_PIN  = 5;
const uint8_t X_ENA_PIN  = 4;
const uint8_t R_STEP_PIN = 7;
const uint8_t R_DIR_PIN  = 22;
const uint8_t R_EN_PIN   = 23;
const uint8_t LIM_HOME_PIN = 2; // NC 리밋: HIGH = 눌림 또는 단선
const uint8_t LIM_END_PIN  = 3;

const float X_PULSE_PER_REV = 3200.0;   // DM542 SW5~SW8 설정과 같아야 함
const float R_PULSE_PER_REV = 3200.0;   // 0.9도 모터(400스텝) x 1/8 마이크로스텝

const float    TMC_R_SENSE = 0.11f;     // 일반 TMC2209 모듈 감지 저항
const uint8_t  TMC_ADDR    = 0;         // MS1, MS2 = GND
const uint16_t R_CURRENT_MA_DEFAULT = 250;   // OK35STH13 정격 0.3A

const float HOME_FAST_MMS   = 20.0;   // 원점 1차 접근 속도
const float HOME_SLOW_MMS   = 3.0;    // 원점 2차 접근 속도
const float HOME_BACKOFF_MM = 5.0;    // 1차 접촉 후 후퇴 거리
const float HOME_OFFSET_MM  = 2.0;    // 스위치 지점에서 0점까지 거리
const float HOME_MAX_MM     = 800.0;  // 이 거리 안에 스위치가 없으면 실패

FastAccelStepperEngine engine;
TMC2209Stepper tmc(&Serial1, TMC_R_SENSE, TMC_ADDR);

enum Mode { IDLE, MOVING, JOGGING, CYCLING, HOME_FAST, HOME_BACKOFF, HOME_SLOW, HOME_OFFSET };

struct Axis {
  const char *name;          // "x" / "r"
  const char *unit;          // "mm" / "deg"
  FastAccelStepper *s;
  float pulsePerRev;
  float unitPerRev;          // X: 1회전당 mm, R: 360
  float speed;               // unit/s
  float accel;               // unit/s^2
  float maxSpeed;            // unit/s
  bool homed;                // 기준점이 잡혀 있는지 (X: home, R: zero)
  bool softLimit;
  float travelMax;           // 소프트 리밋 범위 0 ~ travelMax
  bool outputsOn;
  Mode mode;
  volatile int8_t moveDir;   // +1 / -1 / 0 (리밋 인터럽트에서 읽음)
  unsigned long moveStartMs;
  float cycleA, cycleB;
  uint16_t cycleLeg, cycleLegsTotal;
  unsigned long cycleStartMs;
};

Axis X, R;

struct Debounce { bool was; unsigned long since; };
Debounce dbHome = { false, 0 }, dbEnd = { false, 0 };

int32_t homeStartPos = 0;
bool limHome = false, limEnd = false;            // 디바운스된 리밋 상태
volatile bool homeTrip = false, endTrip = false; // 인터럽트에서 정지시켰는지
uint16_t rCurrentMa = R_CURRENT_MA_DEFAULT;

char lineBuf[64];
uint8_t lineLen = 0;

// ================= 단위 변환 =================
float spu(Axis &a) { return a.pulsePerRev / a.unitPerRev; }
int32_t toSteps(Axis &a, float u) { return (int32_t)lround(u * spu(a)); }
float toUnits(Axis &a, int32_t s) { return s / spu(a); }
float posU(Axis &a) { return toUnits(a, a.s->getCurrentPosition()); }

bool setSpeed(Axis &a, float v) {
  uint32_t hz = (uint32_t)(v * spu(a));
  if (hz < 1) hz = 1;
  if (a.s->setSpeedInHz(hz) != 0) {
    Serial.print(F("ERR 펄스 속도 한계 초과 ("));
    Serial.print(hz);
    Serial.print(F("Hz > "));
    Serial.print(a.s->getMaxSpeedInHz());
    Serial.println(F("Hz)"));
    return false;
  }
  return true;
}

bool applyMotion(Axis &a) {
  if (!setSpeed(a, a.speed)) return false;
  a.s->setAcceleration((int32_t)(a.accel * spu(a)));
  return true;
}

void hardStop(Axis &a) {
  a.s->forceStop();
  while (a.s->isRunning()) {}
  a.moveDir = 0;
}

// ================= 리밋스위치 (NC) =================
// 1ms 이상 연속 HIGH여야 눌림으로 판단 (노이즈 대비)
bool readSwitch(uint8_t pin, Debounce &d) {
  if (digitalRead(pin) == LOW) { d.was = false; return false; }
  if (!d.was) { d.was = true; d.since = micros(); }
  return (micros() - d.since) >= 1000UL;
}

// 인터럽트: 스위치 쪽으로 가는 중이면 즉시 정지 (시리얼 출력 등으로 loop가 늦어도 동작)
void isrHome() {
  if (X.moveDir < 0) { X.s->forceStop(); homeTrip = true; }
}
void isrEnd() {
  if (X.moveDir > 0) { X.s->forceStop(); endTrip = true; }
}

// ================= 출력 =================
void printElapsed(unsigned long sinceMs) {
  Serial.print((millis() - sinceMs) / 1000.0, 3);
  Serial.print(F("s"));
}

void printDone(Axis &a, const __FlashStringHelper *tag) {
  Serial.print(a.name);
  Serial.print(F(" "));
  Serial.print(tag);
  Serial.print(F(" pos="));
  Serial.print(posU(a), 3);
  Serial.print(a.unit);
  Serial.print(F(" t="));
  printElapsed(a.moveStartMs);
  Serial.println();
}

const __FlashStringHelper *modeName(Mode m) {
  switch (m) {
    case IDLE:         return F("IDLE");
    case MOVING:       return F("MOVING");
    case JOGGING:      return F("JOGGING");
    case CYCLING:      return F("CYCLING");
    case HOME_FAST:    return F("HOME_FAST");
    case HOME_BACKOFF: return F("HOME_BACKOFF");
    case HOME_SLOW:    return F("HOME_SLOW");
    case HOME_OFFSET:  return F("HOME_OFFSET");
  }
  return F("?");
}

void printHelp() {
  Serial.println(F("------------------- 명령 목록 -------------------"));
  Serial.println(F(" 명령 앞에 축 이름: x (리니어, mm) / r (회전, deg)"));
  Serial.println(F(" 생략하면 x.  예) go 300 / r go 90"));
  Serial.println(F(""));
  Serial.println(F(" rev <n>            n바퀴 회전 (음수=역방향, 소수 가능)"));
  Serial.println(F(" step <n>           n펄스 이동 (상대)"));
  Serial.println(F(" mv <d>             d만큼 이동 (상대, mm 또는 deg)"));
  Serial.println(F(" mm <d> / deg <d>   mv와 같음"));
  Serial.println(F(" go <p>             p 위치로 이동 (절대)"));
  Serial.println(F(" jog + | jog -      연속 이동 (stop으로 정지)"));
  Serial.println(F(" cycle <a> <b> <n>  a<->b 왕복 n회, 구간별 시간 측정"));
  Serial.println(F(" stop               감속 정지 (축 생략 시 두 축 모두)"));
  Serial.println(F(" estop              두 축 즉시 정지"));
  Serial.println(F(" zero               현재 위치를 0으로 지정"));
  Serial.println(F(" speed <v>          속도 (mm/s 또는 deg/s)"));
  Serial.println(F(" rpm <rpm>          속도를 RPM으로 지정"));
  Serial.println(F(" accel <a>          가감속 (mm/s2 또는 deg/s2)"));
  Serial.println(F(" time <d> <sec>     d를 sec초에 가도록 속도 자동 계산"));
  Serial.println(F(" softlim on|off     소프트 리밋 (기준점 설정 후 0~max)"));
  Serial.println(F(" max <p>            소프트 리밋 최대값"));
  Serial.println(F(" en on|off          모터 여자 ON/OFF"));
  Serial.println(F(""));
  Serial.println(F(" home               [x] 원점 리밋으로 원점 복귀"));
  Serial.println(F(" mmrev <mm>         [x] 1회전당 이동거리 (풀리 교체 시)"));
  Serial.println(F(" cur <mA>           [r] TMC2209 전류 (RMS)"));
  Serial.println(F(" tmc                [r] TMC2209 UART 연결 확인"));
  Serial.println(F(" status | pos       상태 / 현재 위치"));
  Serial.println(F("--------------------------------------------------"));
}

void printAxis(Axis &a) {
  Serial.print(F("["));          Serial.print(a.name);
  Serial.print(F("] mode="));    Serial.print(modeName(a.mode));
  Serial.print(F(" pos="));      Serial.print(posU(a), 3); Serial.print(a.unit);
  Serial.print(F(" ("));         Serial.print(a.s->getCurrentPosition());
  Serial.println(F(" steps)"));
  Serial.print(F("    ref="));   Serial.print(a.homed ? F("yes") : F("no"));
  Serial.print(F(" softlim="));  Serial.print(a.softLimit ? F("on") : F("off"));
  Serial.print(F(" max="));      Serial.print(a.travelMax, 1); Serial.print(a.unit);
  Serial.print(F(" outputs="));  Serial.println(a.outputsOn ? F("on") : F("off"));
  Serial.print(F("    speed="));  Serial.print(a.speed, 2); Serial.print(a.unit);
  Serial.print(F("/s ("));        Serial.print(a.speed * 60.0 / a.unitPerRev, 1);
  Serial.print(F("rpm) accel=")); Serial.print(a.accel, 1); Serial.print(a.unit);
  Serial.print(F("/s2 pulse/unit=")); Serial.println(spu(a), 3);
}

void printStatus() {
  printAxis(X);
  Serial.print(F("    limit home="));  Serial.print(limHome ? F("PRESSED") : F("ok"));
  Serial.print(F(" end="));            Serial.println(limEnd ? F("PRESSED") : F("ok"));
  printAxis(R);
  Serial.print(F("    tmc current=")); Serial.print(rCurrentMa); Serial.println(F("mA"));
}

// ================= 동작 시작 =================
bool towardPressedLimit(Axis &a, int8_t dir) {
  if (&a != &X) return false;
  if (dir < 0 && limHome) { Serial.println(F("ERR 원점 리밋이 눌려 있어 - 방향 이동 불가")); return true; }
  if (dir > 0 && limEnd)  { Serial.println(F("ERR 끝 리밋이 눌려 있어 + 방향 이동 불가")); return true; }
  return false;
}

bool startMoveTo(Axis &a, int32_t target, Mode m) {
  if (a.softLimit && a.homed && (target < 0 || target > toSteps(a, a.travelMax))) {
    Serial.print(F("ERR 소프트 리밋 초과 (0 ~ "));
    Serial.print(a.travelMax, 1);
    Serial.print(a.unit);
    Serial.println(F(")"));
    return false;
  }
  int32_t cur = a.s->getCurrentPosition();
  int8_t dir = (target > cur) ? 1 : (target < cur ? -1 : 0);
  if (towardPressedLimit(a, dir)) return false;
  if (!applyMotion(a)) return false;
  a.moveDir = dir;
  a.s->moveTo(target);
  a.moveStartMs = millis();
  a.mode = m;
  return true;
}

void startHome() {
  X.homed = false;
  if (!applyMotion(X) || !setSpeed(X, HOME_FAST_MMS)) return;
  X.moveStartMs = millis();
  if (limHome) {
    // 이미 눌려 있으면 바로 후퇴부터
    X.moveDir = 1;
    X.s->move(toSteps(X, HOME_BACKOFF_MM));
    X.mode = HOME_BACKOFF;
  } else {
    homeStartPos = X.s->getCurrentPosition();
    X.moveDir = -1;
    X.s->runBackward();
    X.mode = HOME_FAST;
  }
  Serial.println(F("x HOMING..."));
}

void startJog(Axis &a, int8_t dir) {
  if (towardPressedLimit(a, dir)) return;
  if (a.softLimit && a.homed) {
    // 기준점이 잡혀 있으면 리밋 끝점까지만 이동
    startMoveTo(a, dir > 0 ? toSteps(a, a.travelMax) : 0, JOGGING);
    return;
  }
  if (!applyMotion(a)) return;
  a.moveDir = dir;
  if (dir > 0) a.s->runForward();
  else         a.s->runBackward();
  a.moveStartMs = millis();
  a.mode = JOGGING;
}

void startCycle(Axis &a, float p1, float p2, uint16_t n) {
  if (a.softLimit && a.homed && (p1 < 0 || p2 < 0 || p1 > a.travelMax || p2 > a.travelMax)) {
    Serial.println(F("ERR 소프트 리밋 범위 밖"));
    return;
  }
  a.cycleA = p1;
  a.cycleB = p2;
  a.cycleLeg = 0;               // 0 = 시작점 a로 이동 (측정 안 함)
  a.cycleLegsTotal = n * 2;
  startMoveTo(a, toSteps(a, p1), CYCLING);
}

void cycleNext(Axis &a) {
  if (a.cycleLeg > 0) {
    Serial.print(F("leg "));
    Serial.print(a.cycleLeg);
    Serial.print(F("/"));
    Serial.print(a.cycleLegsTotal);
    Serial.print(F(" "));
    printDone(a, F("ok"));
  }
  if (a.cycleLeg >= a.cycleLegsTotal) {
    Serial.print(a.name);
    Serial.print(F(" CYCLE DONE total="));
    printElapsed(a.cycleStartMs);
    Serial.println();
    a.mode = IDLE;
    a.moveDir = 0;
    return;
  }
  a.cycleLeg++;
  if (a.cycleLeg == 1) a.cycleStartMs = millis();
  float target = (a.cycleLeg % 2 == 1) ? a.cycleB : a.cycleA;
  if (!startMoveTo(a, toSteps(a, target), CYCLING)) a.mode = IDLE;
}

// ================= 상태 갱신 (매 loop) =================
// 이동 중 리밋 처리. 정지했으면 true
bool checkLimits() {
  bool trip = homeTrip || endTrip;
  bool hitHome = limHome && X.moveDir < 0;
  bool hitEnd  = limEnd && X.moveDir > 0;
  if (!trip && !hitHome && !hitEnd) return false;

  bool real = hitHome || hitEnd ||
              (homeTrip && digitalRead(LIM_HOME_PIN) == HIGH) ||
              (endTrip && digitalRead(LIM_END_PIN) == HIGH);
  hardStop(X);
  X.mode = IDLE;
  homeTrip = endTrip = false;
  if (real) {
    X.homed = false;
    Serial.println(F("ERR x 리밋 감지 -> 정지. home 필요"));
  } else {
    Serial.print(F("WARN x 리밋 신호가 순간 감지됨(노이즈?) -> 정지. pos="));
    Serial.print(posU(X), 3);
    Serial.println(F("mm"));
  }
  return true;
}

void updateMove(Axis &a) {
  if (!a.s->isRunning()) {
    if (a.mode == CYCLING) {
      cycleNext(a);
    } else {
      printDone(a, F("DONE"));
      a.mode = IDLE;
      a.moveDir = 0;
    }
  }
}

void updateX() {
  limHome = readSwitch(LIM_HOME_PIN, dbHome);
  limEnd  = readSwitch(LIM_END_PIN, dbEnd);

  switch (X.mode) {
    case IDLE:
      homeTrip = endTrip = false;
      break;

    case MOVING:
    case JOGGING:
    case CYCLING:
      if (checkLimits()) return;
      updateMove(X);
      break;

    case HOME_FAST:
      homeTrip = false;
      if (limHome) {
        hardStop(X);
        setSpeed(X, HOME_FAST_MMS);
        X.moveDir = 1;
        X.s->move(toSteps(X, HOME_BACKOFF_MM));
        X.mode = HOME_BACKOFF;
      } else if (homeStartPos - X.s->getCurrentPosition() > toSteps(X, HOME_MAX_MM)) {
        hardStop(X);
        X.mode = IDLE;
        Serial.println(F("ERR home 실패: 원점 스위치를 찾지 못함"));
      } else if (limEnd) {
        hardStop(X);
        X.mode = IDLE;
        Serial.println(F("ERR home 실패: 끝 리밋이 눌림 (방향 또는 배선 확인)"));
      } else if (!X.s->isRunning()) {
        X.s->runBackward();   // 노이즈로 멈췄으면 다시 진행
      }
      break;

    case HOME_BACKOFF:
      if (!X.s->isRunning()) {
        if (limHome) {
          X.mode = IDLE;
          X.moveDir = 0;
          Serial.println(F("ERR home 실패: 후퇴 후에도 원점 스위치가 눌려 있음 (단선 확인)"));
          break;
        }
        setSpeed(X, HOME_SLOW_MMS);
        homeStartPos = X.s->getCurrentPosition();
        X.moveDir = -1;
        X.s->runBackward();
        X.mode = HOME_SLOW;
      }
      break;

    case HOME_SLOW:
      homeTrip = false;
      if (limHome) {
        hardStop(X);
        X.s->setCurrentPosition(-toSteps(X, HOME_OFFSET_MM));
        applyMotion(X);
        X.moveDir = 1;
        X.s->moveTo(0);
        X.mode = HOME_OFFSET;
      } else if (homeStartPos - X.s->getCurrentPosition() > toSteps(X, HOME_BACKOFF_MM * 2)) {
        hardStop(X);
        X.mode = IDLE;
        Serial.println(F("ERR home 실패: 저속 접근 중 스위치를 찾지 못함"));
      } else if (!X.s->isRunning()) {
        X.s->runBackward();
      }
      break;

    case HOME_OFFSET:
      if (checkLimits()) return;
      if (!X.s->isRunning()) {
        X.homed = true;
        X.mode = IDLE;
        X.moveDir = 0;
        printDone(X, F("HOMED"));
      }
      break;
  }
}

void updateR() {
  if (R.mode != IDLE) updateMove(R);
}

// ================= TMC2209 =================
void tmcInit() {
  Serial1.begin(115200);
  tmc.begin();
  tmc.toff(4);
  tmc.blank_time(24);
  tmc.pdn_disable(true);        // PDN 핀을 UART로 사용
  tmc.mstep_reg_select(true);   // 마이크로스텝을 레지스터로 설정
  tmc.I_scale_analog(false);    // Vref 대신 내부 기준 사용
  tmc.rms_current(rCurrentMa);
  tmc.microsteps(8);
  tmc.en_spreadCycle(false);    // StealthChop (저소음)
  tmc.pwm_autoscale(true);
}

void tmcReport() {
  uint8_t r = tmc.test_connection();
  Serial.print(F("tmc UART "));
  if (r == 0) {
    Serial.print(F("OK  current="));
    Serial.print(tmc.rms_current());
    Serial.print(F("mA microsteps="));
    Serial.println(tmc.microsteps());
  } else {
    Serial.print(F("연결 실패 (code "));
    Serial.print(r);
    Serial.println(F(") - PDN_UART, 1kΩ, VIO, VM 배선 확인. MS1/MS2=GND면 1/8 스텝으로는 동작"));
  }
}

// ================= 명령 처리 =================
bool requireIdle(Axis &a) {
  if (a.mode != IDLE) {
    Serial.print(F("ERR "));
    Serial.print(a.name);
    Serial.println(F("축 동작 중 - stop 먼저"));
    return false;
  }
  return true;
}

bool requireArg(const char *arg) {
  if (!arg) {
    Serial.println(F("ERR 인자 필요 (help 참고)"));
    return false;
  }
  return true;
}

bool isOn(const char *s) { return strcmp(s, "on") == 0 || strcmp(s, "1") == 0; }

void stopAxis(Axis &a) {
  if (a.mode == IDLE) return;
  if (a.mode >= HOME_FAST) a.homed = false;
  a.s->stopMove();
  a.mode = MOVING;   // 감속 끝나면 DONE 출력
}

void printSpeed(Axis &a) {
  Serial.print(F("OK "));
  Serial.print(a.name);
  Serial.print(F(" speed="));
  Serial.print(a.speed, 2);
  Serial.print(a.unit);
  Serial.print(F("/s ("));
  Serial.print(a.speed * 60.0 / a.unitPerRev, 1);
  Serial.println(F("rpm)"));
}

void handleLine(char *line) {
  for (char *p = line; *p; ++p) *p = tolower(*p);
  Serial.print(F("> "));
  Serial.println(line);

  char *tok[5] = { NULL, NULL, NULL, NULL, NULL };
  uint8_t n = 0;
  for (char *t = strtok(line, " \t"); t && n < 5; t = strtok(NULL, " \t")) tok[n++] = t;
  if (n == 0) return;

  // 축 접두어
  Axis *ap = &X;
  bool axisGiven = false;
  uint8_t k = 0;
  if (!strcmp(tok[0], "x") || !strcmp(tok[0], "r")) {
    ap = (tok[0][0] == 'r') ? &R : &X;
    axisGiven = true;
    k = 1;
  }
  Axis &a = *ap;
  const char *cmd = tok[k];
  const char *a1 = (k + 1 < 5) ? tok[k + 1] : NULL;
  const char *a2 = (k + 2 < 5) ? tok[k + 2] : NULL;
  const char *a3 = (k + 3 < 5) ? tok[k + 3] : NULL;
  if (!cmd) { printAxis(a); return; }
  bool isX = (&a == &X);

  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
    printHelp();

  } else if (!strcmp(cmd, "status") || !strcmp(cmd, "st")) {
    if (axisGiven) printAxis(a); else printStatus();

  } else if (!strcmp(cmd, "pos")) {
    Serial.print(F("x="));  Serial.print(posU(X), 3);
    Serial.print(F("mm r=")); Serial.print(posU(R), 2); Serial.println(F("deg"));

  } else if (!strcmp(cmd, "rev")) {
    if (!requireIdle(a) || !requireArg(a1)) return;
    int32_t d = (int32_t)lround(atof(a1) * a.pulsePerRev);
    startMoveTo(a, a.s->getCurrentPosition() + d, MOVING);

  } else if (!strcmp(cmd, "step")) {
    if (!requireIdle(a) || !requireArg(a1)) return;
    startMoveTo(a, a.s->getCurrentPosition() + atol(a1), MOVING);

  } else if (!strcmp(cmd, "mv") || !strcmp(cmd, "mm") || !strcmp(cmd, "deg")) {
    if (!requireIdle(a) || !requireArg(a1)) return;
    startMoveTo(a, a.s->getCurrentPosition() + toSteps(a, atof(a1)), MOVING);

  } else if (!strcmp(cmd, "go")) {
    if (!requireIdle(a) || !requireArg(a1)) return;
    if (!a.homed) {
      Serial.print(F("WARN "));
      Serial.print(a.name);
      Serial.println(isX ? F(" 원점 미설정 - 전원 켠 위치 기준") : F(" 기준점 미설정 - zero로 0점 지정 권장"));
    }
    startMoveTo(a, toSteps(a, atof(a1)), MOVING);

  } else if (!strcmp(cmd, "jog")) {
    if (!requireIdle(a) || !requireArg(a1)) return;
    startJog(a, a1[0] == '-' ? -1 : 1);

  } else if (!strcmp(cmd, "cycle")) {
    if (!requireIdle(a) || !requireArg(a1) || !requireArg(a2) || !requireArg(a3)) return;
    long cnt = atol(a3);
    if (cnt < 1 || cnt > 10000) { Serial.println(F("ERR n은 1 ~ 10000")); return; }
    startCycle(a, atof(a1), atof(a2), (uint16_t)cnt);

  } else if (!strcmp(cmd, "stop")) {
    if (axisGiven) stopAxis(a);
    else { stopAxis(X); stopAxis(R); }

  } else if (!strcmp(cmd, "estop")) {
    hardStop(X); hardStop(R);
    X.mode = IDLE; R.mode = IDLE;
    X.homed = false; R.homed = false;
    Serial.print(F("ESTOP x=")); Serial.print(posU(X), 3);
    Serial.print(F("mm r="));    Serial.print(posU(R), 2);
    Serial.println(F("deg (기준점 다시 필요)"));

  } else if (!strcmp(cmd, "home")) {
    if (!isX) { Serial.println(F("ERR r축은 리밋이 없음 - zero 사용")); return; }
    if (!requireIdle(X)) return;
    startHome();

  } else if (!strcmp(cmd, "zero")) {
    if (!requireIdle(a)) return;
    a.s->setCurrentPosition(0);
    a.homed = true;
    Serial.print(F("OK "));
    Serial.print(a.name);
    Serial.println(F(" 현재 위치 = 0"));

  } else if (!strcmp(cmd, "speed") || !strcmp(cmd, "rpm")) {
    if (!requireArg(a1)) return;
    float v = atof(a1);
    if (!strcmp(cmd, "rpm")) v = v * a.unitPerRev / 60.0;
    if (v <= 0 || v > a.maxSpeed) {
      Serial.print(F("ERR 속도 범위 0 ~ "));
      Serial.print(a.maxSpeed, 0);
      Serial.print(a.unit);
      Serial.println(F("/s"));
      return;
    }
    float old = a.speed;
    a.speed = v;
    if (a.s->isRunning() && a.mode < HOME_FAST) {
      if (!applyMotion(a)) { a.speed = old; applyMotion(a); return; }
      a.s->applySpeedAcceleration();
    }
    printSpeed(a);

  } else if (!strcmp(cmd, "accel")) {
    if (!requireArg(a1)) return;
    float v = atof(a1);
    if (v <= 0) { Serial.println(F("ERR accel > 0")); return; }
    a.accel = v;
    if (a.s->isRunning() && a.mode < HOME_FAST) {
      applyMotion(a);
      a.s->applySpeedAcceleration();
    }
    Serial.print(F("OK ")); Serial.print(a.name);
    Serial.print(F(" accel=")); Serial.print(a.accel, 1);
    Serial.print(a.unit); Serial.println(F("/s2"));

  } else if (!strcmp(cmd, "time")) {
    // 사다리꼴 프로파일: T = d/v + v/acc  ->  v = (acc*T - sqrt(acc^2*T^2 - 4*acc*d)) / 2
    if (!requireArg(a1) || !requireArg(a2)) return;
    float d = fabs(atof(a1));
    float T = atof(a2);
    float acc = a.accel;
    if (d <= 0 || T <= 0) { Serial.println(F("ERR d, sec > 0")); return; }
    float disc = acc * acc * T * T - 4.0 * acc * d;
    if (disc < 0) {
      Serial.print(F("ERR 가속도가 부족해 불가능. accel을 "));
      Serial.print(4.0 * d / (T * T), 1);
      Serial.println(F(" 이상으로 올릴 것"));
      return;
    }
    float v = (acc * T - sqrt(disc)) / 2.0;
    if (v > a.maxSpeed) { Serial.println(F("ERR 필요 속도가 최대 속도 초과")); return; }
    a.speed = v;
    printSpeed(a);

  } else if (!strcmp(cmd, "softlim")) {
    if (!requireArg(a1)) return;
    a.softLimit = isOn(a1);
    Serial.print(F("OK ")); Serial.print(a.name);
    Serial.print(F(" softlim=")); Serial.println(a.softLimit ? F("on") : F("off"));

  } else if (!strcmp(cmd, "max")) {
    if (!requireArg(a1)) return;
    float m = atof(a1);
    if (m <= 0) { Serial.println(F("ERR max > 0")); return; }
    a.travelMax = m;
    Serial.print(F("OK ")); Serial.print(a.name);
    Serial.print(F(" max=")); Serial.print(a.travelMax, 1); Serial.println(a.unit);

  } else if (!strcmp(cmd, "mmrev")) {
    if (!isX) { Serial.println(F("ERR x축 전용 명령")); return; }
    if (!requireIdle(X) || !requireArg(a1)) return;
    float m = atof(a1);
    if (m <= 0) { Serial.println(F("ERR mmrev > 0")); return; }
    X.unitPerRev = m;
    X.homed = false;   // 스케일이 바뀌었으므로 원점 다시
    Serial.print(F("OK mmPerRev=")); Serial.print(X.unitPerRev, 3);
    Serial.print(F(" pulse/mm=")); Serial.print(spu(X), 3);
    Serial.println(F(" (home 다시 필요)"));

  } else if (!strcmp(cmd, "cur")) {
    if (isX) { Serial.println(F("ERR x축 전류는 DM542 DIP 스위치로 설정")); return; }
    if (!requireArg(a1)) return;
    long ma = atol(a1);
    if (ma < 50 || ma > 400) { Serial.println(F("ERR 50 ~ 400mA (모터 정격 300mA)")); return; }
    rCurrentMa = (uint16_t)ma;
    tmc.rms_current(rCurrentMa);
    Serial.print(F("OK r current=")); Serial.print(rCurrentMa); Serial.println(F("mA"));

  } else if (!strcmp(cmd, "tmc")) {
    tmcReport();

  } else if (!strcmp(cmd, "en")) {
    if (!requireIdle(a) || !requireArg(a1)) return;
    a.outputsOn = isOn(a1);
    if (a.outputsOn) {
      a.s->enableOutputs();
    } else {
      a.s->disableOutputs();
      a.homed = false;   // 손으로 움직일 수 있으므로 위치 신뢰 불가
    }
    Serial.print(F("OK ")); Serial.print(a.name);
    Serial.print(F(" outputs="));
    Serial.println(a.outputsOn ? F("on") : F("off (기준점 다시 필요)"));

  } else {
    Serial.println(F("ERR 알 수 없는 명령 (help 참고)"));
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      Serial.println(F("ERR 명령이 너무 김 (63자 이하)"));
      lineLen = 0;
      while (Serial.available() && Serial.peek() != '\n') Serial.read();
    }
  }
}

// ================= setup / loop =================
void initAxis(Axis &a, const char *name, const char *unit, float ppr, float upr,
              float speed, float accel, float maxSpeed, bool softLimit, float travelMax) {
  a.name = name; a.unit = unit; a.s = NULL;
  a.pulsePerRev = ppr; a.unitPerRev = upr;
  a.speed = speed; a.accel = accel; a.maxSpeed = maxSpeed;
  a.homed = false; a.softLimit = softLimit; a.travelMax = travelMax;
  a.outputsOn = true; a.mode = IDLE; a.moveDir = 0; a.moveStartMs = 0;
  a.cycleLeg = 0; a.cycleLegsTotal = 0;
}

bool connectAxis(Axis &a, uint8_t stepPin, uint8_t dirPin, uint8_t enPin) {
  a.s = engine.stepperConnectToPin(stepPin);
  if (!a.s) return false;
  a.s->setDirectionPin(dirPin);
  a.s->setEnablePin(enPin);   // LOW = 활성 (DM542 ENA 미통전 / TMC2209 EN LOW)
  a.s->setAutoEnable(false);
  a.s->enableOutputs();
  applyMotion(a);
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(LIM_HOME_PIN, INPUT_PULLUP);
  pinMode(LIM_END_PIN, INPUT_PULLUP);

  //       축   단위   pulse/rev        unit/rev  속도  가속  최대속도 softlim max
  initAxis(X, "x", "mm",  X_PULSE_PER_REV, 40.0,   100,  500,  400,    true,   650);
  initAxis(R, "r", "deg", R_PULSE_PER_REV, 360.0,  180,  720,  1800,   false,  360);

  engine.init();
  if (!connectAxis(X, X_STEP_PIN, X_DIR_PIN, X_ENA_PIN) ||
      !connectAxis(R, R_STEP_PIN, R_DIR_PIN, R_EN_PIN)) {
    Serial.println(F("ERR STEP 핀은 6, 7, 8 중 하나여야 함"));
    while (true) {}
  }

  tmcInit();
  attachInterrupt(digitalPinToInterrupt(LIM_HOME_PIN), isrHome, RISING);
  attachInterrupt(digitalPinToInterrupt(LIM_END_PIN), isrEnd, RISING);

  Serial.println(F("== 2-axis controller (Mega + DM542 + TMC2209) =="));
  printHelp();
  tmcReport();
  if (digitalRead(LIM_HOME_PIN) == HIGH) Serial.println(F("WARN 원점 리밋이 눌려 있거나 단선"));
  if (digitalRead(LIM_END_PIN) == HIGH)  Serial.println(F("WARN 끝 리밋이 눌려 있거나 단선"));
  printStatus();
}

void loop() {
  readSerial();
  updateX();
  updateR();
}
