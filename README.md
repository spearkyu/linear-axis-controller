# Linear Axis Controller

Arduino Mega 2560 하나로 스텝모터 2축을 시리얼 명령으로 제어합니다.

| 축 | 드라이버 | 모터 | 단위 | 용도 |
|---|---|---|---|---|
| **x** | DM542 | STM-4248 (NEMA17, 1.5A) | mm | 벨트 리니어 축, 원점·끝 리밋 |
| **r** | TMC2209 (UART) | OK35STH13 (0.9°, 0.3A) | deg | 소형 회전 축 |

**배선도 (휴대폰에서도 보기 좋음): https://spearkyu.github.io/linear-axis-controller/**
배선도에서 부품을 누르면 사양과 연결 정보가 아래에 표시되고, 연결된 배선만 강조됩니다.

## 준비

1. Arduino IDE 라이브러리 관리자에서 설치
   - `FastAccelStepper` (2.0.1에서 확인)
   - `TMCStepper` (0.7.3에서 확인)
2. 보드: **Arduino Mega or Mega 2560**
3. [`stepper_serial/stepper_serial.ino`](stepper_serial/stepper_serial.ino) 업로드
4. 시리얼 모니터: **115200 baud**, 줄 끝 **Newline**

## 핀 연결

| Mega | 연결 | 비고 |
|---|---|---|
| D6 / D5 / D4 | DM542 PUL+ / DIR+ / ENA+ | PUL-, DIR-, ENA-는 GND |
| D7 / D22 / D23 | TMC2209 STEP / DIR / EN | STEP 핀은 6, 7, 8만 가능 |
| D18 TX1 → 1kΩ, D19 RX1 | TMC2209 PDN_UART | 전류 설정용 |
| 5V | TMC2209 VIO | MS1, MS2는 GND |
| D2 | 원점 리밋 NC | COM은 GND |
| D3 | 끝 리밋 NC | COM은 GND |

전원은 24V SMPS 1개를 퓨즈로 나눠 DM542(F1 3A)와 TMC2209 VM(F2 1A)에 공급합니다. 모든 GND는 공통입니다.

DM542 DIP: `SW1 on, SW2 off, SW3 on` (1.91A peak), `SW4 off`, `SW5 on, SW6 on, SW7 off, SW8 on` (3200 pulse/rev).
기구에 장착된 상태의 첫 시운전은 `SW1~3 모두 on` (1.00A)로 시작합니다.

## 명령

명령 앞에 축 이름 `x` 또는 `r`을 붙입니다. 생략하면 x축입니다.

```
go 300        x축 300mm로 이동
r go 90       r축 90도로 이동
r rev 1       r축 1바퀴
stop          두 축 감속 정지
```

| 명령 | 설명 |
|---|---|
| `rev <n>` | n바퀴 회전 (음수 = 역방향, 소수 가능) |
| `step <n>` | n펄스 이동 |
| `mv <d>` (`mm`, `deg`) | 상대 이동 |
| `go <p>` | 절대 위치로 이동 |
| `jog +` / `jog -` | 연속 이동, `stop`으로 정지 |
| `cycle <a> <b> <n>` | a↔b 왕복 n회, 구간마다 시간 출력 |
| `stop` | 감속 정지 (축 생략 시 두 축 모두) |
| `estop` | 두 축 즉시 정지 (이후 기준점 다시 잡기) |
| `zero` | 현재 위치를 0으로 |
| `speed <v>` / `rpm <rpm>` | 속도 (mm/s 또는 deg/s) |
| `accel <a>` | 가감속 |
| `time <d> <sec>` | d를 sec초에 가는 속도 자동 계산 (예: `time 650 7`) |
| `softlim on\|off`, `max <p>` | 소프트 리밋 0 ~ max |
| `en on\|off` | 모터 여자 ON/OFF |
| `home` | [x] 원점 리밋으로 원점 복귀 |
| `mmrev <mm>` | [x] 1회전당 이동거리 (기본 40 = GT2 20T) |
| `cur <mA>` | [r] TMC2209 전류 (기본 250mA) |
| `tmc` | [r] TMC2209 UART 연결 확인 |
| `status`, `pos`, `help` | 상태 / 위치 / 명령 목록 |

동작이 끝나면 `x DONE pos=300.000mm t=3.412s`처럼 위치와 걸린 시간이 출력됩니다.

## 리밋스위치 (NC 배선)

- 평소 LOW, 눌리거나 **선이 끊어지면 HIGH** → 단선도 정지로 처리됩니다.
- 해당 방향으로 움직이는 중에 눌리면 **인터럽트로 즉시 정지**합니다.
- 리밋에 걸려 멈추면 위치 기준이 풀리므로 `home`을 다시 해야 합니다.
- 전원을 켰을 때 스위치가 HIGH면 `WARN ... 눌려 있거나 단선`이 출력됩니다.

## 처음 시운전 순서

1. 전원을 끈 채로 캐리지를 손으로 레일 중앙에 둡니다.
2. 전원을 켜고 `status` → 리밋 두 개가 `ok`인지, 손으로 누르면 `PRESSED`로 바뀌는지 확인합니다.
3. `mv 1`로 1mm 이동 → 원점 반대쪽으로 가는지 확인합니다.
4. `mv 10` 후 자로 재서 스케일을 확인합니다 (`mmrev`로 보정).
5. `speed 20` → `home` (멀티탭 스위치에 손을 둔 채로).
6. `jog +`로 끝까지 천천히 이동해 실제 행정을 재고 `max`를 그보다 5~10mm 짧게 설정합니다.
7. `time 650 7` → `cycle 0 650 3`으로 목표 동작을 확인합니다.
8. `tmc`로 TMC2209 UART가 `OK`인지 확인하고 `r rev 1`로 r축을 시험합니다.

## 파일

- [`stepper_serial/stepper_serial.ino`](stepper_serial/stepper_serial.ino) – 2축 제어 스케치
- [`wiring/wiring_diagram.html`](wiring/wiring_diagram.html) – 배선도 원본
- [`wiring/build_pages.ps1`](wiring/build_pages.ps1) – 원본으로 `docs/index.html`(GitHub Pages) 생성
