# STM32_Humidity Sensor Project History

## Project

- 프로젝트: STM32_Humidity Sensor
- 보드: Nucleo-F411RE
- 개발환경: PlatformIO + STM32Cube HAL
- 1차 목표: AHT20 습도센서로 습도를 읽고, SG90 서보모터로 제습기 전원 버튼을 물리적으로 누르는 자동 제습기 제어기 제작. LCD 1602A도 나중에 병렬 4비트 방식으로 표시 예정.

## Current Main Files

- `platformio.ini`
- `src/main.c`

## platformio.ini

[env:nucleo_f411re]
platform = ststm32
board = nucleo_f411re
framework = stm32cube
monitor_speed = 115200
monitor_port = COM5
monitor_dtr = 0
monitor_rts = 0
upload_protocol = stlink
debug_tool = stlink

중요: `monitor_port = COM5`는 기존 PC 기준이므로 다른 PC에서는 COM 포트가 달라질 수 있음. 다른 PC에서는 `PlatformIO: device list`로 확인 후 수정 필요.

## Hardware Connections

### 1. AHT20+BMP280 Sensor

기존 기준:

- VCC -> 3V3
- GND -> GND
- SDA -> D14 / PB9
- SCL -> D15 / PB8

수정일: 2026-05-10

현재 코드는 AHT20 센서 2개를 읽기 위한 I2C 핀 다중화 구조를 포함함.

Sensor 0:

- VCC -> 3V3
- GND -> GND
- SDA -> D14 / PB9
- SCL -> D15 / PB8

Sensor 1:

- VCC -> 3V3
- GND -> GND
- SDA -> PB7
- SCL -> PB6 / D10

AHT20은 보통 I2C 주소가 `0x38`로 고정되어 있음. 같은 I2C 버스에 여러 개를 동시에 연결하면 주소가 겹쳐 구분하기 어렵기 때문에, 현재 코드는 한 번에 하나의 I2C1 핀 쌍만 활성화하고 센서 0, 센서 1을 순서대로 읽는 방식으로 구성됨.

### 2. SG90 Servo Motor

- 빨강 -> 외부 5V
- 갈색 -> GND
- 주황 -> D9 / PC7
- 외부 5V 전원 GND와 Nucleo GND는 반드시 공통 연결
- 코드에서는 TIM3_CH2 PWM 사용

### 3. 1602A LCD Parallel 4-bit Mode

- LCD 1 VSS -> GND
- LCD 2 VDD -> 3V3 또는 5V
- LCD 3 V0 -> 가변저항 가운데 핀
- LCD 4 RS -> D2 / PA10
- LCD 5 RW -> GND
- LCD 6 E -> D3 / PB3
- LCD 11 D4 -> D4 / PB5
- LCD 12 D5 -> D5 / PB4
- LCD 13 D6 -> D6 / PB10
- LCD 14 D7 -> D7 / PA8
- LCD 15 A -> 3V3 또는 5V
- LCD 16 K -> GND

## Code Change History

1. AHT20 센서는 라이브러리 없이 `HAL_I2C_Master_Transmit` / `HAL_I2C_Master_Receive`로 직접 제어.
2. 릴레이 방식은 사용하지 않기로 하고, SG90 서보모터가 제습기 전원 버튼을 누르는 방식으로 변경.
3. LCD는 처음 I2C LCD로 가정했으나, 실제 LCD가 SDA/SCL 없는 일반 1602A라 병렬 4비트 방식으로 코드 변경.
4. LCD 표시 내용과 같은 습도/상태 내용을 시리얼 모니터에도 출력하도록 `serial_show_display_status()` 추가.
5. 시리얼 모니터가 처음에는 아무것도 안 나오거나 B0 UART까지만 나왔음.
6. 원인 추적을 위해 B0 UART, B1 I2C, B2 TIM3, B3 PWM, B4 LCD, B5 AHT 같은 부팅 단계 로그 추가.
7. `HAL_Delay()`가 멈추는 문제가 있었음. 원인은 `SysTick_Handler()`가 없어서 HAL tick이 증가하지 않았기 때문.
8. `main.c` 맨 아래에 아래 함수를 추가해서 해결:

void SysTick_Handler(void)
{
  HAL_IncTick();
}

9. 문제 확인을 위해 `SERIAL_ONLY_TEST` 모드를 임시로 만들었음. `#define SERIAL_ONLY_TEST 1`이면 I2C/LCD/센서/서보 없이 시리얼만 1초마다 출력. 현재는 `#define SERIAL_ONLY_TEST 0`으로 다시 꺼둠.
10. 현재 AHT20 습도센서는 시리얼 모니터에 정상 출력됨.
11. SG90 서보는 D9 / PC7 / TIM3_CH2 기준으로 코드 수정됨.
12. 테스트 편의를 위해 부팅 시 서보가 한 번 움직이도록 `SERVO_TEST_ON_BOOT = 1` 설정.
13. 기존 5분 재동작 제한은 테스트를 위해 `min_off_time_passed()`에서 `return true;`로 우회하고, 기존 코드는 주석처리해둠.

수정일: 2026-05-10

14. `src/main.c` 상단에 코드 모듈 지도를 추가함. 하드웨어 핀/상수, HAL 핸들 및 실행 상태, UART/Delay 유틸리티, 서보 제어, 안전 시간 정책, AHT20 드라이버, LCD 드라이버, 표시 계층, `main()` 흐름, STM32Cube HAL 초기화 영역으로 분류함.
15. 습도 센서 문제 확인을 위해 I2C 핀 조합을 바꿔 테스트할 수 있는 구조를 임시로 추가함. PB8/PB9와 PB6/PB7 조합을 선택할 수 있게 했음.
16. 이후 센서 2개 다중화 구조로 확장하면서 `I2C_PIN_SET` 방식은 `SENSOR_COUNT`와 `sensor_i2c_buses[]` 테이블 방식으로 변경함.
17. `SensorI2cBusConfig` 구조체와 `sensor_i2c_buses[]` 테이블을 추가하여 센서별 SCL/SDA 핀 정보를 관리하도록 변경함.
18. `sensor_i2c_release_all_pins()`, `sensor_i2c_configure_bus_pins()`, `sensor_i2c_select()`를 추가하여 I2C1 핀 쌍을 런타임에 전환하도록 구성함.
19. `aht20_init_sensor()`와 `aht20_read_sensor()`를 추가하여 센서 번호 기준으로 AHT20을 초기화하고 읽을 수 있게 변경함.
20. 부팅 시 각 센서의 핀 정보를 UART로 출력하도록 `uart_print_sensor_pin_plan()`을 추가함.
21. 부팅 시 `[AHT20-0] OK`, `[AHT20-1] OK` 형식으로 각 센서 초기화 결과를 출력하도록 변경함.
22. 메인 루프에서 센서 0과 센서 1을 순서대로 읽고, 정상 읽힌 센서 중 가장 높은 습도값을 제습기 제어 기준으로 사용하도록 변경함.
23. 모든 센서 읽기에 실패하면 서보 버튼을 누르지 않고 `[SAFE] No humidity sensor reading. Servo will not press button.` 메시지를 출력하도록 변경함.
24. 두 번째 센서를 2m 정도 떨어뜨리는 방법을 검토함. 블루투스 같은 무선 시리얼 방식은 센서 쪽에 별도 MCU와 전원이 필요하므로, 먼저 3V3/GND/SDA/SCL 4선을 유선으로 연장해서 테스트하는 방식 검토중.

## Current Behavior

- 부팅 후 UART 초기화
- 현재 등록된 센서별 I2C 핀 정보 UART 출력
- I2C1 핀 쌍 선택 및 초기화
- TIM3 PWM 초기화
- PWM 시작
- `SERVO_TEST_ON_BOOT`가 1이면 서보가 한 번 움직임
- LCD 초기화 시도
- AHT20 센서 0 초기화
- AHT20 센서 1 초기화
- 1초마다 센서 0/1의 습도/온도 읽기 시도
- 읽기에 성공한 센서값을 시리얼에 출력
- 정상 읽힌 센서 중 가장 높은 습도값을 제어 기준으로 사용
- 기준 습도 >= 65%이면 제습기 ON 상태로 보고 서보 버튼 누름
- 기준 습도 <= 55%이면 OFF 상태로 보고 서보 버튼 누름
- 최대 ON 시간은 3시간 설정
- 5분 OFF 대기는 현재 테스트 목적으로 비활성화

## Important Constants

```c
#define SERVO_REST_US            1000U
#define SERVO_PRESS_US           1800U
#define SERVO_PRESS_TIME_MS      700UL
#define SERVO_SETTLE_TIME_MS     700UL
#define SERVO_TEST_ON_BOOT       1

#define HUMIDITY_ON_PERCENT      65.0f
#define HUMIDITY_OFF_PERCENT     55.0f

#define SENSOR_READ_INTERVAL_MS  1000UL
#define MAX_ON_TIME_MS           (3UL * 60UL * 60UL * 1000UL)
#define MIN_OFF_TIME_MS          (5UL * 60UL * 1000UL)

#define SENSOR_COUNT             2U
```

## Notes

- 서보모터는 Nucleo 보드 5V에서 직접 전원 공급하지 않는 것을 권장. 외부 5V 전원 사용.
- 외부 전원 GND와 Nucleo GND는 반드시 연결해야 PWM 신호 기준이 맞음.
- LCD는 현재 코드상 병렬 4비트 방식이지만, 아직 실제 화면 출력은 완전히 검증되지 않았음.
- LCD에 검은 블록만 보이는 경우 V0 대비 조절, RS/E/D4~D7 배선, 5V LCD와 3.3V 신호 호환 문제 확인 필요.
- 다른 PC에서는 COM5가 아닐 수 있으므로 `platformio.ini`의 `monitor_port` 수정 필요.
- 수정일: 2026-05-10 - 두 번째 센서를 2m 정도 떨어뜨리기위해, 먼저 별도 전원 없이 Nucleo의 3V3/GND/SDA/SCL 4선을 연장해서 테스트하는 것으로 검토중. 무선 방식으로 갈 경우 센서 쪽 별도 MCU와 전원이 필요함.


## Plan

- 제습기 전원에 대해 물리적이 아닌 릴레이 방식으로 작동
- 센서의 다중화로 실내 여러 위치에 대해 습도 측정
- 센서의 무선화
- 어플 제작 및 UI를 통해 별도 작동 기능 추가, 모드 분리
