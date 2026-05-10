#include "stm32f4xx_hal.h"  // STM32F4 HAL 라이브러리 선언을 가져옵니다.
#include <stdbool.h>  // bool, true, false 타입을 사용하기 위한 표준 헤더입니다.
#include <stdint.h>  // uint8_t, uint16_t, uint32_t 같은 고정 크기 정수 타입을 사용합니다.
#include <string.h>  // strlen() 같은 문자열 처리 함수를 사용합니다.

/*
 * Nucleo-F411RE pin plan
 *
 * AHT20/BMP280 module:
 *   VCC -> 3V3
 *   GND -> GND
 *
 *   Sensor 0, original wiring:
 *     SDA -> PB9  (Arduino D14)
 *     SCL -> PB8  (Arduino D15)
 *
 *   Sensor 1, alternate wiring:
 *     SDA -> PB7
 *     SCL -> PB6  (Arduino D10)
 *
 *   AHT20 modules usually use the fixed I2C address 0x38.
 *   Because of that, multiple sensors cannot share one active I2C bus directly.
 *   This firmware enables only one I2C1 pin pair at a time, reads that sensor,
 *   then switches to the next pin pair.
 *
 * 1602A LCD in 4-bit parallel mode, using CN9 D2~D7:
 *   VSS -> GND
 *   VDD -> 3V3 first. 5V may work, but 3V3 is safer for STM32 logic.
 *   V0  -> 10k potentiometer center pin for contrast
 *   GND -> GND
 *   RS  -> D2  / PA10
 *   RW  -> GND
 *   E   -> D3  / PB3
 *   D4  -> D4  / PB5
 *   D5  -> D5  / PB4
 *   D6  -> D6  / PB10
 *   D7  -> D7  / PA8
 *   A   -> 3V3 or 5V for backlight
 *   K   -> GND
 *
 * Servo motor:
 *   Orange signal -> D9 / PC7  (TIM3_CH2 PWM)
 *   Red VCC       -> external 5V servo power supply
 *   Brown GND     -> external power GND and Nucleo GND must be common
 *
 * SG90 wire colors:
 *   Orange -> PWM signal
 *   Red    -> 5V
 *   Brown  -> GND
 *
 * Do not power the servo from the Nucleo 5V pin while testing under load.
 * Use an external 5V supply, and connect its GND to Nucleo GND.
 *
 * Status LED for firmware-running check:
 *   If your board has a user LED on D13/PA5, it will blink automatically.
 *   If not, connect an external LED:
 *     D13 / PA5 -> 220 ohm resistor -> LED long leg(+)
 *     LED short leg(-) -> GND
 */

/*
 * Code module map
 *
 * 1. Hardware map and constants
 *    - Pin macros, humidity thresholds, servo pulse widths, timing values.
 *    - When wiring or test conditions change, this is the first area to check.
 *
 * 2. HAL peripheral handles and runtime state
 *    - hi2c1, htim3, huart2 are HAL objects for I2C, PWM timer, and UART.
 *    - dehumidifier_on and timestamp variables remember the current control state.
 *
 * 3. Small utility layer
 *    - delay_ms(), uart_print(), boot_mark(), number formatting helpers.
 *    - These keep debug output and timing code simple in the rest of the file.
 *
 * 4. Servo and dehumidifier actuator layer
 *    - servo_set_pulse_us() generates SG90 positions through TIM3_CH2 PWM.
 *    - servo_press_power_button() physically presses the dehumidifier button.
 *    - dehumidifier_set() updates the logical ON/OFF state after pressing.
 *
 * 5. Safety and timing policy layer
 *    - min_off_time_passed() and max_on_time_passed() decide whether another
 *      button press is allowed.
 *    - The 5-minute OFF wait is currently bypassed for servo testing.
 *
 * 6. AHT20 sensor driver layer
 *    - aht20_send_command(), aht20_read_status(), aht20_init(), aht20_read().
 *    - This talks directly to the sensor with HAL I2C, without an external library.
 *
 * 7. LCD 1602A parallel 4-bit driver layer
 *    - lcd_write_4bits(), lcd_send(), lcd_command(), lcd_data(), lcd_init().
 *    - This controls RS/E/D4~D7 GPIO pins directly.
 *
 * 8. Display/reporting layer
 *    - lcd_show_status() writes the user-facing state to the LCD.
 *    - serial_show_display_status() mirrors the LCD-style status to UART.
 *
 * 9. Main application flow
 *    - main() initializes HAL/peripherals, prints boot checkpoints, then loops.
 *    - Every SENSOR_READ_INTERVAL_MS, it reads humidity and applies ON/OFF logic.
 *
 * 10. STM32Cube HAL setup and interrupt glue
 *    - SystemClock_Config(), MX_*_Init(), HAL_*_MspInit(), Error_Handler().
 *    - SysTick_Handler() is required so HAL_Delay() and HAL_GetTick() work.
 */

/* 1. Hardware map and constants */

#define AHT20_I2C_ADDRESS        (0x38 << 1)  // AHT20 센서의 7비트 I2C 주소를 HAL 형식으로 왼쪽 시프트해 정의합니다.

#define SENSOR_COUNT             2U  // 사용할 습도 센서 개수를 정의합니다.

#define LCD_RS_GPIO_PORT         GPIOA  // LCD RS 핀의 GPIO 포트를 정의합니다.
#define LCD_RS_GPIO_PIN          GPIO_PIN_10  // LCD RS 핀 번호를 정의합니다.
#define LCD_E_GPIO_PORT          GPIOB  // LCD Enable 핀의 GPIO 포트를 정의합니다.
#define LCD_E_GPIO_PIN           GPIO_PIN_3  // LCD Enable 핀 번호를 정의합니다.
#define LCD_D4_GPIO_PORT         GPIOB  // LCD D4 데이터 핀의 GPIO 포트를 정의합니다.
#define LCD_D4_GPIO_PIN          GPIO_PIN_5  // LCD D4 데이터 핀 번호를 정의합니다.
#define LCD_D5_GPIO_PORT         GPIOB  // LCD D5 데이터 핀의 GPIO 포트를 정의합니다.
#define LCD_D5_GPIO_PIN          GPIO_PIN_4  // LCD D5 데이터 핀 번호를 정의합니다.
#define LCD_D6_GPIO_PORT         GPIOB  // LCD D6 데이터 핀의 GPIO 포트를 정의합니다.
#define LCD_D6_GPIO_PIN          GPIO_PIN_10  // LCD D6 데이터 핀 번호를 정의합니다.
#define LCD_D7_GPIO_PORT         GPIOA  // LCD D7 데이터 핀의 GPIO 포트를 정의합니다.
#define LCD_D7_GPIO_PIN          GPIO_PIN_8  // LCD D7 데이터 핀 번호를 정의합니다.

#define STATUS_LED_GPIO_PORT     GPIOA  // 상태 LED의 GPIO 포트를 정의합니다.
#define STATUS_LED_GPIO_PIN      GPIO_PIN_5  // 상태 LED의 GPIO 핀 번호를 정의합니다.

#define SERVO_REST_US            1000U  // 서보가 대기 위치에 있을 때의 PWM 펄스 폭입니다.
#define SERVO_PRESS_US           1800U  // 서보가 버튼을 누르는 위치의 PWM 펄스 폭입니다.
#define SERVO_PRESS_TIME_MS      700UL  // 서보가 버튼 누름 위치를 유지하는 시간입니다.
#define SERVO_SETTLE_TIME_MS     700UL  // 서보 이동 후 안정화 대기 시간입니다.
#define SERVO_TEST_ON_BOOT       1  // 부팅 시 서보 테스트 동작을 실행할지 결정합니다.

#define HUMIDITY_ON_PERCENT      65.0f  // 제습기를 켜는 기준 습도입니다.
#define HUMIDITY_OFF_PERCENT     55.0f  // 제습기를 끄는 기준 습도입니다.

#define SENSOR_READ_INTERVAL_MS  1000UL  // 습도 센서를 읽는 주기입니다.
#define MAX_ON_TIME_MS           (3UL * 60UL * 60UL * 1000UL)  // 제습기가 연속으로 켜져 있을 수 있는 최대 시간입니다.
#define MIN_OFF_TIME_MS          (5UL * 60UL * 1000UL)  // 제습기를 다시 켜기 전 최소 OFF 대기 시간입니다.

// Set to 1 while debugging upload/serial problems.
// In this mode, the firmware does not initialize I2C, LCD, sensor, or servo.
#define SERIAL_ONLY_TEST         0  // 시리얼 단독 테스트 모드 사용 여부입니다.

/* 2. HAL peripheral handles and runtime state */

I2C_HandleTypeDef hi2c1;  // I2C1 주변장치를 제어하기 위한 HAL 핸들입니다.
TIM_HandleTypeDef htim3;  // TIM3 타이머를 제어하기 위한 HAL 핸들입니다.
UART_HandleTypeDef huart2;  // USART2 시리얼 통신을 제어하기 위한 HAL 핸들입니다.

typedef struct {  // 센서별 I2C 핀 정보를 묶기 위한 구조체 선언을 시작합니다.
  GPIO_TypeDef *scl_port;  // GPIO 포트 주소를 저장하는 구조체 멤버입니다.
  uint16_t scl_pin;  // 16비트 부호 없는 값을 저장하는 변수를 선언합니다.
  GPIO_TypeDef *sda_port;  // GPIO 포트 주소를 저장하는 구조체 멤버입니다.
  uint16_t sda_pin;  // 16비트 부호 없는 값을 저장하는 변수를 선언합니다.
  const char *label;  // 변경하지 않을 상수 데이터를 선언합니다.
} SensorI2cBusConfig;  // 센서 I2C 버스 설정 구조체 타입 정의를 끝냅니다.

static const SensorI2cBusConfig sensor_i2c_buses[SENSOR_COUNT] = {  // 센서별 I2C 핀 설정 테이블을 선언합니다.
  {GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9, "SCL PB8, SDA PB9"},  // 센서 하나의 SCL/SDA 포트와 핀, 표시용 이름을 등록합니다.
  {GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7, "SCL PB6, SDA PB7"}  // 센서 하나의 SCL/SDA 포트와 핀, 표시용 이름을 등록합니다.
};  // 구조체나 배열 선언을 끝냅니다.

static bool dehumidifier_on = false;  // 현재 제습기를 켰다고 판단하는 논리 상태를 저장합니다.
static bool uart_ready = false;  // UART 출력 가능 여부를 저장합니다.
static uint32_t dehumidifier_turned_on_at = 0;  // 제습기를 켠 시각의 HAL tick 값을 저장합니다.
static uint32_t dehumidifier_turned_off_at = 0;  // 제습기를 끈 시각의 HAL tick 값을 저장합니다.

void SystemClock_Config(void);  // 시스템 클럭 설정 함수의 원형을 선언합니다.
static void MX_GPIO_Init(void);  // GPIO 초기화 함수의 원형을 선언합니다.
static bool MX_I2C1_Init(void);  // I2C1 초기화 함수의 원형을 선언합니다.
static void MX_TIM3_Init(void);  // TIM3 PWM 초기화 함수의 원형을 선언합니다.
static void MX_USART2_UART_Init(void);  // USART2 초기화 함수의 원형을 선언합니다.
static void Error_Handler(void);  // 오류 발생 시 멈추는 처리 함수의 원형을 선언합니다.

/* 3. Small utility layer */

static void delay_ms(uint32_t ms)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  HAL_Delay(ms);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
}  // 현재 실행 블록을 끝냅니다.

static void uart_print(const char *text)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  if (!uart_ready) {  // 조건이 참인지 검사합니다.
    return;  // 현재 함수를 여기서 종료합니다.
  }  // 현재 실행 블록을 끝냅니다.

  HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
}  // 현재 실행 블록을 끝냅니다.

static void boot_mark(const char *mark)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  uart_print(mark);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  uart_print("\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  delay_ms(200);  // 지정한 시간만큼 대기합니다.
}  // 현재 실행 블록을 끝냅니다.

static void uart_print_uint(uint32_t value)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  char buffer[11];  // 문자 배열 또는 문자 변수를 선언합니다.
  int index = 10;  // 정수형 변수를 선언합니다.

  buffer[index] = '\0';  // 변수에 값을 저장하거나 상태를 갱신합니다.

  if (value == 0) {  // 조건이 참인지 검사합니다.
    uart_print("0");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    return;  // 현재 함수를 여기서 종료합니다.
  }  // 현재 실행 블록을 끝냅니다.

  while (value > 0 && index > 0) {  // 조건이 참인 동안 반복합니다.
    index--;  // 한 줄짜리 C 문장을 실행합니다.
    buffer[index] = (char)('0' + (value % 10));  // 변수에 값을 저장하거나 상태를 갱신합니다.
    value /= 10;  // 변수에 값을 저장하거나 상태를 갱신합니다.
  }  // 현재 실행 블록을 끝냅니다.

  uart_print(&buffer[index]);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
}  // 현재 실행 블록을 끝냅니다.

static void uart_print_sensor_pin_plan(void)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  uint8_t sensor_index;  // 8비트 부호 없는 값을 저장하는 변수를 선언합니다.

  for (sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {  // 정해진 범위만큼 반복합니다.
    uart_print("[I2C] sensor ");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    uart_print_uint(sensor_index);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    uart_print(": ");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    uart_print(sensor_i2c_buses[sensor_index].label);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    uart_print("\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

static void uart_print_fixed_1(float value)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  int32_t scaled = (int32_t)(value * 10.0f);  // 32비트 부호 있는 값을 저장하는 변수를 선언합니다.

  if (scaled < 0) {  // 조건이 참인지 검사합니다.
    uart_print("-");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    scaled = -scaled;  // 변수에 값을 저장하거나 상태를 갱신합니다.
  }  // 현재 실행 블록을 끝냅니다.

  uart_print_uint((uint32_t)(scaled / 10));  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  uart_print(".");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  uart_print_uint((uint32_t)(scaled % 10));  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
}  // 현재 실행 블록을 끝냅니다.

/* 4. Servo and dehumidifier actuator layer */

static void servo_set_pulse_us(uint16_t pulse_us)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
}  // 현재 실행 블록을 끝냅니다.

static void servo_press_power_button(void)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  servo_set_pulse_us(SERVO_REST_US);  // 서보모터 위치나 버튼 누름 동작을 수행합니다.
  delay_ms(SERVO_SETTLE_TIME_MS);  // 지정한 시간만큼 대기합니다.
  servo_set_pulse_us(SERVO_PRESS_US);  // 서보모터 위치나 버튼 누름 동작을 수행합니다.
  delay_ms(SERVO_PRESS_TIME_MS);  // 지정한 시간만큼 대기합니다.
  servo_set_pulse_us(SERVO_REST_US);  // 서보모터 위치나 버튼 누름 동작을 수행합니다.
  delay_ms(SERVO_SETTLE_TIME_MS);  // 지정한 시간만큼 대기합니다.
}  // 현재 실행 블록을 끝냅니다.

static void dehumidifier_set(bool on)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  if (dehumidifier_on == on) {  // 조건이 참인지 검사합니다.
    return;  // 현재 함수를 여기서 종료합니다.
  }  // 현재 실행 블록을 끝냅니다.

  servo_press_power_button();  // 서보모터 위치나 버튼 누름 동작을 수행합니다.
  dehumidifier_on = on;  // 변수에 값을 저장하거나 상태를 갱신합니다.

  if (on) {  // 조건이 참인지 검사합니다.
    dehumidifier_turned_on_at = HAL_GetTick();  // 변수에 값을 저장하거나 상태를 갱신합니다.
    uart_print("[SERVO] Power button pressed. State: ON\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  } else {  // 앞 조건이 거짓일 때 실행할 블록입니다.
    dehumidifier_turned_off_at = HAL_GetTick();  // 변수에 값을 저장하거나 상태를 갱신합니다.
    uart_print("[SERVO] Power button pressed. State: OFF\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

/* 5. Safety and timing policy layer */

static bool min_off_time_passed(void)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  // Servo operation test mode:
  // Ignore the 5-minute minimum OFF time so repeated button presses are easy to test.
  return true;  // 함수 결과값을 반환합니다.

  /*
  if (dehumidifier_turned_off_at == 0) {
    return true;
  }

  return (HAL_GetTick() - dehumidifier_turned_off_at) >= MIN_OFF_TIME_MS;
  */
}  // 현재 실행 블록을 끝냅니다.

static bool max_on_time_passed(void)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  return dehumidifier_on && ((HAL_GetTick() - dehumidifier_turned_on_at) >= MAX_ON_TIME_MS);  // 함수 결과값을 반환합니다.
}  // 현재 실행 블록을 끝냅니다.

static void sensor_i2c_release_bus_pins(uint8_t sensor_index)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  GPIO_InitTypeDef GPIO_InitStruct = {0};  // 변수에 값을 저장하거나 상태를 갱신합니다.
  const SensorI2cBusConfig *bus = &sensor_i2c_buses[sensor_index];  // 변경하지 않을 상수 데이터를 선언합니다.

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // GPIO 초기화 구조체의 설정값을 지정합니다.

  if (bus->scl_port == bus->sda_port) {  // 조건이 참인지 검사합니다.
    GPIO_InitStruct.Pin = bus->scl_pin | bus->sda_pin;  // GPIO 초기화 구조체의 설정값을 지정합니다.
    HAL_GPIO_Init(bus->scl_port, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  } else {  // 앞 조건이 거짓일 때 실행할 블록입니다.
    GPIO_InitStruct.Pin = bus->scl_pin;  // GPIO 초기화 구조체의 설정값을 지정합니다.
    HAL_GPIO_Init(bus->scl_port, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

    GPIO_InitStruct.Pin = bus->sda_pin;  // GPIO 초기화 구조체의 설정값을 지정합니다.
    HAL_GPIO_Init(bus->sda_port, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

static void sensor_i2c_release_all_pins(void)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  uint8_t sensor_index;  // 8비트 부호 없는 값을 저장하는 변수를 선언합니다.

  for (sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {  // 정해진 범위만큼 반복합니다.
    sensor_i2c_release_bus_pins(sensor_index);  // 선택한 센서의 I2C 핀 쌍을 준비하거나 전환합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

static void sensor_i2c_configure_bus_pins(uint8_t sensor_index)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  GPIO_InitTypeDef GPIO_InitStruct = {0};  // 변수에 값을 저장하거나 상태를 갱신합니다.
  const SensorI2cBusConfig *bus = &sensor_i2c_buses[sensor_index];  // 변경하지 않을 상수 데이터를 선언합니다.

  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Pull = GPIO_PULLUP;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;  // GPIO 초기화 구조체의 설정값을 지정합니다.

  if (bus->scl_port == bus->sda_port) {  // 조건이 참인지 검사합니다.
    GPIO_InitStruct.Pin = bus->scl_pin | bus->sda_pin;  // GPIO 초기화 구조체의 설정값을 지정합니다.
    HAL_GPIO_Init(bus->scl_port, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  } else {  // 앞 조건이 거짓일 때 실행할 블록입니다.
    GPIO_InitStruct.Pin = bus->scl_pin;  // GPIO 초기화 구조체의 설정값을 지정합니다.
    HAL_GPIO_Init(bus->scl_port, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

    GPIO_InitStruct.Pin = bus->sda_pin;  // GPIO 초기화 구조체의 설정값을 지정합니다.
    HAL_GPIO_Init(bus->sda_port, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

static bool sensor_i2c_select(uint8_t sensor_index)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  if (sensor_index >= SENSOR_COUNT) {  // 조건이 참인지 검사합니다.
    return false;  // 함수 결과값을 반환합니다.
  }  // 현재 실행 블록을 끝냅니다.

  if (hi2c1.Instance == I2C1) {  // 조건이 참인지 검사합니다.
    HAL_I2C_DeInit(&hi2c1);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  }  // 현재 실행 블록을 끝냅니다.
  sensor_i2c_release_all_pins();  // 선택한 센서의 I2C 핀 쌍을 준비하거나 전환합니다.
  sensor_i2c_configure_bus_pins(sensor_index);  // 선택한 센서의 I2C 핀 쌍을 준비하거나 전환합니다.
  delay_ms(2);  // 지정한 시간만큼 대기합니다.

  return MX_I2C1_Init();  // 함수 결과값을 반환합니다.
}  // 현재 실행 블록을 끝냅니다.

/* 6. AHT20 sensor driver layer */

static bool aht20_send_command(uint8_t command, uint8_t data0, uint8_t data1)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  uint8_t tx_data[3] = {command, data0, data1};  // 8비트 부호 없는 값을 저장하는 변수를 선언합니다.

  return HAL_I2C_Master_Transmit(&hi2c1, AHT20_I2C_ADDRESS, tx_data, 3, 100) == HAL_OK;  // 함수 결과값을 반환합니다.
}  // 현재 실행 블록을 끝냅니다.

static bool aht20_read_status(uint8_t *status)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  return HAL_I2C_Master_Receive(&hi2c1, AHT20_I2C_ADDRESS, status, 1, 100) == HAL_OK;  // 함수 결과값을 반환합니다.
}  // 현재 실행 블록을 끝냅니다.

static bool aht20_init(void)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  uint8_t status = 0;  // 8비트 부호 없는 값을 저장하는 변수를 선언합니다.

  delay_ms(40);  // 지정한 시간만큼 대기합니다.

  if (!aht20_read_status(&status)) {  // 조건이 참인지 검사합니다.
    return false;  // 함수 결과값을 반환합니다.
  }  // 현재 실행 블록을 끝냅니다.

  if ((status & 0x08) == 0) {  // 조건이 참인지 검사합니다.
    if (!aht20_send_command(0xBE, 0x08, 0x00)) {  // 조건이 참인지 검사합니다.
      return false;  // 함수 결과값을 반환합니다.
    }  // 현재 실행 블록을 끝냅니다.
    delay_ms(10);  // 지정한 시간만큼 대기합니다.
  }  // 현재 실행 블록을 끝냅니다.

  return true;  // 함수 결과값을 반환합니다.
}  // 현재 실행 블록을 끝냅니다.

static bool aht20_read(float *humidity_percent, float *temperature_c)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  uint8_t rx_data[6];  // 8비트 부호 없는 값을 저장하는 변수를 선언합니다.
  uint32_t humidity_raw;  // 32비트 부호 없는 값을 저장하는 변수를 선언합니다.
  uint32_t temperature_raw;  // 32비트 부호 없는 값을 저장하는 변수를 선언합니다.
  uint32_t start_tick;  // 32비트 부호 없는 값을 저장하는 변수를 선언합니다.

  if (!aht20_send_command(0xAC, 0x33, 0x00)) {  // 조건이 참인지 검사합니다.
    return false;  // 함수 결과값을 반환합니다.
  }  // 현재 실행 블록을 끝냅니다.

  delay_ms(80);  // 지정한 시간만큼 대기합니다.
  start_tick = HAL_GetTick();  // 변수에 값을 저장하거나 상태를 갱신합니다.

  do {  // 최소 한 번 실행되는 반복문을 시작합니다.
    if (HAL_I2C_Master_Receive(&hi2c1, AHT20_I2C_ADDRESS, rx_data, 6, 100) != HAL_OK) {  // 조건이 참인지 검사합니다.
      return false;  // 함수 결과값을 반환합니다.
    }  // 현재 실행 블록을 끝냅니다.

    if ((rx_data[0] & 0x80) == 0) {  // 조건이 참인지 검사합니다.
      break;  // 현재 반복문을 빠져나옵니다.
    }  // 현재 실행 블록을 끝냅니다.
  } while ((HAL_GetTick() - start_tick) < 200);  // 함수를 호출해 필요한 작업을 수행합니다.

  if ((rx_data[0] & 0x80) != 0) {  // 조건이 참인지 검사합니다.
    return false;  // 함수 결과값을 반환합니다.
  }  // 현재 실행 블록을 끝냅니다.

  humidity_raw = ((uint32_t)rx_data[1] << 12) |  // 코드 블록의 일부입니다.
                 ((uint32_t)rx_data[2] << 4) |  // 코드 블록의 일부입니다.
                 ((uint32_t)rx_data[3] >> 4);  // 함수를 호출해 필요한 작업을 수행합니다.

  temperature_raw = (((uint32_t)rx_data[3] & 0x0F) << 16) |  // 코드 블록의 일부입니다.
                    ((uint32_t)rx_data[4] << 8) |  // 코드 블록의 일부입니다.
                    (uint32_t)rx_data[5];  // 한 줄짜리 C 문장을 실행합니다.

  *humidity_percent = ((float)humidity_raw * 100.0f) / 1048576.0f;  // 변수에 값을 저장하거나 상태를 갱신합니다.
  *temperature_c = (((float)temperature_raw * 200.0f) / 1048576.0f) - 50.0f;  // 변수에 값을 저장하거나 상태를 갱신합니다.

  return true;  // 함수 결과값을 반환합니다.
}  // 현재 실행 블록을 끝냅니다.

static bool aht20_init_sensor(uint8_t sensor_index)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  if (!sensor_i2c_select(sensor_index)) {  // 조건이 참인지 검사합니다.
    return false;  // 함수 결과값을 반환합니다.
  }  // 현재 실행 블록을 끝냅니다.

  return aht20_init();  // 함수 결과값을 반환합니다.
}  // 현재 실행 블록을 끝냅니다.

static bool aht20_read_sensor(uint8_t sensor_index, float *humidity_percent, float *temperature_c)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  if (!sensor_i2c_select(sensor_index)) {  // 조건이 참인지 검사합니다.
    return false;  // 함수 결과값을 반환합니다.
  }  // 현재 실행 블록을 끝냅니다.

  return aht20_read(humidity_percent, temperature_c);  // 함수 결과값을 반환합니다.
}  // 현재 실행 블록을 끝냅니다.

/* 7. LCD 1602A parallel 4-bit driver layer */

static void lcd_enable_pulse(void)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  HAL_GPIO_WritePin(LCD_E_GPIO_PORT, LCD_E_GPIO_PIN, GPIO_PIN_SET);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  delay_ms(1);  // 지정한 시간만큼 대기합니다.
  HAL_GPIO_WritePin(LCD_E_GPIO_PORT, LCD_E_GPIO_PIN, GPIO_PIN_RESET);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  delay_ms(1);  // 지정한 시간만큼 대기합니다.
}  // 현재 실행 블록을 끝냅니다.

static void lcd_write_4bits(uint8_t nibble)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  HAL_GPIO_WritePin(LCD_D4_GPIO_PORT, LCD_D4_GPIO_PIN, (nibble & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  HAL_GPIO_WritePin(LCD_D5_GPIO_PORT, LCD_D5_GPIO_PIN, (nibble & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  HAL_GPIO_WritePin(LCD_D6_GPIO_PORT, LCD_D6_GPIO_PIN, (nibble & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  HAL_GPIO_WritePin(LCD_D7_GPIO_PORT, LCD_D7_GPIO_PIN, (nibble & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  lcd_enable_pulse();  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
}  // 현재 실행 블록을 끝냅니다.

static void lcd_send(uint8_t value, bool is_data)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  HAL_GPIO_WritePin(LCD_RS_GPIO_PORT, LCD_RS_GPIO_PIN, is_data ? GPIO_PIN_SET : GPIO_PIN_RESET);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  lcd_write_4bits(value >> 4);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_write_4bits(value & 0x0F);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
}  // 현재 실행 블록을 끝냅니다.

static void lcd_command(uint8_t command)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  lcd_send(command, false);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  if (command == 0x01 || command == 0x02) {  // 조건이 참인지 검사합니다.
    delay_ms(2);  // 지정한 시간만큼 대기합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

static void lcd_data(uint8_t data)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  lcd_send(data, true);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
}  // 현재 실행 블록을 끝냅니다.

static void lcd_print(const char *text)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  while (*text != '\0') {  // 조건이 참인 동안 반복합니다.
    lcd_data((uint8_t)*text);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
    text++;  // 한 줄짜리 C 문장을 실행합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

static void lcd_print_uint(uint32_t value)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  char buffer[11];  // 문자 배열 또는 문자 변수를 선언합니다.
  int index = 10;  // 정수형 변수를 선언합니다.

  buffer[index] = '\0';  // 변수에 값을 저장하거나 상태를 갱신합니다.

  if (value == 0) {  // 조건이 참인지 검사합니다.
    lcd_data('0');  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
    return;  // 현재 함수를 여기서 종료합니다.
  }  // 현재 실행 블록을 끝냅니다.

  while (value > 0 && index > 0) {  // 조건이 참인 동안 반복합니다.
    index--;  // 한 줄짜리 C 문장을 실행합니다.
    buffer[index] = (char)('0' + (value % 10));  // 변수에 값을 저장하거나 상태를 갱신합니다.
    value /= 10;  // 변수에 값을 저장하거나 상태를 갱신합니다.
  }  // 현재 실행 블록을 끝냅니다.

  lcd_print(&buffer[index]);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
}  // 현재 실행 블록을 끝냅니다.

static void lcd_init(void)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  delay_ms(50);  // 지정한 시간만큼 대기합니다.

  HAL_GPIO_WritePin(LCD_RS_GPIO_PORT, LCD_RS_GPIO_PIN, GPIO_PIN_RESET);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  HAL_GPIO_WritePin(LCD_E_GPIO_PORT, LCD_E_GPIO_PIN, GPIO_PIN_RESET);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  lcd_write_4bits(0x03);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  delay_ms(5);  // 지정한 시간만큼 대기합니다.
  lcd_write_4bits(0x03);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  delay_ms(5);  // 지정한 시간만큼 대기합니다.
  lcd_write_4bits(0x03);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  delay_ms(1);  // 지정한 시간만큼 대기합니다.
  lcd_write_4bits(0x02);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  delay_ms(1);  // 지정한 시간만큼 대기합니다.

  lcd_command(0x28);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_command(0x0C);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_command(0x06);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_command(0x01);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  delay_ms(2);  // 지정한 시간만큼 대기합니다.
}  // 현재 실행 블록을 끝냅니다.

static void lcd_set_cursor(uint8_t column, uint8_t row)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  const uint8_t row_offsets[] = {0x00, 0x40};  // 변경하지 않을 상수 데이터를 선언합니다.
  lcd_command((uint8_t)(0x80 | (column + row_offsets[row])));  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
}  // 현재 실행 블록을 끝냅니다.

static void lcd_clear_line(uint8_t row)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  uint8_t i;  // 8비트 부호 없는 값을 저장하는 변수를 선언합니다.

  lcd_set_cursor(0, row);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  for (i = 0; i < 16; i++) {  // 정해진 범위만큼 반복합니다.
    lcd_data(' ');  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

/* 8. Display/reporting layer */

static void lcd_show_status(float humidity)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  uint32_t humidity_ones = (uint32_t)(humidity + 0.5f);  // 32비트 부호 없는 값을 저장하는 변수를 선언합니다.

  lcd_clear_line(0);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_set_cursor(0, 0);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_print("Humidity: ");  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_print_uint(humidity_ones);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_print("%");  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.

  lcd_clear_line(1);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_set_cursor(0, 1);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_print("Power: ");  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_print(dehumidifier_on ? "ON" : "OFF");  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
}  // 현재 실행 블록을 끝냅니다.

static void serial_show_display_status(float humidity)  // 파일 내부에서만 사용하는 함수 정의를 시작합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  uint32_t humidity_ones = (uint32_t)(humidity + 0.5f);  // 32비트 부호 없는 값을 저장하는 변수를 선언합니다.

  uart_print("[DISPLAY] Humidity: ");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  uart_print_uint(humidity_ones);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  uart_print("%, Power: ");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  uart_print(dehumidifier_on ? "ON" : "OFF");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  uart_print("\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
}  // 현재 실행 블록을 끝냅니다.

/* 9. Main application flow */

int main(void)  // 프로그램의 시작점인 main 함수입니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  bool aht20_ready[SENSOR_COUNT] = {false};  // 참/거짓 상태를 저장하는 변수를 선언합니다.
  uint32_t last_sensor_read_at = 0;  // 32비트 부호 없는 값을 저장하는 변수를 선언합니다.
  uint8_t sensor_index;  // 8비트 부호 없는 값을 저장하는 변수를 선언합니다.

  HAL_Init();  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  SystemClock_Config();  // 주변장치 또는 시스템 설정 함수를 호출합니다.

  MX_GPIO_Init();  // 주변장치 또는 시스템 설정 함수를 호출합니다.
  MX_USART2_UART_Init();  // 주변장치 또는 시스템 설정 함수를 호출합니다.
  uart_ready = true;  // 변수에 값을 저장하거나 상태를 갱신합니다.

  boot_mark("B0 UART");  // 부팅 진행 단계 표시 로그를 출력합니다.
  uart_print_sensor_pin_plan();  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.

#if SERIAL_ONLY_TEST  // 전처리 조건에 따라 이 아래 코드를 포함할지 결정합니다.
  while (1) {  // 조건이 참인 동안 반복합니다.
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_GPIO_PIN);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
    uart_print("[TEST] Serial alive\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    delay_ms(1000);  // 지정한 시간만큼 대기합니다.
  }  // 현재 실행 블록을 끝냅니다.
#endif  // 전처리 조건 블록을 끝냅니다.

  sensor_i2c_select(0);  // 선택한 센서의 I2C 핀 쌍을 준비하거나 전환합니다.
  boot_mark("B1 I2C");  // 부팅 진행 단계 표시 로그를 출력합니다.

  MX_TIM3_Init();  // 주변장치 또는 시스템 설정 함수를 호출합니다.
  boot_mark("B2 TIM3");  // 부팅 진행 단계 표시 로그를 출력합니다.

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  servo_set_pulse_us(SERVO_REST_US);  // 서보모터 위치나 버튼 누름 동작을 수행합니다.
  boot_mark("B3 PWM");  // 부팅 진행 단계 표시 로그를 출력합니다.

#if SERVO_TEST_ON_BOOT  // 전처리 조건에 따라 이 아래 코드를 포함할지 결정합니다.
  boot_mark("B3S SERVO");  // 부팅 진행 단계 표시 로그를 출력합니다.
  servo_press_power_button();  // 서보모터 위치나 버튼 누름 동작을 수행합니다.
#endif  // 전처리 조건 블록을 끝냅니다.

  lcd_init();  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  boot_mark("B4 LCD");  // 부팅 진행 단계 표시 로그를 출력합니다.
  lcd_clear_line(0);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_set_cursor(0, 0);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_print("Humidity Ready");  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_clear_line(1);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_set_cursor(0, 1);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  lcd_print("Power: OFF");  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.

  boot_mark("B5 AHT");  // 부팅 진행 단계 표시 로그를 출력합니다.
  for (sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {  // 정해진 범위만큼 반복합니다.
    aht20_ready[sensor_index] = aht20_init_sensor(sensor_index);  // AHT20 센서를 초기화하거나 데이터를 읽습니다.

    uart_print("[AHT20-");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    uart_print_uint(sensor_index);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    uart_print("] ");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    uart_print(aht20_ready[sensor_index] ? "OK\r\n" : "NOT FOUND\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  }  // 현재 실행 블록을 끝냅니다.

  while (1) {  // 조건이 참인 동안 반복합니다.
    bool has_valid_reading = false;  // 참/거짓 상태를 저장하는 변수를 선언합니다.
    float control_humidity = 0.0f;  // 실수형 값을 저장하는 변수를 선언합니다.

    if ((HAL_GetTick() - last_sensor_read_at) < SENSOR_READ_INTERVAL_MS) {  // 조건이 참인지 검사합니다.
      continue;  // 이번 반복의 남은 처리를 건너뜁니다.
    }  // 현재 실행 블록을 끝냅니다.
    last_sensor_read_at = HAL_GetTick();  // 변수에 값을 저장하거나 상태를 갱신합니다.
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_GPIO_PIN);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

    for (sensor_index = 0; sensor_index < SENSOR_COUNT; sensor_index++) {  // 정해진 범위만큼 반복합니다.
      float humidity = 0.0f;  // 실수형 값을 저장하는 변수를 선언합니다.
      float temperature = 0.0f;  // 실수형 값을 저장하는 변수를 선언합니다.

      if (!aht20_ready[sensor_index]) {  // 조건이 참인지 검사합니다.
        continue;  // 이번 반복의 남은 처리를 건너뜁니다.
      }  // 현재 실행 블록을 끝냅니다.

      if (!aht20_read_sensor(sensor_index, &humidity, &temperature)) {  // 조건이 참인지 검사합니다.
        uart_print("[SAFE] AHT20-");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
        uart_print_uint(sensor_index);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
        uart_print(" read failed.\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
        continue;  // 이번 반복의 남은 처리를 건너뜁니다.
      }  // 현재 실행 블록을 끝냅니다.

      uart_print("[AHT20-");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
      uart_print_uint(sensor_index);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
      uart_print("] Humidity: ");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
      uart_print_fixed_1(humidity);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
      uart_print("%, Temp: ");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
      uart_print_fixed_1(temperature);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
      uart_print(" C\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.

      if (!has_valid_reading || humidity > control_humidity) {  // 조건이 참인지 검사합니다.
        control_humidity = humidity;  // 변수에 값을 저장하거나 상태를 갱신합니다.
      }  // 현재 실행 블록을 끝냅니다.
      has_valid_reading = true;  // 변수에 값을 저장하거나 상태를 갱신합니다.
    }  // 현재 실행 블록을 끝냅니다.

    if (!has_valid_reading) {  // 조건이 참인지 검사합니다.
      uart_print("[SAFE] No humidity sensor reading. Servo will not press button.\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
      continue;  // 이번 반복의 남은 처리를 건너뜁니다.
    }  // 현재 실행 블록을 끝냅니다.

    uart_print("[CONTROL] Using highest humidity: ");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    uart_print_fixed_1(control_humidity);  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    uart_print("%\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.

    lcd_show_status(control_humidity);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
    serial_show_display_status(control_humidity);  // 함수를 호출해 필요한 작업을 수행합니다.

    if (dehumidifier_on) {  // 조건이 참인지 검사합니다.
      if (control_humidity <= HUMIDITY_OFF_PERCENT) {  // 조건이 참인지 검사합니다.
        uart_print("[AUTO] Humidity is low enough.\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
        dehumidifier_set(false);  // 함수를 호출해 필요한 작업을 수행합니다.
      } else if (max_on_time_passed()) {  // 앞 조건이 거짓일 때 추가 조건을 검사합니다.
        uart_print("[AUTO] Maximum ON time reached.\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
        dehumidifier_set(false);  // 함수를 호출해 필요한 작업을 수행합니다.
      }  // 현재 실행 블록을 끝냅니다.
    } else {  // 앞 조건이 거짓일 때 실행할 블록입니다.
      if (control_humidity >= HUMIDITY_ON_PERCENT && min_off_time_passed()) {  // 조건이 참인지 검사합니다.
        uart_print("[AUTO] Humidity is high.\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
        dehumidifier_set(true);  // 함수를 호출해 필요한 작업을 수행합니다.
      } else if (control_humidity >= HUMIDITY_ON_PERCENT) {  // 앞 조건이 거짓일 때 추가 조건을 검사합니다.
        uart_print("[WAIT] Minimum OFF time has not passed yet.\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
      }  // 현재 실행 블록을 끝냅니다.
    }  // 현재 실행 블록을 끝냅니다.

    lcd_show_status(control_humidity);  // LCD에 명령이나 데이터를 보내 화면을 갱신합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

/* 10. STM32Cube HAL setup and interrupt glue */

void SystemClock_Config(void)  // 보드의 시스템 클럭을 설정하는 함수입니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // 클럭 또는 전원 관련 설정값을 지정합니다.

  __HAL_RCC_PWR_CLK_ENABLE();  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_OscInitStruct.PLL.PLLM = 16;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_OscInitStruct.PLL.PLLN = 336;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_OscInitStruct.PLL.PLLQ = 7;  // 클럭 또는 전원 관련 설정값을 지정합니다.

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {  // 조건이 참인지 검사합니다.
    Error_Handler();  // 함수를 호출해 필요한 작업을 수행합니다.
  }  // 현재 실행 블록을 끝냅니다.

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |  // 클럭 또는 전원 관련 설정값을 지정합니다.
                                RCC_CLOCKTYPE_SYSCLK |  // 클럭 또는 전원 관련 설정값을 지정합니다.
                                RCC_CLOCKTYPE_PCLK1 |  // 클럭 또는 전원 관련 설정값을 지정합니다.
                                RCC_CLOCKTYPE_PCLK2;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  // 클럭 또는 전원 관련 설정값을 지정합니다.
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  // 클럭 또는 전원 관련 설정값을 지정합니다.

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {  // 조건이 참인지 검사합니다.
    Error_Handler();  // 함수를 호출해 필요한 작업을 수행합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

static bool MX_I2C1_Init(void)  // I2C1 통신 설정을 초기화하고 성공 여부를 반환합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  HAL_StatusTypeDef result;  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  uart_print("[I2C] init start\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.

  hi2c1.Instance = I2C1;  // I2C1 설정 항목에 값을 넣습니다.
  hi2c1.Init.ClockSpeed = 100000;  // I2C1 설정 항목에 값을 넣습니다.
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;  // I2C1 설정 항목에 값을 넣습니다.
  hi2c1.Init.OwnAddress1 = 0;  // I2C1 설정 항목에 값을 넣습니다.
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // I2C1 설정 항목에 값을 넣습니다.
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;  // I2C1 설정 항목에 값을 넣습니다.
  hi2c1.Init.OwnAddress2 = 0;  // I2C1 설정 항목에 값을 넣습니다.
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;  // I2C1 설정 항목에 값을 넣습니다.
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;  // I2C1 설정 항목에 값을 넣습니다.

  result = HAL_I2C_Init(&hi2c1);  // 변수에 값을 저장하거나 상태를 갱신합니다.
  if (result != HAL_OK) {  // 조건이 참인지 검사합니다.
    uart_print("[I2C] init failed\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    return false;  // 함수 결과값을 반환합니다.
  }  // 현재 실행 블록을 끝냅니다.

  uart_print("[I2C] init ok\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
  return true;  // 함수 결과값을 반환합니다.
}  // 현재 실행 블록을 끝냅니다.

static void MX_USART2_UART_Init(void)  // USART2 시리얼 통신 설정을 초기화합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  huart2.Instance = USART2;  // USART2 설정 항목에 값을 넣습니다.
  huart2.Init.BaudRate = 115200;  // USART2 설정 항목에 값을 넣습니다.
  huart2.Init.WordLength = UART_WORDLENGTH_8B;  // USART2 설정 항목에 값을 넣습니다.
  huart2.Init.StopBits = UART_STOPBITS_1;  // USART2 설정 항목에 값을 넣습니다.
  huart2.Init.Parity = UART_PARITY_NONE;  // USART2 설정 항목에 값을 넣습니다.
  huart2.Init.Mode = UART_MODE_TX_RX;  // USART2 설정 항목에 값을 넣습니다.
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;  // USART2 설정 항목에 값을 넣습니다.
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;  // USART2 설정 항목에 값을 넣습니다.

  if (HAL_UART_Init(&huart2) != HAL_OK) {  // 조건이 참인지 검사합니다.
    Error_Handler();  // 함수를 호출해 필요한 작업을 수행합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

static void MX_TIM3_Init(void)  // TIM3 PWM 설정을 초기화합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  TIM_OC_InitTypeDef sConfigOC = {0};  // 변수에 값을 저장하거나 상태를 갱신합니다.
  HAL_StatusTypeDef result;  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  uart_print("[TIM3] init start\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.

  htim3.Instance = TIM3;  // TIM3 설정 항목에 값을 넣습니다.
  htim3.Init.Prescaler = 84 - 1;  // TIM3 설정 항목에 값을 넣습니다.
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;  // TIM3 설정 항목에 값을 넣습니다.
  htim3.Init.Period = 20000 - 1;  // TIM3 설정 항목에 값을 넣습니다.
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // TIM3 설정 항목에 값을 넣습니다.
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  // TIM3 설정 항목에 값을 넣습니다.

  result = HAL_TIM_PWM_Init(&htim3);  // 변수에 값을 저장하거나 상태를 갱신합니다.
  if (result != HAL_OK) {  // 조건이 참인지 검사합니다.
    uart_print("[TIM3] pwm init failed\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    return;  // 현재 함수를 여기서 종료합니다.
  }  // 현재 실행 블록을 끝냅니다.

  sConfigOC.OCMode = TIM_OCMODE_PWM1;  // PWM 출력 채널 설정값을 지정합니다.
  sConfigOC.Pulse = SERVO_REST_US;  // PWM 출력 채널 설정값을 지정합니다.
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;  // PWM 출력 채널 설정값을 지정합니다.
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;  // PWM 출력 채널 설정값을 지정합니다.

  result = HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);  // 변수에 값을 저장하거나 상태를 갱신합니다.
  if (result != HAL_OK) {  // 조건이 참인지 검사합니다.
    uart_print("[TIM3] channel init failed\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
    return;  // 현재 함수를 여기서 종료합니다.
  }  // 현재 실행 블록을 끝냅니다.

  uart_print("[TIM3] init ok\r\n");  // UART 시리얼 모니터로 문자열이나 값을 출력합니다.
}  // 현재 실행 블록을 끝냅니다.

static void MX_GPIO_Init(void)  // GPIO 핀들의 입력/출력/대체기능 설정을 초기화합니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  GPIO_InitTypeDef GPIO_InitStruct = {0};  // 변수에 값을 저장하거나 상태를 갱신합니다.

  __HAL_RCC_GPIOA_CLK_ENABLE();  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  __HAL_RCC_GPIOB_CLK_ENABLE();  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  __HAL_RCC_GPIOC_CLK_ENABLE();  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  GPIO_InitStruct.Pin = STATUS_LED_GPIO_PIN;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  HAL_GPIO_Init(STATUS_LED_GPIO_PORT, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  sensor_i2c_release_all_pins();  // 선택한 센서의 I2C 핀 쌍을 준비하거나 전환합니다.

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Pull = GPIO_PULLUP;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_10;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  GPIO_InitStruct.Pin = GPIO_PIN_7;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
}  // 현재 실행 블록을 끝냅니다.

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle)  // HAL이 I2C 초기화 중 호출하는 저수준 클럭 설정 함수입니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  if (i2cHandle->Instance == I2C1) {  // 조건이 참인지 검사합니다.
    __HAL_RCC_I2C1_CLK_ENABLE();  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)  // HAL이 UART 초기화 중 호출하는 저수준 클럭 설정 함수입니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  if (uartHandle->Instance == USART2) {  // 조건이 참인지 검사합니다.
    __HAL_RCC_USART2_CLK_ENABLE();  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *timHandle)  // HAL이 PWM 타이머 초기화 중 호출하는 저수준 클럭 설정 함수입니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  if (timHandle->Instance == TIM3) {  // 조건이 참인지 검사합니다.
    __HAL_RCC_TIM3_CLK_ENABLE();  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

static void Error_Handler(void)  // 치명적 오류 발생 시 LED를 깜빡이며 무한 대기하는 함수입니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  GPIO_InitTypeDef GPIO_InitStruct = {0};  // 변수에 값을 저장하거나 상태를 갱신합니다.

  __disable_irq();  // 함수를 호출해 필요한 작업을 수행합니다.

  __HAL_RCC_GPIOA_CLK_ENABLE();  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  GPIO_InitStruct.Pin = STATUS_LED_GPIO_PIN;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Pull = GPIO_NOPULL;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;  // GPIO 초기화 구조체의 설정값을 지정합니다.
  HAL_GPIO_Init(STATUS_LED_GPIO_PORT, &GPIO_InitStruct);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.

  while (1) {  // 조건이 참인 동안 반복합니다.
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_GPIO_PIN);  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
    for (volatile uint32_t i = 0; i < 800000; i++) {  // 정해진 범위만큼 반복합니다.
    }  // 현재 실행 블록을 끝냅니다.
  }  // 현재 실행 블록을 끝냅니다.
}  // 현재 실행 블록을 끝냅니다.

void SysTick_Handler(void)  // SysTick 인터럽트가 발생할 때 HAL tick을 증가시키는 함수입니다.
{  // 함수나 제어문의 실행 블록을 시작합니다.
  HAL_IncTick();  // STM32 HAL 함수를 호출해 하드웨어를 설정하거나 제어합니다.
}  // 현재 실행 블록을 끝냅니다.
