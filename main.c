#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * Nucleo-F411RE pin plan
 *
 * AHT20/BMP280 module:
 *   VCC -> 3V3
 *   GND -> GND
 *   SDA -> PB9  (Arduino D14)
 *   SCL -> PB8  (Arduino D15)
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

#define AHT20_I2C_ADDRESS        (0x38 << 1)

#define LCD_RS_GPIO_PORT         GPIOA
#define LCD_RS_GPIO_PIN          GPIO_PIN_10
#define LCD_E_GPIO_PORT          GPIOB
#define LCD_E_GPIO_PIN           GPIO_PIN_3
#define LCD_D4_GPIO_PORT         GPIOB
#define LCD_D4_GPIO_PIN          GPIO_PIN_5
#define LCD_D5_GPIO_PORT         GPIOB
#define LCD_D5_GPIO_PIN          GPIO_PIN_4
#define LCD_D6_GPIO_PORT         GPIOB
#define LCD_D6_GPIO_PIN          GPIO_PIN_10
#define LCD_D7_GPIO_PORT         GPIOA
#define LCD_D7_GPIO_PIN          GPIO_PIN_8

#define STATUS_LED_GPIO_PORT     GPIOA
#define STATUS_LED_GPIO_PIN      GPIO_PIN_5

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

// Set to 1 while debugging upload/serial problems.
// In this mode, the firmware does not initialize I2C, LCD, sensor, or servo.
#define SERIAL_ONLY_TEST         0

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

static bool dehumidifier_on = false;
static bool uart_ready = false;
static uint32_t dehumidifier_turned_on_at = 0;
static uint32_t dehumidifier_turned_off_at = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void Error_Handler(void);

static void delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

static void uart_print(const char *text)
{
  if (!uart_ready) {
    return;
  }

  HAL_UART_Transmit(&huart2, (uint8_t *)text, (uint16_t)strlen(text), 100);
}

static void boot_mark(const char *mark)
{
  uart_print(mark);
  uart_print("\r\n");
  delay_ms(200);
}

static void uart_print_uint(uint32_t value)
{
  char buffer[11];
  int index = 10;

  buffer[index] = '\0';

  if (value == 0) {
    uart_print("0");
    return;
  }

  while (value > 0 && index > 0) {
    index--;
    buffer[index] = (char)('0' + (value % 10));
    value /= 10;
  }

  uart_print(&buffer[index]);
}

static void uart_print_fixed_1(float value)
{
  int32_t scaled = (int32_t)(value * 10.0f);

  if (scaled < 0) {
    uart_print("-");
    scaled = -scaled;
  }

  uart_print_uint((uint32_t)(scaled / 10));
  uart_print(".");
  uart_print_uint((uint32_t)(scaled % 10));
}

static void servo_set_pulse_us(uint16_t pulse_us)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us);
}

static void servo_press_power_button(void)
{
  servo_set_pulse_us(SERVO_REST_US);
  delay_ms(SERVO_SETTLE_TIME_MS);
  servo_set_pulse_us(SERVO_PRESS_US);
  delay_ms(SERVO_PRESS_TIME_MS);
  servo_set_pulse_us(SERVO_REST_US);
  delay_ms(SERVO_SETTLE_TIME_MS);
}

static void dehumidifier_set(bool on)
{
  if (dehumidifier_on == on) {
    return;
  }

  servo_press_power_button();
  dehumidifier_on = on;

  if (on) {
    dehumidifier_turned_on_at = HAL_GetTick();
    uart_print("[SERVO] Power button pressed. State: ON\r\n");
  } else {
    dehumidifier_turned_off_at = HAL_GetTick();
    uart_print("[SERVO] Power button pressed. State: OFF\r\n");
  }
}

static bool min_off_time_passed(void)
{
  // Servo operation test mode:
  // Ignore the 5-minute minimum OFF time so repeated button presses are easy to test.
  return true;

  /*
  if (dehumidifier_turned_off_at == 0) {
    return true;
  }

  return (HAL_GetTick() - dehumidifier_turned_off_at) >= MIN_OFF_TIME_MS;
  */
}

static bool max_on_time_passed(void)
{
  return dehumidifier_on && ((HAL_GetTick() - dehumidifier_turned_on_at) >= MAX_ON_TIME_MS);
}

static bool aht20_send_command(uint8_t command, uint8_t data0, uint8_t data1)
{
  uint8_t tx_data[3] = {command, data0, data1};

  return HAL_I2C_Master_Transmit(&hi2c1, AHT20_I2C_ADDRESS, tx_data, 3, 100) == HAL_OK;
}

static bool aht20_read_status(uint8_t *status)
{
  return HAL_I2C_Master_Receive(&hi2c1, AHT20_I2C_ADDRESS, status, 1, 100) == HAL_OK;
}

static bool aht20_init(void)
{
  uint8_t status = 0;

  delay_ms(40);

  if (!aht20_read_status(&status)) {
    return false;
  }

  if ((status & 0x08) == 0) {
    if (!aht20_send_command(0xBE, 0x08, 0x00)) {
      return false;
    }
    delay_ms(10);
  }

  return true;
}

static bool aht20_read(float *humidity_percent, float *temperature_c)
{
  uint8_t rx_data[6];
  uint32_t humidity_raw;
  uint32_t temperature_raw;
  uint32_t start_tick;

  if (!aht20_send_command(0xAC, 0x33, 0x00)) {
    return false;
  }

  delay_ms(80);
  start_tick = HAL_GetTick();

  do {
    if (HAL_I2C_Master_Receive(&hi2c1, AHT20_I2C_ADDRESS, rx_data, 6, 100) != HAL_OK) {
      return false;
    }

    if ((rx_data[0] & 0x80) == 0) {
      break;
    }
  } while ((HAL_GetTick() - start_tick) < 200);

  if ((rx_data[0] & 0x80) != 0) {
    return false;
  }

  humidity_raw = ((uint32_t)rx_data[1] << 12) |
                 ((uint32_t)rx_data[2] << 4) |
                 ((uint32_t)rx_data[3] >> 4);

  temperature_raw = (((uint32_t)rx_data[3] & 0x0F) << 16) |
                    ((uint32_t)rx_data[4] << 8) |
                    (uint32_t)rx_data[5];

  *humidity_percent = ((float)humidity_raw * 100.0f) / 1048576.0f;
  *temperature_c = (((float)temperature_raw * 200.0f) / 1048576.0f) - 50.0f;

  return true;
}

static void lcd_enable_pulse(void)
{
  HAL_GPIO_WritePin(LCD_E_GPIO_PORT, LCD_E_GPIO_PIN, GPIO_PIN_SET);
  delay_ms(1);
  HAL_GPIO_WritePin(LCD_E_GPIO_PORT, LCD_E_GPIO_PIN, GPIO_PIN_RESET);
  delay_ms(1);
}

static void lcd_write_4bits(uint8_t nibble)
{
  HAL_GPIO_WritePin(LCD_D4_GPIO_PORT, LCD_D4_GPIO_PIN, (nibble & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D5_GPIO_PORT, LCD_D5_GPIO_PIN, (nibble & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D6_GPIO_PORT, LCD_D6_GPIO_PIN, (nibble & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_D7_GPIO_PORT, LCD_D7_GPIO_PIN, (nibble & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  lcd_enable_pulse();
}

static void lcd_send(uint8_t value, bool is_data)
{
  HAL_GPIO_WritePin(LCD_RS_GPIO_PORT, LCD_RS_GPIO_PIN, is_data ? GPIO_PIN_SET : GPIO_PIN_RESET);
  lcd_write_4bits(value >> 4);
  lcd_write_4bits(value & 0x0F);
}

static void lcd_command(uint8_t command)
{
  lcd_send(command, false);
  if (command == 0x01 || command == 0x02) {
    delay_ms(2);
  }
}

static void lcd_data(uint8_t data)
{
  lcd_send(data, true);
}

static void lcd_print(const char *text)
{
  while (*text != '\0') {
    lcd_data((uint8_t)*text);
    text++;
  }
}

static void lcd_print_uint(uint32_t value)
{
  char buffer[11];
  int index = 10;

  buffer[index] = '\0';

  if (value == 0) {
    lcd_data('0');
    return;
  }

  while (value > 0 && index > 0) {
    index--;
    buffer[index] = (char)('0' + (value % 10));
    value /= 10;
  }

  lcd_print(&buffer[index]);
}

static void lcd_init(void)
{
  delay_ms(50);

  HAL_GPIO_WritePin(LCD_RS_GPIO_PORT, LCD_RS_GPIO_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_E_GPIO_PORT, LCD_E_GPIO_PIN, GPIO_PIN_RESET);

  lcd_write_4bits(0x03);
  delay_ms(5);
  lcd_write_4bits(0x03);
  delay_ms(5);
  lcd_write_4bits(0x03);
  delay_ms(1);
  lcd_write_4bits(0x02);
  delay_ms(1);

  lcd_command(0x28);
  lcd_command(0x0C);
  lcd_command(0x06);
  lcd_command(0x01);
  delay_ms(2);
}

static void lcd_set_cursor(uint8_t column, uint8_t row)
{
  const uint8_t row_offsets[] = {0x00, 0x40};
  lcd_command((uint8_t)(0x80 | (column + row_offsets[row])));
}

static void lcd_clear_line(uint8_t row)
{
  uint8_t i;

  lcd_set_cursor(0, row);
  for (i = 0; i < 16; i++) {
    lcd_data(' ');
  }
}

static void lcd_show_status(float humidity)
{
  uint32_t humidity_ones = (uint32_t)(humidity + 0.5f);

  lcd_clear_line(0);
  lcd_set_cursor(0, 0);
  lcd_print("Humidity: ");
  lcd_print_uint(humidity_ones);
  lcd_print("%");

  lcd_clear_line(1);
  lcd_set_cursor(0, 1);
  lcd_print("Power: ");
  lcd_print(dehumidifier_on ? "ON" : "OFF");
}

static void serial_show_display_status(float humidity)
{
  uint32_t humidity_ones = (uint32_t)(humidity + 0.5f);

  uart_print("[DISPLAY] Humidity: ");
  uart_print_uint(humidity_ones);
  uart_print("%, Power: ");
  uart_print(dehumidifier_on ? "ON" : "OFF");
  uart_print("\r\n");
}

int main(void)
{
  bool aht20_ready;
  uint32_t last_sensor_read_at = 0;

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  uart_ready = true;

  boot_mark("B0 UART");

#if SERIAL_ONLY_TEST
  while (1) {
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_GPIO_PIN);
    uart_print("[TEST] Serial alive\r\n");
    delay_ms(1000);
  }
#endif

  MX_I2C1_Init();
  boot_mark("B1 I2C");

  MX_TIM3_Init();
  boot_mark("B2 TIM3");

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  servo_set_pulse_us(SERVO_REST_US);
  boot_mark("B3 PWM");

#if SERVO_TEST_ON_BOOT
  boot_mark("B3S SERVO");
  servo_press_power_button();
#endif

  lcd_init();
  boot_mark("B4 LCD");
  lcd_clear_line(0);
  lcd_set_cursor(0, 0);
  lcd_print("Humidity Ready");
  lcd_clear_line(1);
  lcd_set_cursor(0, 1);
  lcd_print("Power: OFF");

  aht20_ready = aht20_init();
  boot_mark("B5 AHT");
  uart_print(aht20_ready ? "[AHT20] OK\r\n" : "[AHT20] NOT FOUND\r\n");

  while (1) {
    float humidity = 0.0f;
    float temperature = 0.0f;

    if ((HAL_GetTick() - last_sensor_read_at) < SENSOR_READ_INTERVAL_MS) {
      continue;
    }
    last_sensor_read_at = HAL_GetTick();
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_GPIO_PIN);

    if (!aht20_ready) {
      uart_print("[SAFE] Humidity sensor missing. Servo will not press button.\r\n");
      continue;
    }

    if (!aht20_read(&humidity, &temperature)) {
      uart_print("[SAFE] Sensor read failed. Servo will not press button.\r\n");
      continue;
    }

    uart_print("[AHT20] Humidity: ");
    uart_print_fixed_1(humidity);
    uart_print("%, Temp: ");
    uart_print_fixed_1(temperature);
    uart_print(" C\r\n");

    lcd_show_status(humidity);
    serial_show_display_status(humidity);

    if (dehumidifier_on) {
      if (humidity <= HUMIDITY_OFF_PERCENT) {
        uart_print("[AUTO] Humidity is low enough.\r\n");
        dehumidifier_set(false);
      } else if (max_on_time_passed()) {
        uart_print("[AUTO] Maximum ON time reached.\r\n");
        dehumidifier_set(false);
      }
    } else {
      if (humidity >= HUMIDITY_ON_PERCENT && min_off_time_passed()) {
        uart_print("[AUTO] Humidity is high.\r\n");
        dehumidifier_set(true);
      } else if (humidity >= HUMIDITY_ON_PERCENT) {
        uart_print("[WAIT] Minimum OFF time has not passed yet.\r\n");
      }
    }

    lcd_show_status(humidity);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  HAL_StatusTypeDef result;

  uart_print("[I2C] init start\r\n");

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  result = HAL_I2C_Init(&hi2c1);
  if (result != HAL_OK) {
    uart_print("[I2C] init failed\r\n");
    return;
  }

  uart_print("[I2C] init ok\r\n");
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  HAL_StatusTypeDef result;

  uart_print("[TIM3] init start\r\n");

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  result = HAL_TIM_PWM_Init(&htim3);
  if (result != HAL_OK) {
    uart_print("[TIM3] pwm init failed\r\n");
    return;
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_REST_US;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  result = HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  if (result != HAL_OK) {
    uart_print("[TIM3] channel init failed\r\n");
    return;
  }

  uart_print("[TIM3] init ok\r\n");
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = STATUS_LED_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2cHandle)
{
  if (i2cHandle->Instance == I2C1) {
    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
  if (uartHandle->Instance == USART2) {
    __HAL_RCC_USART2_CLK_ENABLE();
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *timHandle)
{
  if (timHandle->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
}

static void Error_Handler(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __disable_irq();

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = STATUS_LED_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_PORT, &GPIO_InitStruct);

  while (1) {
    HAL_GPIO_TogglePin(STATUS_LED_GPIO_PORT, STATUS_LED_GPIO_PIN);
    for (volatile uint32_t i = 0; i < 800000; i++) {
    }
  }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}
