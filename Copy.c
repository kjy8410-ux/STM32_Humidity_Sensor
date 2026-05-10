#include "stm32f4xx_hal.h"  //STM32-F411RE 보드 사용
#include <stdbool.h>        
#include <stdint.h>
#include <string.h>

#define AHT20_I2C_ADDRESS   (0x38 << 1)         //습도센서 하드웨어 주소를 펌웨어 주소로 재가공
#define SENSOR_COUNT        2U                  //센서 개수에 대해 타입 Unsigned로 일치시킴, 경고 제거용

#define LCD_RS_GPIO_PORT            GPIOA
#define LCD_RS_GPIO_PIN             GPIO_PIN_10
#define LCD_E_GPIO_PORT             GPIOB
#define LCD_E_GPIO_PIN              GPIO_PIN_3
#define LCD_D4_GPIO_PORT            GPIOB
#define LCD_D4_GPIO_PIN             GPIO_PIN_5
#define LCD_D5_GPIO_PORT            GPIOB
#define LCD_D5_GPIO_PIN             GPIO_PIN_14
#define LCD_D6_GPIO_PORT            GPIOB
#define LCD_D6_GPIO_PIN             GPIO_PIN_10
#define LCD_D7_GPIO_PORT            GPIOA
#define LCD_D7_GPIO_PIN             GPIO_PIN_8

#define STATUS_LED_GPIO_PORT        GPIOA
#define STATUS_LED_GPIO_PIN         GPIO_PIN_5

#define SERVO_REST_US               1000U
#define SERVO_PRESS_US              1800U
#define SERVO_PRESS_TIME_MS         700UL
#define SERVO_SETTLE_TIME_MS        700UL
#define SERVO_TEST_ON_BOOT          1

#define HUMIDITY_ON_PERCENT         65.0f
#define HUMIDITY_OFF_PERCENT        55.0f

#define SERIAL_INLY_TEST            0

I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

typedef struct{
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
    const char *label;    
}   SensorI2cBusConfig;

static const SensorI2cBusConfig sensor_i2c_buses[SENSOR_COUNT]={
    {GPIOB,GPIO_PIN_8,GPIOB,GPIO_PIN_9,"SCL PB8,SDA PB9"},
    {GPIOB,GPIO_PIN_6,GPIOB,GPIO_PIN_7,"SCL PB6,SDA PB7"}
};

static bool dehumidifier_on = false;
static bool uart_ready = false;
static uint32_t dehumidifier_turned_on_at=0;
static uint32_t dehumidifier_turned_off_at=0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static bool MX_I2C1O_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_Init(void);
static void MX_Error_Handler_Init(void);

/* 함수 재가공 - 코드 시인성 */

static void delay_ms(uint32_t ms){
    HAL_Delay(ms);
}

static void uart_print(const char *text){
    if(!uart_ready){return;}
    HAL_UART_Transmit(&huart2,(uint8_t *)text,(uint16_t)strlen(text),100);
}

static void boot_mark(const char *mark){
    uart_print(mark);
    uart_print("\r\n");
    delay_ms(200);
}

static void uart_print_uint(uint32_t value){
    char buffer[11];
    int index=10;
    buffer[index]='\0';
    if(value==0){
        uart_print("0");
        return;
    }
    while(value>0&&index>0){
        index--;
        buffer[index]=(char)('0'+(value%10));
        value/=10;
    }
    uart_print(&buffer[index]);
}

static void uart_print_sensor_pin_plan(void){
    uint8_t sensor_index;
    for(sensor_index=0; semsor_index < SENSOR_COUNT; sensor_index++){
        uart_print("[I2C] sensor ");
        uart_print_uint(sensor_index);
        uart_print(": ");
        uart_print(sensor_i2c_buses[sensor_index].label);
        uart_print("\r\n");
    }
}

