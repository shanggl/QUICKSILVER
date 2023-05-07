#include "config/config.h"

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PB7PA9     \
  USART2_PA3PA2     \
  USART3_PC11PC10   \
  USART4_PA1PA0     \
  USART6_PC7PC6

// LEDS
#define LED_NUMBER 2
#define LED1PIN PIN_B8
#define LED1_INVERT
#define LED2PIN PIN_B9
// #define LED2_INVERT
#define BUZZER_PIN PIN_C3
#define BUZZER_INVERT
#define FPV_PIN PIN_C0

// GYRO
#define GYRO_SPI_PORT SPI_PORT1
#define GYRO_NSS PIN_A4
#define GYRO_INT PIN_C4
#define GYRO_ORIENTATION GYRO_FLIP_180

// RADIO
#define USART3_INVERTER_PIN PIN_C15

// OSD
#define USE_MAX7456
#define MAX7456_SPI_PORT SPI_PORT2
#define MAX7456_NSS PIN_B12

// VOLTAGE DIVIDER
#define VBAT_PIN PIN_C2
#define VBAT_DIVIDER_R1 10000
#define VBAT_DIVIDER_R2 1000

#define IBAT_PIN PIN_C1

// MOTOR PINS
#define MOTOR_PIN0 MOTOR_PIN_PB11
#define MOTOR_PIN1 MOTOR_PIN_PB10
#define MOTOR_PIN2 MOTOR_PIN_PB0
#define MOTOR_PIN3 MOTOR_PIN_PB1