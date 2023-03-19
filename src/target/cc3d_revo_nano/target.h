#include "config/config.h"

// PORTS
#define SPI_PORTS   \
  SPI1_PA5PA6PA7    \
  SPI2_PB13PB14PB15 \
  SPI3_PB3PB4PB5

#define USART_PORTS \
  USART1_PB7PB6     \
  USART2_PA3PA2

//***********MOTORS NOT YET SUPPORTED*************

// LEDS, Buzzer, FPV
#define LED_NUMBER 2
#define LED1PIN PIN_C14
#define LED2PIN PIN_C13
#define LED1_INVERT
#define BUZZER_PIN PIN_C13
#define BUZZER_INVERT
#define FPV_PIN PIN_A13

// GYRO
#define GYRO_SPI_PORT SPI_PORT2
#define GYRO_NSS PIN_B12
#define GYRO_INT PIN_A15
#define GYRO_ORIENTATION GYRO_ROTATE_180

// RADIO
#define USART2_INVERTER_PIN PIN_C15

// VOLTAGE DIVIDER
#define VBAT_PIN PIN_A6
#define VBAT_DIVIDER_R1 10000
#define VBAT_DIVIDER_R2 1000

// MOTOR PINS		dummy pins					***********MOTORS NOT YET SUPPORTED*************
#define MOTOR_PIN0 MOTOR_PIN_PB8
#define MOTOR_PIN1 MOTOR_PIN_PB9
#define MOTOR_PIN2 MOTOR_PIN_PB3
#define MOTOR_PIN3 MOTOR_PIN_PB10
