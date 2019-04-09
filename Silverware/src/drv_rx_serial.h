#include "config.h"

void usart_rx_init(void);
void Ibus_USART_ISR(void);
void DSM_USART_ISR(void);
void 	SBUS_USART_ISR(void);

//  CONDITIONAL SELECTION OF DEFINES BASED ON USER SELECTED UART FOR SERIAL RX, TARGET MCU, AND TARGET DEFINED USART ALTERNATE MAPPING PINS

#if defined (UART_1) && defined (F0)
#ifdef USART1_PA3PA2
	#define SERIAL_RX_USART USART1
	#define SERIAL_RX_SPEKBIND_BINDTOOL_PIN GPIO_Pin_2
	#define SERIAL_RX_SPEKBIND_RX_PIN GPIO_Pin_3
	#define SERIAL_RX_PIN GPIO_Pin_3
	#define SERIAL_RX_PORT GPIOA
	#define SERIAL_RX_SOURCE GPIO_PinSource3
	#define SERIAL_RX_CHANNEL GPIO_AF_1
	#define SERIAL_USART_IRQ USART1_IRQn
#endif
#ifdef USART1_SDA
	#define SERIAL_RX_USART USART1
	#define SERIAL_RX_SPEKBIND_BINDTOOL_PIN GPIO_Pin_3
	#define SERIAL_RX_PIN GPIO_Pin_14
	#define SERIAL_RX_PORT GPIOA
	#define SERIAL_RX_SOURCE GPIO_PinSource14
	#define SERIAL_RX_CHANNEL GPIO_AF_1
	#define SERIAL_USART_IRQ USART1_IRQn

#endif
#endif


#if defined (UART_1) && defined (F405)
#ifdef USART1_PA10PA9
	#define SERIAL_RX_USART USART1
	#define SERIAL_RX_SPEKBIND_BINDTOOL_PIN GPIO_Pin_9
	#define SERIAL_RX_SPEKBIND_RX_PIN GPIO_Pin_10
	#define SERIAL_RX_PIN GPIO_Pin_10
	#define SERIAL_RX_PORT GPIOA
	#define SERIAL_RX_SOURCE GPIO_PinSource10
	#define SERIAL_RX_CHANNEL GPIO_AF_USART1
	#define SERIAL_USART_IRQ USART1_IRQn
#endif
#endif

#if defined (UART_3) && defined (F405)
#ifdef USART3_PB11PB10
	#define SERIAL_RX_USART USART3
	#define SERIAL_RX_SPEKBIND_BINDTOOL_PIN GPIO_Pin_10
	#define SERIAL_RX_SPEKBIND_RX_PIN GPIO_Pin_11
	#define SERIAL_RX_PIN GPIO_Pin_11
	#define SERIAL_RX_PORT GPIOB
	#define SERIAL_RX_SOURCE GPIO_PinSource11
	#define SERIAL_RX_CHANNEL GPIO_AF_USART3
	#define SERIAL_USART_IRQ USART3_IRQn
#endif
#endif



