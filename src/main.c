#include "../system/include/cmsis/stm32f4xx.h"

#define _HAL_RCC_GPIOD_CLK_ENABLE()                                 (RCC->AHB1ENR |= (1 << 3))

#define EXTIx_IRQn                               EXTI0_IRQn
#define EXTIx_IRQHandler                         EXTI0_IRQHandler

#define GPIO_BUTTON_PIN                          0
#define GPIO_BUTTON_PORT                         GPIOA
#define GPIO_PIN_INPUT_MODE                                          ( (uint32_t)0x00 )
#define GPIO_PIN_OUTPUT_MODE                                         ( (uint32_t)0x01 )
#define GPIO_PIN_ALT_FUN_MODE                                        ( (uint32_t)0x02 )
#define GPIO_PIN_ANALOG_MODE                                         ( (uint32_t)0x03 )


/* GPIO Output type selection value */
#define GPIO_PIN_OUTPUT_TYPE_PUSHPULL                                ( (uint32_t)0x00 )                            
#define GPIO_PIN_OUTPUT_TYPE_OPEN_DRAIN                              ( (uint32_t)0x01 )


/* GPIO Speed type selection value */
#define GPIO_PIN_SPEED_LOW                                           ( (uint32_t)0x00 )
#define GPIO_PIN_SPEED_MEDIUM                                        ( (uint32_t)0x01 )
#define GPIO_PIN_SPEED_HIGH                                          ( (uint32_t)0x02 )
#define GPIO_PIN_SPEED_VERY_HIGH                                     ( (uint32_t)0x03 )


/* GPIO  pull-up/pull-down values */
#define GPIO_PIN_NO_PUSH_PULL                                        ( (uint32_t)0x00 )
#define GPIO_PIN_PULL_UP                                             ( (uint32_t)0x01 )
#define GPIO_PIN_PULL_DOWN                                           ( (uint32_t)0x02 )


/*GPIO Port Addresss */
#define GPIO_PORT_A             GPIOA
#define GPIO_PORT_B             GPIOB   
#define GPIO_PORT_C             GPIOC   
#define GPIO_PORT_D             GPIOD  
#define GPIO_PORT_E             GPIOE  
#define GPIO_PORT_F             GPIOF
#define GPIO_PORT_G             GPIOG
#define GPIO_PORT_H             GPIOH  
#define GPIO_PORT_I             GPIOI

#define GPIOD_PIN_12                             12
#define GPIOD_PIN_13                             13
#define GPIOD_PIN_14                             14
#define GPIOD_PIN_15                             15

#define LED_GREEN                                GPIOD_PIN_12
#define LED_ORANGE                               GPIOD_PIN_13
#define LED_RED                                  GPIOD_PIN_14
#define LED_BLUE                                 GPIOD_PIN_15

typedef struct {
	uint32_t pin;               /* Specifies the GPIO pin to be configured */
	uint32_t mode;              /* Specifies the operatin mode for selected pin */
	uint32_t output_type;       /* Specifies the output type for selected pin*/
	uint32_t pull;              /* Specifies the pull-down / pull-up activation for selected pin */
	uint32_t speed;             /* Specifies the speed of the selected pin */
	uint32_t alternate;         /* Specifies alternate function value */

} gpio_pin_config_typedef;

/**
  * @breief Configure the mode of the pin : input, output, analog
  * @param  GPIOx : GPIO port base address
	* @param  pin_no : GPIO pin number
	* @param  mode : mode to be cinfigured
	* @retval none
**/	
static void hal_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t mode)
{
	GPIOx->MODER |= (mode << (2 * pin_no)); 
}


/**
  * @breief Configure the output type of the pin
  * @param  GPIOx : GPIO port base address
	* @param  pin_no : GPIO pin number
	* @param  output_type: output type to be configured
	* @retval none
**/
static void hal_gpio_configure_pin_output_type(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t output_type)
{
	GPIOx->OTYPER |= ( output_type << pin_no);

}
/**
  * @breief Configure the Speed of the pin
  * @param  GPIOx : GPIO port base address
	* @param  pin_no : GPIO pin number
	* @param  speed : value of the spedd
	* @retval none
**/
static void hal_gpio_configure_pin_speed(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t speed)
{
	GPIOx->OSPEEDR |= (speed << (2 * pin_no));
}



/**
  * @breief Activate the internal pull-up/pull-down resistor
  * @param  GPIOx : GPIO port base address
	* @param  pin_no : GPIO pin number
	* @param  pupd : specifies which resistor to activate
	* @retval none
**/
static void hal_gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pupd)
{
	GPIOx->PUPDR |= (pupd << (2 * pin_no));	
}

void hal_gpio_init(GPIO_TypeDef *GPIOx, gpio_pin_config_typedef *gpio_pin_config) {
	//Configure Pin MOde
	hal_gpio_configure_pin_mode(GPIOx, gpio_pin_config->pin,gpio_pin_config->mode);
	//Configure the output type of the pin
	hal_gpio_configure_pin_output_type(GPIOx, gpio_pin_config->pin, gpio_pin_config->output_type);
	//Configure the Speed of the pin
	hal_gpio_configure_pin_speed(GPIOx, gpio_pin_config->pin, gpio_pin_config->speed);
	//Activate the internal pull-up/pull-down resistor
	hal_gpio_configure_pin_pupd(GPIOx, gpio_pin_config->pin, gpio_pin_config->pull);
}

void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint8_t value) {
	if(value)
		GPIOx->ODR |= (1 << pin_no);
	else
		GPIOx->ODR &= ~(1 << pin_no);
}

void led_init(void) {

	/* Enable clock for  GPIOD port */
	_HAL_RCC_GPIOD_CLK_ENABLE();

	gpio_pin_config_typedef led_pin_config;

	led_pin_config.pin = LED_ORANGE;
	led_pin_config.mode = GPIO_PIN_OUTPUT_MODE;
	led_pin_config.output_type = GPIO_PIN_OUTPUT_TYPE_PUSHPULL;
	led_pin_config.speed = GPIO_PIN_SPEED_LOW;
	led_pin_config.pull = GPIO_PIN_NO_PUSH_PULL;

	hal_gpio_init(GPIO_PORT_D, &led_pin_config);

	led_pin_config.pin = LED_GREEN;
	hal_gpio_init(GPIO_PORT_D, &led_pin_config);

	led_pin_config.pin = LED_RED;
	hal_gpio_init(GPIO_PORT_D, &led_pin_config);

	led_pin_config.pin = LED_BLUE;
	hal_gpio_init(GPIO_PORT_D, &led_pin_config);

}

void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin) {
	hal_gpio_write_to_pin(GPIOx, pin, 1);
}


void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin) {
	hal_gpio_write_to_pin(GPIOx, pin, 0);
}


void msDelay(uint32_t msTime)
{
	for(uint32_t i=0;i<msTime*4000;i++);
}

int main(void) {
	int temp;
    led_init();


	while(1) {	//for(;;) {}
        led_turn_on(GPIOD, LED_BLUE);
		msDelay(1000);
        led_turn_on(GPIOD, LED_GREEN);
		led_turn_off(GPIOD, LED_BLUE);
		msDelay(1000);
        led_turn_on(GPIOD, LED_ORANGE);
		led_turn_off(GPIOD, LED_GREEN);
		msDelay(1000);
        led_turn_on(GPIOD, LED_RED);
		led_turn_off(GPIOD, LED_ORANGE);
		msDelay(1000);
        led_turn_off(GPIOD, LED_RED);
	}
}


