#include <stm32f407xx.h>
#include <gpio.h>
#include <main.h>

void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin)
{
    // Output type open-drain : OTy = 1
    port->OTYPER |= (1 << pin);

    // Output data low : ODRy = 0
    port->ODR &= ~(1 << pin);

    // Floating, no pull-up/down : PUPDRy = 00
    port->PUPDR &= ~(3 << (pin * 2));

    // Output speed highest : OSPEEDRy = 11
    port->OSPEEDR |= (3 << (pin * 2));

    // Output mode : MODERy = 01
    port->MODER = (port->MODER & ~(3 << (pin * 2))) | (1 << (pin * 2));
}

void gpio_config_output_pull_up(GPIO_TypeDef *port, unsigned int pin)
{
    // Output type open-drain : OTy = 1
    port->OTYPER &= ~(1 << pin);

    // Output data low : ODRy = 0
    port->ODR &= ~(1 << pin);

    // Floating, no pull-up/down : PUPDRy = 00
    port->PUPDR &= ~(3 << (pin * 2));

    // Output speed highest : OSPEEDRy = 11
    port->OSPEEDR |= (3 << (pin * 2));

    // Output mode : MODERy = 01
    port->MODER = (port->MODER & ~(3 << (pin * 2))) | (1 << (pin * 2));
}

void gpio_config_selector(GPIO_TypeDef *port, unsigned int pin1, unsigned int pin2, unsigned pin3)
{

	    // input mode : MODERy = 00
	    port->MODER = (port->MODER & ~(3 << (pin1 * 2)));
	    port->MODER = (port->MODER & ~(3 << (pin2 * 2)));
	    port->MODER = (port->MODER & ~(3 << (pin3 * 2)));
	    port->PUPDR = (port->PUPDR & ~(1 << (pin1*2)));
	    port->PUPDR = (port->PUPDR & ~(1 << (pin2*2)));
	    port->PUPDR = (port->PUPDR & ~(1 << (pin3*2)));

}

void gpio_set(GPIO_TypeDef *port, unsigned int pin)
{
    port->BSRR = (1 << pin);
}

void gpio_clear(GPIO_TypeDef *port, unsigned int pin)
{
    port->BSRR = (1 << (pin + 16));
}

void gpio_toggle(GPIO_TypeDef *port, unsigned int pin)
{
    if (port->ODR & (1<<pin)) {
        gpio_clear(port, pin);
    } else {
        gpio_set(port, pin);
    }
}
int gpio_read_pin_idr(GPIO_TypeDef *port, unsigned int pin)
{

	return ( (port->IDR) & (1<<pin) );
}
