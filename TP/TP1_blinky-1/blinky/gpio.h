#ifndef GPIO_H
#define GPIO_H

#include <stm32f407xx.h>

void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin);
void gpio_set(GPIO_TypeDef *port, unsigned int pin);
void gpio_clear(GPIO_TypeDef *port, unsigned int pin);
void gpio_toggle(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_output_pull_up(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_selector(GPIO_TypeDef *port, unsigned int pin1, unsigned int pin2, unsigned pin3);
int gpio_read_pin_idr(GPIO_TypeDef *port, unsigned int pin);

#endif /* GPIO_H */
