#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>

// Init function required by __libc_init_array
void _init(void) {}

void wait_halfsec(int half_seconds)
{
	for(int i=0;i<14000000/4*half_seconds ; i++){
		__NOP;
	}
}

void circular_blink(void){
	LED7_PORT->BSRR &= ~(1 << (LED1_PIN));
	LED7_PORT->BSRR |= (1 << (LED1_PIN+16));
    	wait_halfsec(1);
    	LED7_PORT->BSRR &= ~(1 << (LED1_PIN+16));
    	LED7_PORT->BSRR |= (1 << (LED1_PIN));
    	LED7_PORT->BSRR &= ~(1 << (LED3_PIN));
    	LED7_PORT->BSRR |= (1 << (LED3_PIN+16));
    	wait_halfsec(1);
    	LED7_PORT->BSRR &= ~(1 << (LED3_PIN+16));
    	LED7_PORT->BSRR |= (1 << (LED3_PIN));
    	LED7_PORT->BSRR &= ~(1 << (LED5_PIN));
    	LED7_PORT->BSRR |= (1 << (LED5_PIN+16));
    	wait_halfsec(1);
    	LED7_PORT->BSRR &= ~(1 << (LED5_PIN+16));
    	LED7_PORT->BSRR |= (1 << (LED5_PIN));
    	LED7_PORT->BSRR &= ~(1 << (LED7_PIN));
    	LED7_PORT->BSRR |= (1 << (LED7_PIN+16));
    	wait_halfsec(1);
    	LED7_PORT->BSRR &= ~(1 << (LED7_PIN+16));
    	LED7_PORT->BSRR |= (1 << (LED7_PIN));

}


int main(void)
{
    SystemClock_Config();


    // Enable GPIOD peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIODEN;
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOCEN;



    // LED used init
    //gpio_config_output_opendrain(LED_USED);
    //gpio_clear(LED_USED);
    gpio_config_output_pull_up(FRONT_LED);
    gpio_clear(LED_USED_FRONT);
    gpio_config_output_pull_up(LED1);
    gpio_clear(LED1);
    gpio_config_output_pull_up(LED3);
    gpio_clear(LED3);
    gpio_config_output_pull_up(LED5);
    gpio_clear(LED5);
    gpio_config_output_pull_up(LED7);
    gpio_clear(LED7);
    gpio_config_output_pull_up(BODY_LED);
    gpio_clear(BODY_LED);
    gpio_config_selector(SELECTOR);
    //gpio_toggle(FRONT_LED);


    //timer7_start();
    while (1) {
    		//circular_blink();
    		wait_halfsec(1);
    		gpio_toggle(FRONT_LED);

        /*GPIOB->BSRR &= ~(1 << (BODYLED_PIN+16));
        	LED7_PORT->BSRR &= ~(1 << (LED1_PIN+16));
        	LED7_PORT->BSRR &= ~(1 << (LED3_PIN+16));
        	LED7_PORT->BSRR &= ~(1 << (LED5_PIN+16));
        	LED7_PORT->BSRR &= ~(1 << (LED7_PIN+16));
        	//LED7_PORT->BSRR |= (1 << (FRONTLED_PIN));
        	GPIOB->BSRR |= (1 << (BODYLED_PIN));
        	LED7_PORT->BSRR |= (1 << (LED1_PIN));
        	LED7_PORT->BSRR |= (1 << (LED3_PIN));
        	LED7_PORT->BSRR |= (1 << (LED5_PIN));
        	LED7_PORT->BSRR |= (1 << (LED7_PIN));
        	wait_halfsec(1);
        	GPIOB->BSRR &= ~(1 << (BODYLED_PIN));
        	LED7_PORT->BSRR &= ~(1 << (LED1_PIN));
        	LED7_PORT->BSRR &= ~(1 << (LED3_PIN));
        	LED7_PORT->BSRR &= ~(1 << (LED5_PIN));
        	LED7_PORT->BSRR &= ~(1 << (LED7_PIN));
        	LED7_PORT->BSRR |= (1 << (FRONTLED_PIN));
        	GPIOB->BSRR |= (1 << (BODYLED_PIN+16));
        	LED7_PORT->BSRR |= (1 << (LED1_PIN+16));
        	LED7_PORT->BSRR |= (1 << (LED3_PIN+16));
        	LED7_PORT->BSRR |= (1 << (LED5_PIN+16));
        	LED7_PORT->BSRR |= (1 << (LED7_PIN+16));*/

        }
}
