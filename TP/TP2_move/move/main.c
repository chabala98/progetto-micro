#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <timer.h>
#include <motor.h>
#include <selector.h>

#define PI                  3.1415926536f
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

// Init function required by __libc_init_array
void _init(void) {}

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}


int main(void)
{
    SystemClock_Config();
    // Enable GPIOD peripheral clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    gpio_config_output_af_pushpull(FRONT_LED,2);
    timer4_start(0);
    init_selector();
    //gpio_clear(FRONT_LED);

    // Enable GPIOD and GPIOE peripheral clock


    while (1) {
    		delay(SystemCoreClock/32);
    		timer4_set_duty_cycle(get_selector() / (float) MAX_VALUE_SELECTOR);
    	}
}

