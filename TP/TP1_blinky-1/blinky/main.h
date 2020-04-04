#ifndef MAIN_H
#define MAIN_H

/* LEDs that can be used in this TP1
  LED1 			: GPIOD pin 5
  LED3 			: GPIOD pin 6
  LED5 			: GPIOD pin 10
  LED7 			: GPIOD pin 11
  FRONT_LED 	: GPIOD pin 14
WARNING : Not on the same port !!
  BODY_LED		: GPIOB pin 2
*/
#define LED1     	GPIOD, 5
#define LED3     	GPIOD, 6
#define LED5     	GPIOD, 10
#define LED7     	GPIOD, 11
#define FRONT_LED	GPIOD, 14
#define BODY_LED		GPIOB, 2
#define SELECTOR		GPIOC,13,14,15
#define SELECTOR1	13
#define SELECTOR2	14
#define SELECTOR3	15
#define LED7_PORT	GPIOD
#define LED7_PIN		11
#define FRONTLED_PIN		14
#define LED1_PIN		5
#define LED3_PIN		6
#define LED5_PIN		10
#define BODYLED_PIN		2


#define LED_USED	LED7
#define LED_USED_FRONT	FRONT_LED

void wait_halfsec(int half_seconds);
void circular_blink(void);

#endif /* MAIN_H_ */
