#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H


#define	NOT_RIGHT_COLOR	60
#define	YIELD_DISTINGUISH_COLOR	200
#define	NB_GOALS		3
#define	RED		0
#define	BLUE 	1
#define	GREEN	2
#define	WHITE	3
#define	COLOR_NOT_ATTRIBUTED		4
#define	OFF						5


uint16_t get_line_width(uint8_t color);
uint16_t get_line_position(void);
void process_image_start(void);
uint8_t get_color_detected(void);


#endif /* PROCESS_IMAGE_H */
