#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm();
void process_image_start(void);
uint16_t get_black_width_pixel(uint8_t image[IMAGE_BUFFER_SIZE]);

#endif /* PROCESS_IMAGE_H */
