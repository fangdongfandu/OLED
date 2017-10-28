#ifndef OLED_GUI_H
#define OLED_GUI_H
#include "stdint.h"

#define PI 3.1425926

void oled_draw_line(uint8_t xstart,uint8_t ystart,uint8_t xend,uint8_t yend,uint8_t t);
void oled_draw_circle(uint8_t x,uint8_t y,uint8_t r,uint8_t t);
void oled_sectorial_circle(uint8_t x,uint8_t y,double angle_start,double angle_end,uint8_t r,uint8_t t);


#endif
