#ifndef OLED_H
#define OLED_H

#include "stm32f1xx_hal.h"

#define oled_rst_clr() HAL_GPIO_WritePin(OLED_SPI_RST_GPIO_Port,OLED_SPI_RST_Pin,GPIO_PIN_RESET)
#define oled_rst_set() HAL_GPIO_WritePin(OLED_SPI_RST_GPIO_Port,OLED_SPI_RST_Pin,GPIO_PIN_SET)

#define oled_rs_clr() HAL_GPIO_WritePin(SPI_RS_SET_GPIO_Port,SPI_RS_SET_Pin,GPIO_PIN_RESET)
#define oled_rs_set() HAL_GPIO_WritePin(SPI_RS_SET_GPIO_Port,SPI_RS_SET_Pin,GPIO_PIN_SET)

#define oled_cmd 0
#define oled_data 1

void oled_refresh_gram(void);
void oled_wr_byte(uint8_t dat,uint8_t cmd);
void oled_display_on(void);
void oled_display_off(void);
void oled_clear(void);
void oled_drawpoint(uint8_t x,uint8_t y,uint8_t t);
void oled_fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void oled_showchar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);
uint32_t oled_pow(uint8_t m,uint8_t n);
void oled_shownum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void oled_showstring(uint8_t x,uint8_t y,const uint8_t *p);
void oled_init(void);
#endif
