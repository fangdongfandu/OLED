#include "main.h"
#include "oled.h"
#include "oledfont.h"

SPI_HandleTypeDef hspi1;
//view way of OLED
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 			   
uint8_t oled_gram[128][8];

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 5;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    log_printf("SPI Init fail!\r\n");
  }
  
}

void oled_refresh_gram(void)
{
    uint8_t i,n;
    for(i = 0; i < 8; i++)
    {
        oled_wr_byte(0xb0 + i,oled_cmd);
        oled_wr_byte(0x00,oled_cmd);
        oled_wr_byte(0x10,oled_cmd);
        for(n = 0; n < 128; n++)
        {
            oled_wr_byte(oled_gram[n][i],oled_data);
        }
    }
}

void oled_wr_byte(uint8_t dat,uint8_t cmd)
{
    uint8_t temp;
    uint8_t data;
    data = dat;
    if(cmd)
        oled_rs_set();
    else
        oled_rs_clr();
    if(HAL_SPI_Transmit(&hspi1,&data,sizeof(uint8_t),5000) != HAL_OK)
    {
        log_printf("spi transmit fail!\r\n");
    }
    oled_rs_set();
}

void oled_display_on(void)
{
    oled_wr_byte(0x8D,oled_cmd);
    oled_wr_byte(0x14,oled_cmd);
    oled_wr_byte(0xAF,oled_cmd);
}

void oled_display_off(void)
{
    oled_wr_byte(0x8D,oled_cmd);
    oled_wr_byte(0x10,oled_cmd);
    oled_wr_byte(0xAE,oled_cmd);
}

void oled_clear(void)
{
    uint8_t i,n;
    for(i = 0; i < 8; i++ )
        for(n = 0; n < 128; n++ )
            oled_gram[n][i] = 0x00;
    oled_refresh_gram();
}

void oled_drawpoint(uint8_t x,uint8_t y,uint8_t t)
{
    uint8_t pos,bx,temp = 0;
    if(x > 127||y > 63)
        return ;
    pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    if(t)
        oled_gram[x][pos] |= temp;
    else
        oled_gram[x][pos] &=~temp;
}

void oled_fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot)
{
    uint8_t x,y;
    for(x = x1 ;x <= x2 ;x++ )
    {
        for(y = y1 ;y <= y2 ;y++ )
            oled_drawpoint(x,y,dot);
    }
    oled_refresh_gram();
}

void oled_showchar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode)
{
    uint8_t temp,t,t1;
    uint8_t y0 = y;
    chr = chr - ' ';
    for(t = 0 ;t < size ;t++ )
        {
            if(size == 12)
                temp = oled_asc2_1206[chr][t];
            else
                temp = oled_asc2_1608[chr][t];
            for(t1 = 0 ;t1 < 8 ;t1++ )
                {
                    if(temp & 0x80)
                        oled_drawpoint(x ,y ,t );
                    else
                        oled_drawpoint(x ,y ,!mode );
                        temp <<= 1;
                        y++;
                        if((y - y0) == size)
                            {
                                y = y0;
                                x++;
                                break;
                            }
                }
        }
}

uint32_t oled_pow(uint8_t m,uint8_t n)
{
    uint32_t result = 1;
    while(n--)
        result *= m;
    return result;
}

void oled_shownum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size)
{
    uint8_t t,temp;
    uint8_t enshow = 0;
    for(t = 0 ;t < len ;t++)
        {
            temp = (num / oled_pow(10, len -t -1)) % 10;
            if(enshow == 0 && t < (len - 1))
                {
                    oled_showchar(x + (size / 2) * t ,y ,' ' ,size ,1 );
                    continue;
                }
            else
                enshow = 1;
        }
    oled_showchar(x + (size / 2) * t ,y , temp + '0' ,size ,1 );
}

void oled_showstring(uint8_t x,uint8_t y,const uint8_t *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
    while(*p != '\0')
        {
            if(x > MAX_CHAR_POSX)
                {
                    x = 0;
                    y += 16;
                }
            if(y > MAX_CHAR_POSY)
                {
                    y = x = 0;
                    oled_clear();
                }
            oled_showchar(x ,y ,*p ,16 ,1 );
            x += 8;
            p++;
        }
}


void oled_init(void)
{
    MX_SPI1_Init();
    //__HAL_SPI_ENABLE(&hspi1);
    oled_rst_clr();
    HAL_Delay(100);
    oled_rst_set();

    oled_wr_byte(0xAE, oled_cmd);
    oled_wr_byte(0xD5, oled_cmd);
    oled_wr_byte(80, oled_cmd);
    oled_wr_byte(0xA8, oled_cmd);
    oled_wr_byte(0x3F, oled_cmd);
    oled_wr_byte(0xD3, oled_cmd);
    oled_wr_byte(0x00, oled_cmd);

    oled_wr_byte(0x40, oled_cmd);

    oled_wr_byte(0x8D, oled_cmd);
    oled_wr_byte(0x14, oled_cmd);
    oled_wr_byte(0x20, oled_cmd);
    oled_wr_byte(0x02, oled_cmd);
    oled_wr_byte(0xA1, oled_cmd);
    oled_wr_byte(0xC0, oled_cmd);
    oled_wr_byte(0xDA, oled_cmd);
    oled_wr_byte(0x12, oled_cmd);

    oled_wr_byte(0x81, oled_cmd);
    oled_wr_byte(0xEF, oled_cmd);
//    oled_wr_byte(0x0F, oled_cmd);
    oled_wr_byte(0xD9, oled_cmd);
    oled_wr_byte(0xF1, oled_cmd);
    oled_wr_byte(0xDB, oled_cmd);
    oled_wr_byte(0x30, oled_cmd);

    oled_wr_byte(0xA4, oled_cmd);
    oled_wr_byte(0xA6, oled_cmd);
    oled_wr_byte(0xAF, oled_cmd);
    oled_clear();
}

