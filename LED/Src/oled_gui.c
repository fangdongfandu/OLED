#include "oled_gui.h"
#include "oled.h"
#include "main.h"
#include <math.h>

void oled_draw_line(uint8_t xstart,uint8_t ystart,uint8_t xend,uint8_t yend,uint8_t t)
{
    uint32_t  i;
    int xerr = 0,yerr = 0,delta_x,delta_y,distance;
    int incx,incy;
    uint32_t row,col;
    delta_x = xend - xstart;
    delta_y = yend - ystart;
    col = xstart;
    row = ystart;
    if(delta_x > 0)
        {
            incx = 1;
        }
    else
        {
            if(delta_x == 0)
                {
                    incx = 0;
                }
            else
                {
                    incx = -1;
                    delta_x = -delta_x;
                }
        }
    if(delta_y > 0)
        {
            incy = 1;
        }
    else
        {
            if(delta_y == 0)
                {
                    incy = 0;
                }
            else
                {
                    incy = -1;
                    delta_y = -delta_y;
                }
        }
    if(delta_x > delta_y)
        {
            distance = delta_x;
        }
    else
        {
            distance = delta_y;
        }
    for(i = 0; i <= distance + 1; i++)
        {
            oled_drawpoint(col, row, t);
            xerr += delta_x;
            yerr += delta_y;
            if(xerr > distance)
                {
                    xerr -= distance;
                    col += incx;
                }
            if(yerr > distance)
                {
                    yerr -= distance;
                    row += incy;
                }
        }
}

//draw circle with the middle point way
void oled_draw_circle(uint8_t x,uint8_t y,uint8_t r,uint8_t t)
{
    uint8_t xend,yend;
    int d;
    
    xend = 0;
    yend = r;
    d = 3 - 2 * r;
    
    oled_drawpoint(x + xend,y + yend,1);
    oled_drawpoint(x + xend,y - yend,1);
    oled_drawpoint(x - xend,y + yend,1);
    oled_drawpoint(x - xend,y - yend,1);
    oled_drawpoint(x + yend,y + xend,1);
    oled_drawpoint(x + yend,y - xend,1);
    oled_drawpoint(x - yend,y + xend,1);
    oled_drawpoint(x - yend,y - xend,1);
    
    while(xend < yend)
    {
        if(d < 0)
        {
            d = d + 4 * xend + 6;
        }
        else
        {
            d = d + 4 * (xend - yend) + 10;
            yend--;
        }
        xend++;
        oled_drawpoint(x + xend,y + yend,1);
        oled_drawpoint(x + xend,y - yend,1);
        oled_drawpoint(x - xend,y + yend,1);
        oled_drawpoint(x - xend,y - yend,1);
        oled_drawpoint(x + yend,y + xend,1);
        oled_drawpoint(x + yend,y - xend,1);
        oled_drawpoint(x - yend,y + xend,1);
        oled_drawpoint(x - yend,y - xend,1);
    }
}

void oled_sectorial_circle(uint8_t x,uint8_t y,double angle_start,double angle_end,uint8_t r,uint8_t t)
{
    uint8_t xstart,ystart;
    int8_t xend,yend;
    double i;
    xstart = x;
    ystart = y;
    for(i = angle_start * 2; i < angle_end * 2; i++)
        {
            xend = xstart - (int8_t)(r * cos(i/360 *PI));
            if(xend < 0||xend >127)
                {
                    if(xend < 0)
                        {
                            xend = 0;
                        }
                    else
                        {
                            xend = 127;
                        }
                }             
            yend = ystart - (int8_t)(r * sin(i/360 *PI));
            if(yend < 0||yend > 63)
                {
                    if(yend < 0)
                        {
                            yend = 0;
                        }
                    else
                        {
                            yend = 63;
                        }
                }
            oled_draw_line(xstart,ystart,xend,yend,t);
        }   
}


