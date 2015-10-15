/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_lcd.c
 @brief  : this file include the driver functions for the lcd module.
 @author : wangxianwen
 @history:
           2015-5-19    wangxianwen    Created file
           ...
******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "drv_lcd.h"

#include "stm32f4xx.h"
#include "bsp_lcd_HX8257.h"




/** @defgroup Private_Defines
  * @{
  */

#define POLY_Y(Z)          ((int32_t)((Points + Z)->X))
#define POLY_X(Z)          ((int32_t)((Points + Z)->Y))

/**
  * @}
  */  


/** @defgroup Private_FunctionPrototypes
  * @{
  */ 

static DRV_LCD_ERR_CODE lcd_ioctl_set_layer_transparency
(
    /* pinter to the layer transparency set structure. */
    drv_lcd_layer_transparency_set_st_ptr transparency_ptr
);

static DRV_LCD_ERR_CODE lcd_ioctl_draw_pixel
(
    /* Pointer to pixel draw structure. */
    drv_lcd_pixel_draw_st_ptr pixel_ptr
);

static DRV_LCD_ERR_CODE lcd_ioctl_draw_hv_line
(
    /* Pointer to hv line draw structure. */
    drv_lcd_hv_line_draw_st_ptr line_ptr
);

static DRV_LCD_ERR_CODE lcd_ioctl_draw_universal_line
(
    /* Pointer to screen clear structure. */
    drv_lcd_universal_line_draw_st_ptr line_ptr
);

static DRV_LCD_ERR_CODE lcd_ioctl_draw_absolute_poly_line
(
    /* Pointer to poly line draw structure. */
    drv_lcd_poly_line_draw_st_ptr line_ptr
);

static DRV_LCD_ERR_CODE lcd_ioctl_draw_relative_poly_line
(
    /* Pointer to poly line draw structure. */
    drv_lcd_poly_line_draw_st_ptr line_ptr
);

static DRV_LCD_ERR_CODE lcd_ioctl_draw_circle
(
    /* Pointer to circle draw structure. */
    drv_lcd_circle_draw_st_ptr circle_ptr
);


static DRV_LCD_ERR_CODE lcd_ioctl_set_colorkeying
(
    /* Pointer to colorkeying set structure. */
    drv_lcd_colorkeying_set_st_ptr colorkeying_ptr
);

static DRV_LCD_ERR_CODE lcd_ioctl_reset_colorkeying
(
    /* Pointer to colorkeying set structure. */
    drv_lcd_colorkeying_reset_st_ptr colorkeying_ptr
);






static DRV_LCD_ERR_CODE lcd_ioctl_clear_layer
(
    /* Pointer to screen clear structure. */
    drv_lcd_layer_clear_st_ptr layer_ptr
);


/**
  * @}
  */ 




/** @defgroup Private_Functions
  * @{
  */


/**
  * @brief  Configure the specific layer's transparency.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_set_layer_transparency
(
    /* pinter to the layer transparency set structure. */
    drv_lcd_layer_transparency_set_st_ptr transparency_ptr
)
{
    DRV_LCD_ERR_CODE err_code = DRV_LCD_ERR_IOCTL;
    
    /* Key for background and foreground set.  */
    uint8_t    key_background = 0;
    uint8_t    key_foreground = 0;
    
    
    if (transparency_ptr == (void *)0)
    {
        err_code = DRV_LCD_ERR_IOCTL;
    }
    else
    {
        /* Initialize the key of background and foreground based on parameter. */
        switch (transparency_ptr->layer)
        {
            case LCD_BACKGROUND_LAYER:  {  key_background = 1;            
                                           key_foreground = 0;            break; }
            
            case LCD_FOREGROUND_LAYER:  {  key_background = 0; 
                                           key_foreground = 1;            break; }
            
            case LCD_BACK_FORE_LAYER:   {  key_background = 1;
                                           key_foreground = 1;            break; }
            
            default:                    {  key_background = 0;
                                           key_foreground = 0;  
                                           err_code = DRV_LCD_ERR_IOCTL;  break; }
        }
        
        if (key_background == 1)
        {
            /* Must reload the shadow registers after slpha reconfiguration. */
            LTDC_LayerAlpha(LTDC_Layer1, transparency_ptr->transparency);
            LTDC_ReloadConfig(LTDC_IMReload);   
            
            err_code = DRV_LCD_ERR_OK;
        }
        
        if (key_foreground == 1)
        {  
            /* Must reload the shadow registers after slpha reconfiguration. */  
            LTDC_LayerAlpha(LTDC_Layer2, transparency_ptr->transparency);
            LTDC_ReloadConfig(LTDC_IMReload);   
            
            err_code = DRV_LCD_ERR_OK;
        }
    }
    
    return err_code;
}


/**
  * @brief  Displays a pixel.
  * @param  See below. 
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_draw_pixel
(
    /* Pointer to pixel draw structure. */
    drv_lcd_pixel_draw_st_ptr pixel_ptr
)
{ 
    DRV_LCD_ERR_CODE       err_code = DRV_LCD_ERR_IOCTL;
    drv_lcd_hv_line_draw_st hv_line = { LCD_FOREGROUND_LAYER };

    
    if (pixel_ptr == (void*)0)
    {
        err_code = DRV_LCD_ERR_IOCTL;
    }
    else
    {
        if ( (pixel_ptr->x < 0) || (LCD_SCREEN_PIXEL_WIDTH <= pixel_ptr->x)
          || (pixel_ptr->y < 0) || (LCD_SCREEN_PIXEL_HEIGHT <= pixel_ptr->y) )
        {
            err_code = DRV_LCD_ERR_IOCTL;
        }
        else
        {
            hv_line.layer = pixel_ptr->layer;
            hv_line.color = pixel_ptr->color;
            hv_line.x = pixel_ptr->x;
            hv_line.y = pixel_ptr->y;
            
            hv_line.length = 1;
            hv_line.width = 1;
            hv_line.line_dir = LCD_LINE_DIR_HORIZONTAL;
            
            err_code = lcd_ioctl_draw_hv_line(&hv_line);   
        }
    }
    
    return err_code;
}


/**
  * @brief  Displays a horizental or vertical line to lcd.
            We can set the line's length and width based on line direction.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_draw_hv_line
(
    /* Pointer to hv line draw structure. */
    drv_lcd_hv_line_draw_st_ptr line_ptr
)
{
    DMA2D_InitTypeDef      DMA2D_InitStruct = { 0 };
    uint32_t                       Xaddress = 0;
    drv_lcd_layer_value           cur_layer = LCD_BACK_FORE_LAYER;
    
    
    /* Jump to error when coordinate X,Y without the lcd screen. */
    if ( (line_ptr == (void*)0) 
      || (LCD_SCREEN_PIXEL_WIDTH <= line_ptr->x) || (LCD_SCREEN_PIXEL_HEIGHT <= line_ptr->y) 
      || (line_ptr->length == 0) || (line_ptr->width == 0) )
    {
        goto DATA_ERR;
    }
    
    /* Jump to error when horizental line draw without the lcd screen. */
    if ( (line_ptr->line_dir == LCD_LINE_DIR_HORIZONTAL) 
      && ((LCD_SCREEN_PIXEL_WIDTH < (line_ptr->x + line_ptr->length)) || (LCD_SCREEN_PIXEL_HEIGHT < (line_ptr->y + line_ptr->width))) )
    {
        goto DATA_ERR; 
    }
    
    /* Jump to error when vertical line draw without the lcd screen. */
    if ( (line_ptr->line_dir == LCD_LINE_DIR_VERTICAL) 
      && ((LCD_SCREEN_PIXEL_WIDTH < (line_ptr->x + line_ptr->width)) || (LCD_SCREEN_PIXEL_HEIGHT < (line_ptr->y + line_ptr->length))) )
    {
        goto DATA_ERR; 
    }

       
    cur_layer = line_ptr->layer;
    
    do
    {
        /* Initialize the screen buffer start address based on active layer. */
        switch (cur_layer)
        {
            /* when active layer is back & fore layer,we draw the background layer firstly. */
            case LCD_BACK_FORE_LAYER:
            case LCD_BACKGROUND_LAYER: {   
                Xaddress = LCD_FRAME_BUFFER_BACKGROUND + 2 * (LCD_SCREEN_PIXEL_WIDTH * line_ptr->y + line_ptr->x);  
                break; 
            }
            case LCD_FOREGROUND_LAYER: {  
                Xaddress = LCD_FRAME_BUFFER_FOREGROUND + 2 * (LCD_SCREEN_PIXEL_WIDTH * line_ptr->y + line_ptr->x);  
                break; 
            }
            default: {
                goto DATA_ERR;
                break;
            }
        }
   
        /* Configure DMA2D module. */    
        DMA2D_DeInit();
        
        DMA2D_InitStruct.DMA2D_Mode = DMA2D_R2M;       
        DMA2D_InitStruct.DMA2D_CMode = DMA2D_RGB565;      
        DMA2D_InitStruct.DMA2D_OutputGreen = LCD_DISMISS_G(line_ptr->color);      
        DMA2D_InitStruct.DMA2D_OutputBlue = LCD_DISMISS_B(line_ptr->color);     
        DMA2D_InitStruct.DMA2D_OutputRed = LCD_DISMISS_R(line_ptr->color);                
        DMA2D_InitStruct.DMA2D_OutputAlpha = 0x0F;                  
        DMA2D_InitStruct.DMA2D_OutputMemoryAdd = Xaddress;                  

        if (line_ptr->line_dir == LCD_LINE_DIR_HORIZONTAL)
        {                                                      
            DMA2D_InitStruct.DMA2D_OutputOffset = LCD_SCREEN_PIXEL_WIDTH - line_ptr->length;                
            DMA2D_InitStruct.DMA2D_NumberOfLine = line_ptr->width;            
            DMA2D_InitStruct.DMA2D_PixelPerLine = line_ptr->length; 
        }
        else
        {                                                            
            DMA2D_InitStruct.DMA2D_OutputOffset = LCD_SCREEN_PIXEL_WIDTH - line_ptr->width;                
            DMA2D_InitStruct.DMA2D_NumberOfLine = line_ptr->length;            
            DMA2D_InitStruct.DMA2D_PixelPerLine = line_ptr->width;  
        }

        DMA2D_Init(&DMA2D_InitStruct);  
        
        /* Start Transfer. */ 
        DMA2D_StartTransfer();  
        
        /* Wait for CTC Flag activation. */
        while (DMA2D_GetFlagStatus(DMA2D_FLAG_TC) == RESET)
        {
        }
        
     /* when current layer is BACK & FORE layer, set to FORE layer and draw again. */
    }while((cur_layer == LCD_BACK_FORE_LAYER) && (cur_layer = LCD_FOREGROUND_LAYER));  
        
    
    return DRV_LCD_ERR_OK;
    
DATA_ERR:
    return DRV_LCD_ERR_IOCTL; 
}


/**
  * @brief  Displays an universal-line (between two points). 
            Based on Bresenham draw line algorithm.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_draw_universal_line
(
    /* Pointer to universal line draw structure. */
    drv_lcd_universal_line_draw_st_ptr line_ptr
)
{
    int16_t deltax = 0, deltay = 0,   x = 0,   y = 0,  xinc1 = 0,     xinc2 = 0, 
             yinc1 = 0,  yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
          curpixel = 0;
    
    DRV_LCD_ERR_CODE   err_code = DRV_LCD_ERR_IOCTL;
    drv_lcd_pixel_draw_st pixel = { LCD_FOREGROUND_LAYER };


    /* make error detection. */
	if (line_ptr == (void*)0)
	{
        return DRV_LCD_ERR_IOCTL;
	}

    if ( ((LCD_SCREEN_PIXEL_WIDTH - 1) < line_ptr->x1) || ((LCD_SCREEN_PIXEL_HEIGHT - 1) < line_ptr->y1) 
      || ((LCD_SCREEN_PIXEL_WIDTH - 1) < line_ptr->x2) || ((LCD_SCREEN_PIXEL_HEIGHT - 1) < line_ptr->y2) )
    {
        return DRV_LCD_ERR_IOCTL;
    }
    
    /* Start x off/y off at the first pixel. */
    x = line_ptr->x1;                                
    y = line_ptr->y1;

    if (line_ptr->x1 <= line_ptr->x2)                 
    {
        /* The x-values are increasing. */
        deltax = line_ptr->x2 - line_ptr->x1;
        
        xinc1 = 1;
        xinc2 = 1;
    }
    else                          
    {
        /* The x-values are decreasing. */
        deltax = line_ptr->x1 - line_ptr->x2;
        
        xinc1 = -1;
        xinc2 = -1;
    }

    if (line_ptr->y1 <= line_ptr->y2)                 
    {
        /* The y-values are increasing. */
        deltay = line_ptr->y2 - line_ptr->y1;
        
        yinc1 = 1;
        yinc2 = 1;
    }
    else                          
    {
        /* The y-values are decreasing. */
        deltay = line_ptr->y1 - line_ptr->y2;
        
        yinc1 = -1;
        yinc2 = -1;
    }

    if (deltay <= deltax)           /* There is at least one x-value for every y-value. */
    {
        xinc1 = 0;                  /* Don't change the x when numerator >= denominator. */
        yinc2 = 0;                  /* Don't change the y for every iteration. */
        den = deltax;
        num = deltax / 2;
        numadd = deltay;
        numpixels = deltax;         /* There are more x-values than y-values. */
    }
    else                            /* There is at least one y-value for every x-value. */
    {
        xinc2 = 0;                  /* Don't change the x for every iteration. */
        yinc1 = 0;                  /* Don't change the y when numerator >= denominator. */
        den = deltay;
        num = deltay / 2;
        numadd = deltax;
        numpixels = deltay;         /* There are more y-values than x-values. */
    }

    for (curpixel = 0; curpixel <= numpixels; curpixel++)
    {
        /* Initialize pixel draw structure. */
        pixel.x = x;
        pixel.y = y;
        pixel.layer = line_ptr->layer;
        pixel.color = line_ptr->color;
        
        /* Draw the current pixel. */
        err_code = lcd_ioctl_draw_pixel(&pixel); 
      
        num += numadd;                /* Increase the numerator by the top of the fraction. */
        if (num >= den)               /* Check if numerator >= denominator. */
        {
            num -= den;               /* Calculate the new numerator value. */
            x += xinc1;               /* Change the x as appropriate. */
            y += yinc1;               /* Change the y as appropriate. */
        }
        x += xinc2;                   /* Change the x as appropriate. */
        y += yinc2;                   /* Change the y as appropriate. */
    }
    
    return err_code;
}


/**
  * @brief  Displays an absolute poly-line (between many points).
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_draw_absolute_poly_line
(
    /* Pointer to poly line draw structure. */
    drv_lcd_poly_line_draw_st_ptr line_ptr
)
{     
    uint16_t                    point_index = 0;
    drv_lcd_universal_line_draw_st uni_line = { LCD_FOREGROUND_LAYER };
    
    
    /* Error detection. */
    if ( (line_ptr == (void*)0) || (line_ptr->point_count < 2) )
    {
        return DRV_LCD_ERR_IOCTL;
    }
    
    /* Initialize fixed part of universal line structure. */
    uni_line.layer = line_ptr->layer;
    uni_line.color = line_ptr->color;
    
    /* Draw line of (first point & last point) if line type is closed. */
    if(line_ptr->poly_line_type == LCD_POLY_LINE_CLOSED)
    {
        uni_line.x1 = line_ptr->point_ptr[0].x;
        uni_line.y1 = line_ptr->point_ptr[0].y;
        
        uni_line.x2 = line_ptr->point_ptr[line_ptr->point_count - 1].x;
        uni_line.y2 = line_ptr->point_ptr[line_ptr->point_count - 1].y;

        /* Break the draw process when get error. */
        if(lcd_ioctl_draw_universal_line(&uni_line)== DRV_LCD_ERR_IOCTL)
        {
            return DRV_LCD_ERR_IOCTL;
        }
    }
    
    /* Draw universal line based on 2 point. Caution: line number one less than point number. */
    for (point_index = 0; point_index < (line_ptr->point_count - 1); point_index ++)
    {
        uni_line.x1 = line_ptr->point_ptr[point_index].x;
        uni_line.y1 = line_ptr->point_ptr[point_index].y;
        
        uni_line.x2 = line_ptr->point_ptr[point_index + 1].x;
        uni_line.y2 = line_ptr->point_ptr[point_index + 1].y;

        /* Break the draw process when get error. */
        if(lcd_ioctl_draw_universal_line(&uni_line) == DRV_LCD_ERR_IOCTL)
        {
            break;
        }
    }
    
    return (point_index == (line_ptr->point_count - 1) ? DRV_LCD_ERR_OK : DRV_LCD_ERR_IOCTL);  
}


/**
  * @brief  Displays a relative poly-line (between many points).
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_draw_relative_poly_line
(
    /* Pointer to poly line draw structure. */
    drv_lcd_poly_line_draw_st_ptr line_ptr
)
{     
    uint16_t                    point_index = 0;
    drv_lcd_universal_line_draw_st uni_line = { LCD_FOREGROUND_LAYER };
    
    
    /* Error detection. */
    if ( (line_ptr == (void*)0) || (line_ptr->point_count < 2) )
    {
        return DRV_LCD_ERR_IOCTL;
    }
    
    /* Initialize fixed part of universal line structure. */
    uni_line.layer = line_ptr->layer;
    uni_line.color = line_ptr->color;
     
    uni_line.x1 = line_ptr->point_ptr[0].x;
    uni_line.y1 = line_ptr->point_ptr[0].y;

    /* Draw universal line based on 2 point. Caution: line number one less than point number. */
    for (point_index = 0; point_index < (line_ptr->point_count - 1); point_index ++)
    {
        /* Use relative coordinate. */
        uni_line.x2 = uni_line.x1 + line_ptr->point_ptr[point_index + 1].x;
        uni_line.y2 = uni_line.y1 + line_ptr->point_ptr[point_index + 1].y;

        /* Break the draw process when get error. */
        if (lcd_ioctl_draw_universal_line(&uni_line) == DRV_LCD_ERR_IOCTL)
        {
            break;
        }
        else
        {
            /* Update the specific segment's start endpoint. */
            uni_line.x1 = uni_line.x2;
            uni_line.y1 = uni_line.y2;
        }
    }
    
    /* Check if has finished the whole draw process. */
    if (point_index == (line_ptr->point_count - 1))
    {
        /* Draw line of (first point & last point) if line type is closed. */
        if (line_ptr->poly_line_type == LCD_POLY_LINE_CLOSED)
        {
            /* Only initialize (x1,y1),the (x2,y2) is valid. */
            uni_line.x1 = line_ptr->point_ptr[0].x;
            uni_line.y1 = line_ptr->point_ptr[0].y;
            
            lcd_ioctl_draw_universal_line(&uni_line);
        }
        
        return DRV_LCD_ERR_OK;
    }
    else
    {
        return DRV_LCD_ERR_IOCTL;
    }
}

 
/**
  * @brief  Displays a rectangle.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_draw_rectangle
(
    /* Pointer to rectangle draw structure. */
    drv_lcd_rectangle_draw_st_ptr rectangle_ptr
)
{
    /* Horizental line and vertical line structure. */
    drv_lcd_hv_line_draw_st line_h1 = { rectangle_ptr->layer, rectangle_ptr->x,rectangle_ptr->y, rectangle_ptr->width,1, 
                                        LCD_LINE_DIR_HORIZONTAL, rectangle_ptr->color };
    drv_lcd_hv_line_draw_st line_h2 = { rectangle_ptr->layer, rectangle_ptr->x,(rectangle_ptr->y + rectangle_ptr->height - 1), rectangle_ptr->width,1, 
                                        LCD_LINE_DIR_HORIZONTAL, rectangle_ptr->color };
    
    drv_lcd_hv_line_draw_st line_v1 = { rectangle_ptr->layer, rectangle_ptr->x,rectangle_ptr->y, rectangle_ptr->height,1, 
                                        LCD_LINE_DIR_VERTICAL, rectangle_ptr->color };
    drv_lcd_hv_line_draw_st line_v2 = { rectangle_ptr->layer, (rectangle_ptr->x + rectangle_ptr->width - 1),rectangle_ptr->y, rectangle_ptr->height,1, 
                                        LCD_LINE_DIR_VERTICAL, rectangle_ptr->color };


    /* Error detection. */
    if (rectangle_ptr == (void*)0)
    {
        return DRV_LCD_ERR_IOCTL;
    }

    /* draw horizontal lines */
    lcd_ioctl_draw_hv_line(&line_h1);
    lcd_ioctl_draw_hv_line(&line_h2);
    
    /* draw vertical lines */
    lcd_ioctl_draw_hv_line(&line_v1);
    lcd_ioctl_draw_hv_line(&line_v2);

    return DRV_LCD_ERR_OK;
}


/**
  * @brief  Draw 8 symmetrical pixels,only for routine draw circle.
  * @param  See below.
  * @retval None.
  */
static void draw_8_pixel_of_circle(int16_t xc, int16_t yc, int16_t x, int16_t y, drv_lcd_layer_value layer, uint16_t color)   
{
    drv_lcd_pixel_draw_st pixel = { layer, 0,0, color };
    uint8_t               index = 0;
    uint16_t test[16]           = { xc+x,yc+y, xc-x,yc+y, xc+x,yc-y, xc-x,yc-y, 
                                    xc+y,yc+x, xc-y,yc+x, xc+y,yc-x, xc-y,yc-x };
    
    /* Draw 8 symmetrical pixels. */
    for(index = 0; index < (sizeof(test) / sizeof(test[0])); index += 2)
    {
        pixel.x = test[index];
        pixel.y = test[index + 1];
        lcd_ioctl_draw_pixel(&pixel);
    }
}


/**
  * @brief  Displays a circle. Based on Bresenham's circle algorithm.
  * @param  See below.
  * @Caution This routine do not check the parameters' validation and user must ensure that,
             so application can draw the special circle at the border of lcd screen.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_draw_circle
(
    /* Pointer to circle draw structure. */
    drv_lcd_circle_draw_st_ptr circle_ptr
)
{
    int16_t  x = 0, y = 0;    /* dot coordinate for origin. */
    int16_t yi = 0, d = 0; 

    //uint8_t dot_num = 0, blank_num = 0;
    
    
    /* Only check the pointer data. */
    if(circle_ptr == (void*)0)
    {
        return DRV_LCD_ERR_IOCTL;
    }
    
    /* Initial the circle factor. */
    y = circle_ptr->r;
    d = 3 - 2 * circle_ptr->r;    
   
    if (circle_ptr->circle_type == LCD_CIRCLE_FILLED)  
    {   
        /* Draw the filled circle. */
        while(x <= y)   
        {    
            for(yi = x; yi <= y; yi ++)    
            {  
                draw_8_pixel_of_circle(circle_ptr->x, circle_ptr->y, x, yi, circle_ptr->layer, circle_ptr->color);
            } 
            
            if(d < 0)   
            {    
                d = d + 4 * x + 6;    
            }   
            else   
            {    
                d = d + 4 * (x - y) + 10;
                y --;    
            } 
            
            x++;    
        }    
    }
    else   
    {    
        /* Draw the empty circle. */
        while (x <= y)   
        {   
            draw_8_pixel_of_circle(circle_ptr->x, circle_ptr->y, x, y, circle_ptr->layer, circle_ptr->color);
           
            /*
                注释掉画虚线圆的代码，该代码会在45度对称位置，因为对称画而是两个空格或者实线连起来。尚未找到方案解决。
            if(dot_num < 5)
            {
                dot_num ++;
                blank_num = 0;
                
                draw_8_pixel_of_circle(circle_ptr->x, circle_ptr->y, x, y, circle_ptr->layer, circle_ptr->color);
            }
            else
            {
                blank_num ++;
                if(5 <= blank_num)
                {
                    dot_num = 0;
                }
            }
            */

            if(d < 0)   
            {    
                d = d + 4 * x + 6;    
            }   
            else   
            {    
                d = d + 4 * (x - y) + 10;    
                y --;    
            } 
            
            x ++;   
        }    
    } 

    return DRV_LCD_ERR_OK;
}   


/**
  * @brief  Draws a character on LCD.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_draw_universal_char
(
    /* Pointer to char draw structure. */
    drv_lcd_char_draw_st_ptr char_st_ptr
)
{
    uint32_t   line_index = 0,  dot_index = 0; 
    uint32_t  addr_offset = 0, addr_index = 0;

    /* Initialize character's start code. */
    const uint16_t *code_ptr = &(char_st_ptr->font_ptr->table_ptr[(char_st_ptr->ascii_char - ASCII_INDEX_IN_TABLE) * char_st_ptr->font_ptr->height]);
    
    
    /* Error detection. */
    if( (char_st_ptr == (void*)0) || (char_st_ptr->font_ptr == (void*)0)
      || (LCD_SCREEN_PIXEL_WIDTH < (char_st_ptr->column + char_st_ptr->font_ptr->width))
      || (LCD_SCREEN_PIXEL_HEIGHT < (char_st_ptr->line + char_st_ptr->font_ptr->height)) )
    {
        return DRV_LCD_ERR_IOCTL;
    }

    /* Initialize address offset and address index. */
    addr_offset = char_st_ptr->line * LCD_SCREEN_PIXEL_WIDTH * 2;
    addr_index += char_st_ptr->column;

    /* Draw the charecter's pixel based on line. */
    for (line_index = 0; line_index < char_st_ptr->font_ptr->height; line_index ++)
    {
        for (dot_index = 0; dot_index < char_st_ptr->font_ptr->width; dot_index ++)
        {
            /* Write data value to all SDRAM memory */
            if ( (((code_ptr[line_index] & ((0x80 << ((char_st_ptr->font_ptr->width / 12 ) * 8 ) ) >> dot_index)) == 0x00) && (char_st_ptr->font_ptr->width <= 12))
              || (((code_ptr[line_index] & (0x1 << dot_index)) == 0x00) && (char_st_ptr->font_ptr->width > 12 )) )
            {
                if (char_st_ptr->layer == LCD_BACKGROUND_LAYER)
                {
                    *(__IO uint16_t*) (LCD_FRAME_BUFFER_BACKGROUND + addr_offset + (2 * addr_index)) = char_st_ptr->back_color;
                }
                else if (char_st_ptr->layer == LCD_FOREGROUND_LAYER)
                {  
                    *(__IO uint16_t*) (LCD_FRAME_BUFFER_FOREGROUND + addr_offset + (2 * addr_index)) = char_st_ptr->back_color;
                }
                else
                {
                    *(__IO uint16_t*) (LCD_FRAME_BUFFER_BACKGROUND + addr_offset + (2 * addr_index)) = char_st_ptr->back_color;
                    *(__IO uint16_t*) (LCD_FRAME_BUFFER_FOREGROUND + addr_offset + (2 * addr_index)) = char_st_ptr->back_color;
                }
            }
            else
            { 
               if (char_st_ptr->layer == LCD_BACKGROUND_LAYER)
               {
                   *(__IO uint16_t*) (LCD_FRAME_BUFFER_BACKGROUND + addr_offset + (2 * addr_index)) = char_st_ptr->text_color;
               }
               else if (char_st_ptr->layer == LCD_FOREGROUND_LAYER)
               {  
                   *(__IO uint16_t*) (LCD_FRAME_BUFFER_FOREGROUND + addr_offset + (2 * addr_index)) = char_st_ptr->text_color;
               }
               else
               {
                   *(__IO uint16_t*) (LCD_FRAME_BUFFER_BACKGROUND + addr_offset + (2 * addr_index)) = char_st_ptr->text_color;
                   *(__IO uint16_t*) (LCD_FRAME_BUFFER_FOREGROUND + addr_offset + (2 * addr_index)) = char_st_ptr->text_color;
               }  
            }
            
            addr_index ++;
        }

        addr_index += (LCD_SCREEN_PIXEL_WIDTH - char_st_ptr->font_ptr->width);
    }

    return DRV_LCD_ERR_OK;
}


/**
  * @brief  Displays string on the LCD.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_display_string
(
    /* Pointer to string draw structure. */
    drv_lcd_string_display_st_ptr str_disp_ptr
)
{  
    char *char_dis_ptr = str_disp_ptr->string_ptr;
   
    drv_lcd_char_draw_st char_st = { str_disp_ptr->layer,           str_disp_ptr->line,    str_disp_ptr->column, 
                                     str_disp_ptr->back_color, str_disp_ptr->text_color, str_disp_ptr->font_ptr, 0 };
    
    
    /* Error detection. */
    if ( (str_disp_ptr == (void*)0) || (str_disp_ptr->string_ptr == (void*)0) ) 
    { 
        return DRV_LCD_ERR_IOCTL;
    }  
    
    /* Send the string character by character on lCD. */
    while (*char_dis_ptr != 0)
    {
        /* Set ascii char. */
        char_st.ascii_char = *char_dis_ptr;
        
        /* Display one character on LCD. */
        if(lcd_ioctl_draw_universal_char(&char_st) == DRV_LCD_ERR_IOCTL)
        {
           return DRV_LCD_ERR_IOCTL; 
        }
      
        /* Increment the column position by width. */
        char_st.column += str_disp_ptr->font_ptr->width;
      
        /* Point on the next character. */
        char_dis_ptr++;
    }
    
    return DRV_LCD_ERR_OK;
}


/**
  * @brief  Draws a block on LCD.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_draw_block
(
    /* Pointer to block draw structure. */
    drv_lcd_block_draw_st_ptr block_st_ptr
)
{
    uint32_t   line_index = 0,  dot_index = 0; 
    uint32_t  addr_offset = 0, addr_index = 0;
    
    
    /* Error detection. */
    if( (block_st_ptr == (void*)0) || (block_st_ptr->plane_ptr == (void*)0) )
    {
        return DRV_LCD_ERR_IOCTL;
    }

    /* Initialize address offset. */
    addr_offset = (block_st_ptr->line * LCD_SCREEN_PIXEL_WIDTH + block_st_ptr->column) * 2;

    /* Draw the charecter's pixel based on line. */
    for (line_index = 0; (line_index < block_st_ptr->plane_ptr->height) && ((line_index + block_st_ptr->line) < LCD_SCREEN_PIXEL_HEIGHT); line_index++)
    {
        for (dot_index = 0; (dot_index < block_st_ptr->plane_ptr->width) && ((dot_index + block_st_ptr->column) < LCD_SCREEN_PIXEL_WIDTH); dot_index ++)
        {
            addr_index = line_index * block_st_ptr->plane_ptr->width + dot_index;
            
            /* Write data value to all SDRAM memory. */
            if (block_st_ptr->layer == LCD_BACKGROUND_LAYER)
            {
                *(__IO uint16_t*) (LCD_FRAME_BUFFER_BACKGROUND + addr_offset + (2 * dot_index)) = block_st_ptr->plane_ptr->table_ptr[addr_index];
            }
            else if (block_st_ptr->layer == LCD_FOREGROUND_LAYER)
            {  
                *(__IO uint16_t*) (LCD_FRAME_BUFFER_FOREGROUND + addr_offset + (2 * dot_index)) = block_st_ptr->plane_ptr->table_ptr[addr_index];
            }
            else
            {
                *(__IO uint16_t*) (LCD_FRAME_BUFFER_BACKGROUND + addr_offset + (2 * dot_index)) = block_st_ptr->plane_ptr->table_ptr[addr_index];
                *(__IO uint16_t*) (LCD_FRAME_BUFFER_FOREGROUND + addr_offset + (2 * dot_index)) = block_st_ptr->plane_ptr->table_ptr[addr_index];
            }
        }

        addr_offset += LCD_SCREEN_PIXEL_WIDTH * 2;
    }

    return DRV_LCD_ERR_OK;
}






/**
  * @brief  Displays one character (16dots width, 24dots height).
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..29
  * @param  Column: start column address.
  * @param  Ascii: character ascii code, must be between 0x20 and 0x7E.
  * @retval None
  */
void LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii)
{
  Ascii -= 32;

  //LCD_DrawChar(Line, Column, &LCD_Currentfonts->table[Ascii * LCD_Currentfonts->Height]);
}









/**
  * @brief  Config and Sets the specific layer's color Keying.
  * @param  See below. 
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_set_colorkeying
(
    /* Pointer to colorkeying set structure. */
    drv_lcd_colorkeying_set_st_ptr colorkeying_ptr
)
{  
    LTDC_ColorKeying_InitTypeDef LTDC_colorkeying_InitStruct = { 0 };
    
    /* Key for background and foreground set.  */
    uint8_t          key_background = 0;
    uint8_t          key_foreground = 0;
    DRV_LCD_ERR_CODE       err_code = DRV_LCD_ERR_IOCTL;

    
    if (colorkeying_ptr == (void*)0)
    {
        err_code = DRV_LCD_ERR_IOCTL;
    }
    else
    {
        /* Configure the color Keying. */
        LTDC_colorkeying_InitStruct.LTDC_ColorKeyBlue = LCD_DISMISS_B(colorkeying_ptr->color);
        LTDC_colorkeying_InitStruct.LTDC_ColorKeyGreen = LCD_DISMISS_G(colorkeying_ptr->color);
        LTDC_colorkeying_InitStruct.LTDC_ColorKeyRed = LCD_DISMISS_R(colorkeying_ptr->color);  
        
        /* Initialize the key of background and foreground based on parameter. */
        switch (colorkeying_ptr->layer)
        {
            case LCD_BACKGROUND_LAYER:  {  key_background = 1;            
                                           key_foreground = 0;            break; }
            
            case LCD_FOREGROUND_LAYER:  {  key_background = 0; 
                                           key_foreground = 1;            break; }
            
            case LCD_BACK_FORE_LAYER:   {  key_background = 1;
                                           key_foreground = 1;            break; }
            
            default:                    {  key_background = 0;
                                           key_foreground = 0;  
                                           err_code = DRV_LCD_ERR_IOCTL;  break; }
        }
        
        if (key_background == 1)
        {   
            /* Enable the color Keying for Layer1. */
            LTDC_ColorKeyingConfig(LTDC_Layer1, &LTDC_colorkeying_InitStruct, ENABLE);
            LTDC_ReloadConfig(LTDC_IMReload);
            
            err_code = DRV_LCD_ERR_OK;
        }
        
        if (key_foreground == 1)
        {
            /* Enable the color Keying for Layer2. */
            LTDC_ColorKeyingConfig(LTDC_Layer2, &LTDC_colorkeying_InitStruct, ENABLE);
            LTDC_ReloadConfig(LTDC_IMReload);
            
            err_code = DRV_LCD_ERR_OK;
        }
    }
    
    return err_code;
}


/**
  * @brief  Resets the specific layer's color Keying.
  * @param  See below. 
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_reset_colorkeying
(
    /* Pointer to colorkeying set structure. */
    drv_lcd_colorkeying_reset_st_ptr colorkeying_ptr
)
{  
    LTDC_ColorKeying_InitTypeDef LTDC_colorkeying_InitStruct = { 0 };
    
    /* Key for background and foreground set.  */
    uint8_t          key_background = 0;
    uint8_t          key_foreground = 0;
    DRV_LCD_ERR_CODE       err_code = DRV_LCD_ERR_IOCTL;

    
    if (colorkeying_ptr == (void*)0)
    {
        err_code = DRV_LCD_ERR_IOCTL;
    }
    else
    {        
        /* Initialize the key of background and foreground based on parameter. */
        switch (colorkeying_ptr->layer)
        {
            case LCD_BACKGROUND_LAYER:  {  key_background = 1;            
                                           key_foreground = 0;            break; }
            
            case LCD_FOREGROUND_LAYER:  {  key_background = 0; 
                                           key_foreground = 1;            break; }
            
            case LCD_BACK_FORE_LAYER:   {  key_background = 1;
                                           key_foreground = 1;            break; }
            
            default:                    {  key_background = 0;
                                           key_foreground = 0;  
                                           err_code = DRV_LCD_ERR_IOCTL;  break; }
        }
        
        if (key_background == 1)
        {               
            /* Disable the color Keying for Layer1. */
            LTDC_ColorKeyingConfig(LTDC_Layer1, &LTDC_colorkeying_InitStruct, DISABLE);
            LTDC_ReloadConfig(LTDC_IMReload);

            err_code = DRV_LCD_ERR_OK;
        }
        
        if (key_foreground == 1)
        {
            /* Disable the color Keying for Layer2. */
            LTDC_ColorKeyingConfig(LTDC_Layer2, &LTDC_colorkeying_InitStruct, DISABLE);
            LTDC_ReloadConfig(LTDC_IMReload);
            
            err_code = DRV_LCD_ERR_OK;
        }
    }
    
    return err_code;
}







/**
  * @brief  Clears the whole LCD screen.
  * @param  See below. 
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE lcd_ioctl_clear_layer
(
    /* Pointer to screen clear structure. */
    drv_lcd_layer_clear_st_ptr layer_ptr
)
{
    DRV_LCD_ERR_CODE err_code = DRV_LCD_ERR_IOCTL;
    uint32_t            index = 0;

    
    if (layer_ptr == (void*)0)
    {
        err_code = DRV_LCD_ERR_IOCTL;
    }
    else
    {
        switch(layer_ptr->layer)
        {
            case LCD_BACKGROUND_LAYER:
            {
                /* Erase background lcd buffer memory. */
                for (index = 0x00; index < LCD_FRAME_SIZE_BACKGROUND; index ++)
                {
                    *(__IO uint16_t*)(LCD_FRAME_BUFFER_BACKGROUND + (2 * index)) = layer_ptr->color;
                }
                err_code = DRV_LCD_ERR_OK;
                break;                
            }
            case LCD_FOREGROUND_LAYER:
            {
                /* Erase foreground lcd buffer memory. */
                for (index = 0x00; index < LCD_FRAME_SIZE_FOREGROUND; index ++)
                {
                    *(__IO uint16_t*)(LCD_FRAME_BUFFER_FOREGROUND + (2 * index)) = layer_ptr->color;
                }
                err_code = DRV_LCD_ERR_OK;
                break; 
            }
            case LCD_BACK_FORE_LAYER:
            {
                /* Erase background and foreground lcd buffer memory. */
                for (index = 0x00; index < (LCD_FRAME_SIZE_BACKGROUND + LCD_FRAME_SIZE_FOREGROUND); index ++)
                {
                    *(__IO uint16_t*)(LCD_FRAME_BUFFER_BACKGROUND + (2 * index)) = layer_ptr->color;
                }
                err_code = DRV_LCD_ERR_OK;
                break;                
            }
            default:
            {
                err_code = DRV_LCD_ERR_IOCTL;
                break;  
            }
        }
    }
    
    return err_code; 
}









/**
  * @brief  Main routine for lcd driver open.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_OPEN.
  */
static DRV_LCD_ERR_CODE drv_lcd_open
(
	/* Pointer of device driver structure. */
	void *drv_ptr
)
{
    /* Initialize the LCD. */
    LCD_init();
    
    return DRV_LCD_ERR_OK;
}


/**
  * @brief  Main routine for lcd driver close.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_CLOSE.
  */
static DRV_LCD_ERR_CODE drv_lcd_close
(
	/* Pointer of device driver structure. */
	void *drv_ptr
)
{
    return DRV_LCD_ERR_CLOSE;
}


/**
  * @brief  Main routine for lcd driver read.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_READ.
  */
static DRV_LCD_ERR_CODE drv_lcd_read
(
	/* Pointer of device driver structure. */
	void *drv_ptr,
    
    /* Data address for data fetching. */
    uint32_t addr,
    
    /* Data destination buffer address. */
    uint32_t * buffer_ptr,
    
    /* Data size based on uint8_t. */
    uint32_t size
)
{
    return DRV_LCD_ERR_READ;
}


/**
  * @brief  Main routine for lcd driver write.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_WRITE.
  */
static DRV_LCD_ERR_CODE drv_lcd_write
(
    /* Pointer of device driver structure. */
	void *drv_ptr,
    
    /* Data address for data writing. */
    uint32_t addr,
    
    /* Data source buffer address. */
    uint32_t * buffer_ptr,
    
    /* Data size based on uint8_t. */
    uint32_t size
)
{
    return DRV_LCD_ERR_WRITE;
}



/**
  * @brief  Main routine for lcd driver ioctl.
  * @param  See below.
  * @retval DRV_LCD_ERR_OK or DRV_LCD_ERR_IOCTL.
  */
static DRV_LCD_ERR_CODE drv_lcd_ioctl
(
	/* Pointer of device driver structure. */
	void *drv_ptr, 
	
	/* Ioctl command. */
	DRV_LCD_IOCTL_CMD cmd, 
	
	/* Pointer of parameter. */
	void *param_ptr
)
{
	DRV_LCD_ERR_CODE err_code = DRV_LCD_ERR_IOCTL;
	
	
	switch (cmd)
	{
        /* set the specific layer's transparency. */
		case LCD_IOCTL_SET_LAYER_TRANSPARENCY:      {  err_code = lcd_ioctl_set_layer_transparency(param_ptr);  break;  }
       
        
        /* draw a pixel to lcd layer. */
        case LCD_IOCTL_DRAW_PIXEL:                  {  err_code = lcd_ioctl_draw_pixel(param_ptr);              break;  }    
       
        /* draw a horizental/vertical line to lcd layer. */
        case LCD_IOCTL_DRAW_HV_LINE:                {  err_code = lcd_ioctl_draw_hv_line(param_ptr);            break;  }
       
        /* draw a universal line to lcd layer. */
        case LCD_IOCTL_DRAW_UNIVERSAL_LINE:         {  err_code = lcd_ioctl_draw_universal_line(param_ptr);     break;  }
        
        /* draw an absolute poly line to lcd layer. */
        case LCD_IOCTL_DRAW_ABSOLUTE_POLY_LINE:     {  err_code = lcd_ioctl_draw_absolute_poly_line(param_ptr); break;  }
                
        /* draw a relative poly line to lcd layer. */
        case LCD_IOCTL_DRAW_RELATIVE_POLY_LINE:     {  err_code = lcd_ioctl_draw_relative_poly_line(param_ptr); break;  }

        /* draw a rectangle to lcd layer. */
        case LCD_IOCTL_DRAW_RECTANGLE:              {  err_code = lcd_ioctl_draw_rectangle(param_ptr);          break;  }
       
        /* draw a circle to lcd layer. */
        case LCD_IOCTL_DRAW_CIRCLE:                 {  err_code = lcd_ioctl_draw_circle(param_ptr);             break;  }

        /* draw a charecter to lcd layer. */
        case LCD_IOCTL_DRAW_UNIVERSAL_CHAR:         {  err_code = lcd_ioctl_draw_universal_char(param_ptr);     break;  }

        /* display string to lcd layer. */
        case LCD_IOCTL_DISPLAY_STRING:              {  err_code = lcd_ioctl_display_string(param_ptr);          break;  }

        /* draw a block to lcd layer. */
        case LCD_IOCTL_DRAW_BLOCK:                  {  err_code = lcd_ioctl_draw_block(param_ptr);              break;  }
        
        /* set the specific layer's color keying. */
        case LCD_IOCTL_SET_COLORKEYING:             {  err_code = lcd_ioctl_set_colorkeying(param_ptr);         break;  }
        
        /* set the specific layer's color keying. */
        case LCD_IOCTL_RESET_COLORKEYING:           {  err_code = lcd_ioctl_reset_colorkeying(param_ptr);       break;  }


        /* clear the lcd screen. */
        case LCD_IOCTL_CLEAR_LAYER:                 {  err_code = lcd_ioctl_clear_layer(param_ptr);             break;  }

			
		default:                                    {  err_code = DRV_LCD_ERR_IOCTL;                            break;	}
			
	}
	
	return err_code;
}





#if 0
















/**
  * @brief  Clears the selected line.
  * @param  Line: the Line to be cleared.
  *   This parameter can be one of the following values:
  *     @arg LCD_LINE_x: where x can be: 0..13 if LCD_Currentfonts is Font16x24
  *                                      0..26 if LCD_Currentfonts is Font12x12 or Font8x12
  *                                      0..39 if LCD_Currentfonts is Font8x8
  * @retval None
  */
void LCD_ClearLine(uint16_t Line)
{
  uint16_t refcolumn = 0;
    
  /* Send the string character by character on lCD */
  while ((refcolumn < LCD_SCREEN_PIXEL_WIDTH) && (((refcolumn + LCD_Currentfonts->Width)& 0xFFFF) >= LCD_Currentfonts->Width))
  {
    /* Display one character on LCD */
    LCD_DisplayChar(Line, refcolumn, ' ');
    /* Decrement the column position by 16 */
    refcolumn += LCD_Currentfonts->Width;
  }
}













/**
  * @brief  Sets a display window
  * @param  Xpos: specifies the X bottom left position from 0 to 240.
  * @param  Ypos: specifies the Y bottom left position from 0 to 320.
  * @param  Height: display window height, can be a value from 0 to 320.
  * @param  Width: display window width, can be a value from 0 to 240.
  * @retval None
  */
void LCD_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width)
{

  if (CurrentLayer == LCD_BACKGROUND_LAYER)
  { 
    /* reconfigure the layer1 position */
    LTDC_LayerPosition(LTDC_Layer1, Xpos, Ypos);
    LTDC_ReloadConfig(LTDC_IMReload);
    
    /* reconfigure the layer1 size */
    LTDC_LayerSize(LTDC_Layer1, Width, Height);
    LTDC_ReloadConfig(LTDC_IMReload);
 }
 else
 {   
    /* reconfigure the layer2 position */
    LTDC_LayerPosition(LTDC_Layer2, Xpos, Ypos);
    LTDC_ReloadConfig(LTDC_IMReload); 
   
   /* reconfigure the layer2 size */
    LTDC_LayerSize(LTDC_Layer2, Width, Height);
    LTDC_ReloadConfig(LTDC_IMReload);
  }
}

/**
  * @brief  Disables LCD Window mode.
  * @param  None
  * @retval None
  */
void LCD_WindowModeDisable(void)
{
  LCD_SetDisplayWindow(0, 0, LCD_SCREEN_PIXEL_HEIGHT, LCD_SCREEN_PIXEL_WIDTH); 
}


/**
  * @brief  Draw a circle.
  * @param  Xpos: specifies the X position, can be a value from 0 to 240.
  * @param  Ypos: specifies the Y position, can be a value from 0 to 320.
  * @param  Radius: radius of the circle.
  * @retval None
  */
void LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
    int x = -Radius, y = 0, err = 2 - 2 * Radius, e2;
    
    
    do {
        *(__IO uint16_t*) (CurrentFrameBuffer + (2 * ((Xpos-x) + LCD_SCREEN_PIXEL_WIDTH*(Ypos+y)))) = CurrentTextColor; 
        *(__IO uint16_t*) (CurrentFrameBuffer + (2 * ((Xpos+x) + LCD_SCREEN_PIXEL_WIDTH*(Ypos+y)))) = CurrentTextColor;
        *(__IO uint16_t*) (CurrentFrameBuffer + (2 * ((Xpos+x) + LCD_SCREEN_PIXEL_WIDTH*(Ypos-y)))) = CurrentTextColor;
        *(__IO uint16_t*) (CurrentFrameBuffer + (2 * ((Xpos-x) + LCD_SCREEN_PIXEL_WIDTH*(Ypos-y)))) = CurrentTextColor;
      
        e2 = err;
        if (e2 <= y)  
        {
            err += ++y * 2 + 1;
            if (-x == y && e2 <= x)
            {
                e2 = 0;
            }               
        }
        if (e2 > x) 
        {
            err += ++x * 2 + 1;
        }
    }
    while (x <= 0);
}















/**
  * @brief  Draw a full ellipse.
  * @param  Xpos: specifies the X position, can be a value from 0 to 240.
  * @param  Ypos: specifies the Y position, can be a value from 0 to 320.
  * @param  Radius: minor radius of ellipse.
  * @param  Radius2: major radius of ellipse.  
  * @retval None
  */
void LCD_DrawFullEllipse(int Xpos, int Ypos, int Radius, int Radius2)
{
  int x = -Radius, y = 0, err = 2-2*Radius, e2;
  float K = 0, rad1 = 0, rad2 = 0;
  
  rad1 = Radius;
  rad2 = Radius2;
  
  if (Radius > Radius2)
  { 
    do 
    {
      K = (float)(rad1/rad2);
//      LCD_DrawLine((Xpos+x), (Ypos-(uint16_t)(y/K)), (2*(uint16_t)(y/K) + 1), LCD_LINE_DIR_VERTICAL);
//      LCD_DrawLine((Xpos-x), (Ypos-(uint16_t)(y/K)), (2*(uint16_t)(y/K) + 1), LCD_LINE_DIR_VERTICAL);
      
      e2 = err;
      if (e2 <= y) 
      {
        err += ++y*2+1;
        if (-x == y && e2 <= x) e2 = 0;
      }
      if (e2 > x) err += ++x*2+1;
      
    }
    while (x <= 0);
  }
  else
  {
    y = -Radius2; 
    x = 0;
    do 
    { 
      K = (float)(rad2/rad1);       
//      LCD_DrawLine((Xpos-(uint16_t)(x/K)), (Ypos+y), (2*(uint16_t)(x/K) + 1), LCD_LINE_DIR_HORIZONTAL);
//      LCD_DrawLine((Xpos-(uint16_t)(x/K)), (Ypos-y), (2*(uint16_t)(x/K) + 1), LCD_LINE_DIR_HORIZONTAL);
      
      e2 = err;
      if (e2 <= x) 
      {
        err += ++x*2+1;
        if (-y == x && e2 <= y) e2 = 0;
      }
      if (e2 > y) err += ++y*2+1;
    }
    while (y <= 0);
  }
}

/**
  * @brief  Displays an Ellipse.
  * @param  Xpos: specifies the X position, can be a value from 0 to 240.
  * @param  Ypos: specifies the Y position, can be a value from 0 to 320.
  * @param  Radius: specifies Radius.
  * @param  Radius2: specifies Radius2.
  * @retval None
  */
void LCD_DrawEllipse(int Xpos, int Ypos, int Radius, int Radius2)
{
  int x = -Radius, y = 0, err = 2-2*Radius, e2;
  float K = 0, rad1 = 0, rad2 = 0;
   
  rad1 = Radius;
  rad2 = Radius2;
  
  if (Radius > Radius2)
  { 
    do {
      K = (float)(rad1/rad2);
      *(__IO uint16_t*) (CurrentFrameBuffer + (2*((Xpos-x) + LCD_SCREEN_PIXEL_WIDTH*(Ypos+(uint16_t)(y/K))))) = CurrentTextColor; 
      *(__IO uint16_t*) (CurrentFrameBuffer + (2*((Xpos+x) + LCD_SCREEN_PIXEL_WIDTH*(Ypos+(uint16_t)(y/K))))) = CurrentTextColor;
      *(__IO uint16_t*) (CurrentFrameBuffer + (2*((Xpos+x) + LCD_SCREEN_PIXEL_WIDTH*(Ypos-(uint16_t)(y/K))))) = CurrentTextColor;
      *(__IO uint16_t*) (CurrentFrameBuffer + (2*((Xpos-x) + LCD_SCREEN_PIXEL_WIDTH*(Ypos-(uint16_t)(y/K))))) = CurrentTextColor;
            
      e2 = err;
      if (e2 <= y) {
        err += ++y*2+1;
        if (-x == y && e2 <= x) e2 = 0;
      }
      if (e2 > x) err += ++x*2+1;
    }
    while (x <= 0);
  }
  else
  {
    y = -Radius2; 
    x = 0;
    do { 
      K = (float)(rad2/rad1);
      *(__IO uint16_t*) (CurrentFrameBuffer + (2*((Xpos-(uint16_t)(x/K)) + LCD_SCREEN_PIXEL_WIDTH*(Ypos+y)))) = CurrentTextColor; 
      *(__IO uint16_t*) (CurrentFrameBuffer + (2*((Xpos+(uint16_t)(x/K)) + LCD_SCREEN_PIXEL_WIDTH*(Ypos+y)))) = CurrentTextColor;
      *(__IO uint16_t*) (CurrentFrameBuffer + (2*((Xpos+(uint16_t)(x/K)) + LCD_SCREEN_PIXEL_WIDTH*(Ypos-y)))) = CurrentTextColor;
      *(__IO uint16_t*) (CurrentFrameBuffer + (2*((Xpos-(uint16_t)(x/K)) + LCD_SCREEN_PIXEL_WIDTH*(Ypos-y)))) = CurrentTextColor;
      
      e2 = err;
      if (e2 <= x) {
        err += ++x*2+1;
        if (-y == x && e2 <= y) e2 = 0;
      }
      if (e2 > y) err += ++y*2+1;     
    }
    while (y <= 0);
  }
}


/**
  * @brief  Displays a bitmap picture loaded in the internal Flash.
  * @param  BmpAddress: Bmp picture address in the internal Flash.
  * @retval None
  */
void LCD_WriteBMP(uint32_t BmpAddress)
{
    uint32_t index = 0, size = 0, width = 240, height = 320, bit_pixel = 16;
    uint32_t Address;
    uint32_t currentline = 0, linenumber = 0;

    Address = LCD_FRAME_BUFFER + LCD_BUFFER_OFFSET;

#if 0
    /* Read bitmap size. */
    size = *(__IO uint16_t *) (BmpAddress + 2);
    size |= (*(__IO uint16_t *) (BmpAddress + 4)) << 16;

    /* Get bitmap data address offset. */
    index = *(__IO uint16_t *) (BmpAddress + 10);
    index |= (*(__IO uint16_t *) (BmpAddress + 12)) << 16;

    
    /* Read bitmap width. */
    width = *(uint16_t *) (BmpAddress + 18);
    width |= (*(uint16_t *) (BmpAddress + 20)) << 16;

    /* Read bitmap height. */
    height = *(uint16_t *) (BmpAddress + 22);
    height |= (*(uint16_t *) (BmpAddress + 24)) << 16;

    /* Read bit/pixel. */
    bit_pixel = *(uint16_t *) (BmpAddress + 28);  
    
#endif

    if (0)//(CurrentLayer == LCD_BACKGROUND_LAYER)
    {
        /* reconfigure layer size in accordance with the picture. */
        LTDC_LayerSize(LTDC_Layer1, width, height);
        LTDC_ReloadConfig(LTDC_VBReload);

        /* Reconfigure the Layer pixel format in accordance with the picture. */    
        if ((bit_pixel/8) == 4)
        {
            LTDC_LayerPixelFormat(LTDC_Layer1, LTDC_Pixelformat_ARGB8888);
        }
        else if ((bit_pixel/8) == 2)
        {
            LTDC_LayerPixelFormat(LTDC_Layer1, LTDC_Pixelformat_RGB565);
        }
        else 
        {
            LTDC_LayerPixelFormat(LTDC_Layer1, LTDC_Pixelformat_RGB888);
        } 
        LTDC_ReloadConfig(LTDC_VBReload);
    }
    else  
    {
        /* reconfigure layer size in accordance with the picture. */
        LTDC_LayerSize(LTDC_Layer2, width, height);
        LTDC_ReloadConfig(LTDC_VBReload); 

        /* Reconfigure the Layer pixel format in accordance with the picture. */
        if ((bit_pixel/8) == 4)
        {
          LTDC_LayerPixelFormat(LTDC_Layer2, LTDC_Pixelformat_ARGB8888);
        }
        else if ((bit_pixel/8) == 2)
        {
          LTDC_LayerPixelFormat(LTDC_Layer2, LTDC_Pixelformat_RGB565);
        }
        else
        {
          LTDC_LayerPixelFormat(LTDC_Layer2, LTDC_Pixelformat_RGB888);  
        }
        LTDC_ReloadConfig(LTDC_VBReload);
    }

    /* compute the real size of the picture (without the header)). */  
    size = (size - index); 

    /* bypass the bitmap header. */
    BmpAddress += index;

    /* start copie image from the bottom. */
    Address += width * (height-1) * (bit_pixel/8);

 //   for(index = 0; index < size; index++)
    for(index = 0; index < 240*320*2; index++)
    {
        *(__IO uint8_t*) (Address) = *(__IO uint8_t *)BmpAddress;

        /*jump on next byte */   
        BmpAddress++;
        Address++;
        currentline++;

        if((currentline/(bit_pixel/8)) == width)
        {
            if(linenumber < height)
            {
                linenumber++;
                Address -=(2*width*(bit_pixel/8));
                currentline = 0;
            }
        }
    }
}

/**
  * @brief  Displays a full rectangle.
  * @param  Xpos: specifies the X position, can be a value from 0 to 240.
  * @param  Ypos: specifies the Y position, can be a value from 0 to 320.
  * @param  Height: rectangle height.
  * @param  Width: rectangle width.
  * @retval None
  */
void LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  DMA2D_InitTypeDef      DMA2D_InitStruct;
  
  uint32_t  Xaddress = 0; 
  uint16_t Red_Value = 0, Green_Value = 0, Blue_Value = 0;
 
  Red_Value = (0xF800 & CurrentTextColor) >> 11;
  Blue_Value = 0x001F & CurrentTextColor;
  Green_Value = (0x07E0 & CurrentTextColor) >> 5;
  
  Xaddress = CurrentFrameBuffer + 2*(LCD_SCREEN_PIXEL_WIDTH*Ypos + Xpos);
  
  /* configure DMA2D */
  DMA2D_DeInit();
  DMA2D_InitStruct.DMA2D_Mode = DMA2D_R2M;       
  DMA2D_InitStruct.DMA2D_CMode = DMA2D_RGB565;      
  DMA2D_InitStruct.DMA2D_OutputGreen = Green_Value;      
  DMA2D_InitStruct.DMA2D_OutputBlue = Blue_Value;     
  DMA2D_InitStruct.DMA2D_OutputRed = Red_Value;                
  DMA2D_InitStruct.DMA2D_OutputAlpha = 0x0F;                  
  DMA2D_InitStruct.DMA2D_OutputMemoryAdd = Xaddress;                
  DMA2D_InitStruct.DMA2D_OutputOffset = (LCD_SCREEN_PIXEL_WIDTH - Width);                
  DMA2D_InitStruct.DMA2D_NumberOfLine = Height;            
  DMA2D_InitStruct.DMA2D_PixelPerLine = Width;
  DMA2D_Init(&DMA2D_InitStruct); 
  
  /* Start Transfer */ 
  DMA2D_StartTransfer();
  
  /* Wait for CTC Flag activation */
  while(DMA2D_GetFlagStatus(DMA2D_FLAG_TC) == RESET)
  {
  } 

//  LCD_SetTextColor(CurrentTextColor);
}

/**
  * @brief  Displays a full circle.
  * @param  Xpos: specifies the X position, can be a value from 0 to 240.
  * @param  Ypos: specifies the Y position, can be a value from 0 to 320.
  * @param  Radius
  * @retval None
  */
void LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;    /* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (Radius << 1);
  
  CurX = 0;
  CurY = Radius;
  
  while (CurX <= CurY)
  {
    if(CurY > 0) 
    {
//      LCD_DrawLine(Xpos - CurX, Ypos - CurY, 2*CurY, LCD_LINE_DIR_VERTICAL);
//      LCD_DrawLine(Xpos + CurX, Ypos - CurY, 2*CurY, LCD_LINE_DIR_VERTICAL);
    }
    
    if(CurX > 0) 
    {
//      LCD_DrawLine(Xpos - CurY, Ypos - CurX, 2*CurX, LCD_LINE_DIR_VERTICAL);
//      LCD_DrawLine(Xpos + CurY, Ypos - CurX, 2*CurX, LCD_LINE_DIR_VERTICAL);
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
  
  LCD_DrawCircle(Xpos, Ypos, Radius);  
}


/**
  * @brief  Displays an triangle.
  * @param  Points: pointer to the points array.
  * @retval None
  */
void LCD_Triangle(pPoint Points, uint16_t PointCount)
{
  int16_t X = 0, Y = 0;
  pPoint First = Points;

  if(PointCount != 3)
  {
    return;
  }

  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
//    LCD_DrawUniLine(X, Y, Points->X, Points->Y);
  }
//  LCD_DrawUniLine(First->X, First->Y, Points->X, Points->Y);
}

/**
  * @brief  Fill an triangle (between 3 points).
  * @param  x1..3: x position of triangle point 1..3.
  * @param  y1..3: y position of triangle point 1..3.
  * @retval None
  */
void LCD_FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3)
{ 
  
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0, 
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0, 
  curpixel = 0;
  
//  deltax = ABS(x2 - x1);        /* The difference between the x's */
//  deltay = ABS(y2 - y1);        /* The difference between the y's */
  x = x1;                       /* Start x off at the first pixel */
  y = y1;                       /* Start y off at the first pixel */
  
  if (x2 >= x1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }
  
  if (y2 >= y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }
  
  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }
  
  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
//    LCD_DrawUniLine(x, y, x3, y3);
    
    num += numadd;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
  }  
  
  
}




/**
  * @brief  Displays a  full poly-line (between many points).
  * @param  Points: pointer to the points array.
  * @param  PointCount: Number of points.
  * @retval None
  */
void LCD_FillPolyLine(pPoint Points, uint16_t PointCount)
{
 
  int16_t X = 0, Y = 0, X2 = 0, Y2 = 0, X_center = 0, Y_center = 0, X_first = 0, Y_first = 0, pixelX = 0, pixelY = 0, counter = 0;
  uint16_t  IMAGE_LEFT = 0, IMAGE_RIGHT = 0, IMAGE_TOP = 0, IMAGE_BOTTOM = 0;  

  IMAGE_LEFT = IMAGE_RIGHT = Points->X;
  IMAGE_TOP= IMAGE_BOTTOM = Points->Y;

  for(counter = 1; counter < PointCount; counter++)
  {
    pixelX = POLY_X(counter);
    if(pixelX < IMAGE_LEFT)
    {
      IMAGE_LEFT = pixelX;
    }
    if(pixelX > IMAGE_RIGHT)
    {
      IMAGE_RIGHT = pixelX;
    }
    
    pixelY = POLY_Y(counter);
    if(pixelY < IMAGE_TOP)
    { 
      IMAGE_TOP = pixelY;
    }
    if(pixelY > IMAGE_BOTTOM)
    {
      IMAGE_BOTTOM = pixelY;
    }
  }  
  
  if(PointCount < 2)
  {
    return;
  }
  
  X_center = (IMAGE_LEFT + IMAGE_RIGHT)/2;
  Y_center = (IMAGE_BOTTOM + IMAGE_TOP)/2;
 
  X_first = Points->X;
  Y_first = Points->Y;
  
  while(--PointCount)
  {
    X = Points->X;
    Y = Points->Y;
    Points++;
    X2 = Points->X;
    Y2 = Points->Y;    
  
    LCD_FillTriangle(X, X2, X_center, Y, Y2, Y_center);
    LCD_FillTriangle(X, X_center, X2, Y, Y_center, Y2);
    LCD_FillTriangle(X_center, X2, X, Y_center, Y2, Y);   
  }
  
  LCD_FillTriangle(X_first, X2, X_center, Y_first, Y2, Y_center);
  LCD_FillTriangle(X_first, X_center, X2, Y_first, Y_center, Y2);
  LCD_FillTriangle(X_center, X2, X_first, Y_center, Y2, Y_first); 
}


#endif



/* Main object for lcd driver. */
drv_lcd_st LcdDriver = { drv_lcd_open, drv_lcd_close, drv_lcd_read, drv_lcd_write, drv_lcd_ioctl, LCD_FOREGROUND_LAYER };














