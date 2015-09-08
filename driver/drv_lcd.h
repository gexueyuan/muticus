/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : drv_lcd.h
 @brief  : this file include the driver variables and functions prototypes for 
           the lcd module.
 @author : wangxianwen
 @history:
           2015-5-19    wangxianwen    Created file
           ...
******************************************************************************/

/* Define to prevent recursive inclusion. */
#ifndef _DRV_LCD_H_
#define _DRV_LCD_H_

#ifdef __cplusplus
 extern "C" {
#endif 


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"


 


/** @defgroup Constants
  * @{
  */ 


/* LCD color. */ 
#define LCD_COLOR_WHITE              0xFFFF
#define LCD_COLOR_BLACK              0x0000
     
#define LCD_COLOR_RED                0xF800 
#define LCD_COLOR_GREEN              0x07E0
#define LCD_COLOR_BLUE               0x001F

#define LCD_COLOR_BLUE2              0x051F
#define LCD_COLOR_GREY               0xF7DE
#define LCD_COLOR_MAGENTA            0xF81F
#define LCD_COLOR_CYAN               0x7FFF
#define LCD_COLOR_YELLOW             0xFFE0


/* LCD transparency. Caution: The user defined value should in the range of NO to FULL. */
#define LCD_TRANSPARENCY_NO          0xFF
#define LCD_TRANSPARENCY_FULL        0x00


/* LCD screen's pixel size. */
#define LCD_SCREEN_PIXEL_WIDTH       LCD_PIXEL_WIDTH
#define LCD_SCREEN_PIXEL_HEIGHT      LCD_PIXEL_HEIGHT     

/* LCD frame buffer and size for background and foreground. */
#define LCD_FRAME_BUFFER_BACKGROUND  (LCD_FRAME_BUFFER) 
#define LCD_FRAME_SIZE_BACKGROUND    (LCD_BUFFER_OFFSET)
     
#define LCD_FRAME_BUFFER_FOREGROUND  (LCD_FRAME_BUFFER + LCD_BUFFER_OFFSET)
#define LCD_FRAME_SIZE_FOREGROUND    (LCD_BUFFER_OFFSET)

/* RGB: 565 module - 16bit. */
#define LCD_ASSEMBLE_RGB(R, G, B)    (((((uint16_t)R)& 0xF8) << 8) | ((((uint16_t)G) & 0xFC) << 3) | ((((uint16_t)B) & 0xF8) >> 3))  
#define LCD_DISMISS_R(RGB)           (((uint16_t)RGB & 0xF800) >> 11)
#define LCD_DISMISS_G(RGB)           (((uint16_t)RGB & 0x07E0) >> 5)
#define LCD_DISMISS_B(RGB)           ((uint16_t)RGB & 0x001F)


/* Ascii char index in fonts table. */
#define ASCII_INDEX_IN_TABLE         32    

/**
  * @}
  */







/* Error code for lcd device driver. */
typedef enum _drv_lcd_err_code
{
    DRV_LCD_ERR_OK = 0x0010,
    
    DRV_LCD_ERR_OPEN,
    DRV_LCD_ERR_CLOSE,
    DRV_LCD_ERR_READ,
    DRV_LCD_ERR_WRITE,
    DRV_LCD_ERR_IOCTL
    
}DRV_LCD_ERR_CODE, *DRV_LCD_ERR_CODE_PTR;

#define DRV_LCD_ERR_CODE_LEN    (sizeof(DRV_LCD_ERR_CODE))





 /* lcd layer enum definition. */    
typedef enum _drv_lcd_layer_value     
{
    LCD_BACKGROUND_LAYER,
    LCD_FOREGROUND_LAYER,
    LCD_BACK_FORE_LAYER     /* LCD_BACKGROUND_LAYER + LCD_FOREGROUND_LAYER. */
    
}drv_lcd_layer_value, *drv_lcd_layer_value_ptr;

#define DRV_LCD_LAYER_VALUE_LEN    (sizeof(drv_lcd_layer_value))


 /* lcd line direction enum definition. */    
typedef enum _drv_lcd_line_dir_value     
{
    /* LCD line direction. */ 
    LCD_LINE_DIR_HORIZONTAL,
    LCD_LINE_DIR_VERTICAL
    
}drv_lcd_line_dir_value, *drv_lcd_line_dir_value_ptr;

#define DRV_LCD_LINE_DIR_VALUE_LEN    (sizeof(drv_lcd_line_dir_value))




/* Cmd for lcd device ioctl. */
typedef enum dev_lcd_ioctl_cmd
{       
    /* ioctl command: set layer transparency. */
	LCD_IOCTL_SET_LAYER_TRANSPARENCY,

    /* ioctl command: draw a pixel to lcd layer. */
    LCD_IOCTL_DRAW_PIXEL,

    /* ioctl command: draw a horizontal/vertical line. */
    LCD_IOCTL_DRAW_HV_LINE,   
    /* ioctl command: draw a universal line to lcd layer. */
    LCD_IOCTL_DRAW_UNIVERSAL_LINE,
    /* ioctl command: draw an absolute poly line. */
    LCD_IOCTL_DRAW_ABSOLUTE_POLY_LINE,
    /* ioctl command: draw an absolute poly line. */
    LCD_IOCTL_DRAW_RELATIVE_POLY_LINE,
    /* ioctl command: draw a rectangle. */
    LCD_IOCTL_DRAW_RECTANGLE,
    /* ioctl command: draw a circle. */
    LCD_IOCTL_DRAW_CIRCLE,

    /* ioctl command: draw a universal char. */
    LCD_IOCTL_DRAW_UNIVERSAL_CHAR,
    /* ioctl command: draw a ascii char. */
    LCD_IOCTL_DRAW_ASCII_CHAR,
    /* ioctl command: display string. */
    LCD_IOCTL_DISPLAY_STRING,
    /* ioctl command: draw a block. */
    LCD_IOCTL_DRAW_BLOCK,

    /* ioctl command: set specific layer's color keying. */
    LCD_IOCTL_SET_COLORKEYING,  
    /* ioctl command: reset specific layer's color keying. */
    LCD_IOCTL_RESET_COLORKEYING,

    /* ioctl command: clear the lcd screen. */
    LCD_IOCTL_CLEAR_LAYER
	
}DRV_LCD_IOCTL_CMD, *DRV_LCD_IOCTL_CMD_PTR;


/* Device driver structure for lcd device. */
typedef struct _drv_lcd_st
{	
    /* Object's operation group. */
    DRV_LCD_ERR_CODE (*open) (void *);
    DRV_LCD_ERR_CODE (*close)(void *);
    DRV_LCD_ERR_CODE (*read) (void *, uint32_t, uint32_t *, uint32_t);
    DRV_LCD_ERR_CODE (*write)(void *, uint32_t, uint32_t *, uint32_t);
    DRV_LCD_ERR_CODE (*ioctl)(void *, DRV_LCD_IOCTL_CMD, void *);
        
    /* active layer for lcd. */
    drv_lcd_layer_value  active_layer;
	
}drv_lcd_st, *drv_lcd_st_ptr;

#define DRV_LCD_ST_LEN    (sizeof(drv_lcd_st))



typedef struct
{
    int16_t X;
    int16_t Y;
    
} Point, * pPoint;   


/* Lcd pixel coordinate structure. */
typedef struct _drv_lcd_point_st
{
    /* Coordinate for a pixel. 
       Caution: the coordinate can be absolutely or relatively. so use "int16_t" */
    int16_t x;
    int16_t y;
    
}drv_lcd_point_st, *drv_lcd_point_st_ptr;  

#define DRV_LCD_POINT_ST_LEN    (sizeof(drv_lcd_point_st))


/* Lcd plane structure. */
typedef struct _drv_lcd_plane_st
{
    /* Plane's code details start address. */
    uint16_t  * table_ptr;

    /* Plane's size information. */
    uint16_t        width;
    uint16_t       height;
    
}drv_lcd_plane_st, *drv_lcd_plane_st_ptr;

#define DRV_LCD_PLANE_ST_LEN    (sizeof(drv_lcd_plane_st))



/* Lcd layer transparency set structure. */     
typedef struct _drv_lcd_layer_transparency_set_st
{
    /* The specific layer. */
    drv_lcd_layer_value layer;
    
    /* Transparency data, This parameter must range from 0x00 to 0xFF. */
    uint8_t      transparency;
    
}drv_lcd_layer_transparency_set_st, *drv_lcd_layer_transparency_set_st_ptr;    
     
#define DRV_LCD_LAYER_TRANSPARENCY_SET_ST_LEN    (sizeof(drv_lcd_layer_transparency_set_st))     


/* Lcd pixel draw structure. */     
typedef struct _drv_lcd_pixel_draw_st
{
    /* The specific layer. */
    drv_lcd_layer_value layer;
    
    /* Coordinate for pixel. */
    uint16_t                x;
    uint16_t                y;
    
    /* The specific color. */
    uint16_t            color;
    
}drv_lcd_pixel_draw_st, *drv_lcd_pixel_draw_st_ptr;    
     
#define DRV_LCD_PIXEL_DRAW_ST_LEN    (sizeof(drv_lcd_pixel_draw_st))     


/* Lcd horizental/vertical line draw structure. */
typedef struct _drv_lcd_hv_line_draw_st
{
    /* The specific layer. */
    drv_lcd_layer_value       layer;
    
    /* coordinate for started pixel. */
    uint16_t                      x;
    uint16_t                      y;
    
    /* line length and width and line direction. */
    uint16_t                 length;
    uint16_t                  width;
    drv_lcd_line_dir_value line_dir;
    
    /* The specific color. */
    uint16_t                  color;
    
}drv_lcd_hv_line_draw_st, *drv_lcd_hv_line_draw_st_ptr;

#define DRV_LCD_HV_LINE_DRAW_ST_LEN    (sizeof(drv_lcd_hv_line_draw_st))


/* Lcd universal line draw structure. */
typedef struct _drv_lcd_universal_line_draw_st
{
    /* The specific layer. */
    drv_lcd_layer_value layer;
    
    /* Coordinate for point 1 and point 2. */
    uint16_t               x1;
    uint16_t               y1;
    uint16_t               x2;
    uint16_t               y2;
    
    /* The specific color. */
    uint16_t            color;
    
}drv_lcd_universal_line_draw_st, *drv_lcd_universal_line_draw_st_ptr;

#define DRV_LCD_UNIVERSAL_LINE_DRAW_ST_LEN    (sizeof(drv_lcd_universal_line_draw_st))


/* Lcd poly line draw structure. */
typedef struct _drv_lcd_poly_line_draw_st
{
    /* The specific layer. */
    drv_lcd_layer_value      layer;
    
    /* Point array start address and count. */
    drv_lcd_point_st_ptr point_ptr;
    uint16_t           point_count;
    
    /* Poly line type. */
    enum _poly_line_type
    {
        LCD_POLY_LINE_CLOSED,  /* Closed line. */
        LCD_POLY_LINE_OPEN     /* Open line. */ 
        
    }poly_line_type;
    
    /* The specific color. */
    uint16_t                 color;

}drv_lcd_poly_line_draw_st, *drv_lcd_poly_line_draw_st_ptr;

#define DRV_LCD_POLY_LINE_DRAW_ST_LEN    (sizeof(drv_lcd_poly_line_draw_st))


/* Lcd rectangle draw structure. */
typedef struct _drv_lcd_rectangle_draw_st
{
    /* The specific layer. */
    drv_lcd_layer_value  layer;

    /* (x,y)position and width height. */
    uint16_t                 x;
    uint16_t                 y;
    uint16_t             width;
    uint16_t            height;
       
    /* The specific color. */
    uint16_t             color;

}drv_lcd_rectangle_draw_st, *drv_lcd_rectangle_draw_st_ptr;

#define DRV_LCD_RECTANGLE_DRAW_ST_LEN    (sizeof(drv_lcd_rectangle_draw_st))








/* Lcd poly line draw structure. */
typedef struct _drv_lcd_circle_draw_st
{
    /* The specific layer. */
    drv_lcd_layer_value layer;
    
    /* Coordinate for centre of circle and radius. */
    int16_t                 x;
    int16_t                 y;
    int16_t                 r;
    
    /* Circle type. */
    enum _circle_type
    {
        LCD_CIRCLE_FILLED,  /* filled circle. */
        LCD_CIRCLE_EMPTY    /* empty circle. */ 
        
    }circle_type;
    
    /* The specific color. */
    uint16_t            color;

}drv_lcd_circle_draw_st, *drv_lcd_circle_draw_st_ptr;

#define DRV_LCD_CIRCLE_DRAW_ST_LEN    (sizeof(drv_circle_line_draw_st))


/* Lcd draw a char structure. */
typedef struct _drv_lcd_char_draw_st
{
    /* The specific layer. */
    drv_lcd_layer_value  layer;

    /* Coordinate for start drawing a char. */
    uint16_t              line;
    uint16_t            column;

    /* charecter's back color and text color. */
    uint16_t        back_color;
    uint16_t        text_color;
    
    /* Front attribution pointer. */
    const drv_lcd_plane_st *font_ptr;
    
    /* Ascii code. */
    uint8_t               ascii_char;
    
}drv_lcd_char_draw_st, *drv_lcd_char_draw_st_ptr;

#define DRV_LCD_CHAR_DRAW_ST_LEN    (drv_lcd_char_draw_st)


/* Lcd display string line structure. */
typedef struct _drv_lcd_string_display_st
{
    /* The specific layer. */
    drv_lcd_layer_value  layer;

    /* Coordinate for start displaying string. */
    uint16_t              line;
    uint16_t            column;

    /* String's back color and text color. */
    uint16_t        back_color;
    uint16_t        text_color;
    
    /* Front attribution pointer. */
    const drv_lcd_plane_st *font_ptr;
    
    /* String's start address. */
    char                 *string_ptr;
    
}drv_lcd_string_display_st, *drv_lcd_string_display_st_ptr;

#define DRV_LCD_STRING_DISPLAY_ST_LEN    (drv_lcd_srting_display_st)


/* Lcd draw a block structure. */
typedef struct _drv_lcd_block_draw_st
{
    /* The specific layer. */
    drv_lcd_layer_value         layer;

    /* Coordinate for start drawing a block. */
    uint16_t                     line;
    uint16_t                   column;
    
    /* Front attribution pointer. */
    const drv_lcd_plane_st *plane_ptr;
    
}drv_lcd_block_draw_st, *drv_lcd_block_draw_st_ptr;

#define DRV_LCD_BLOCK_DRAW_ST_LEN    (drv_lcd_block_draw_st)










/* Lcd colorkeying set structure. */
typedef struct _drv_lcd_colorkeying_set_st
{
    /* The specific layer that draw a pixel. */
    drv_lcd_layer_value layer;
    
    /* The specific color. */
    uint16_t            color;
    
}drv_lcd_colorkeing_set_st, *drv_lcd_colorkeying_set_st_ptr;

#define DRV_LCD_COLORKEYING_SET_ST    (sizeof(drv_lcd_colorkeying_set_st))


/* Lcd colorkeying reset structure. */
typedef struct _drv_lcd_colorkeying_reset_st
{
    /* The specific layer that draw a pixel. */
    drv_lcd_layer_value layer;
    
}drv_lcd_colorkeing_reset_st, *drv_lcd_colorkeying_reset_st_ptr;

#define DRV_LCD_COLORKEYING_RESET_ST    (sizeof(drv_lcd_colorkeying_reset_st))








/* Lcd layer clear structure. */     
typedef struct _drv_lcd_layer_clear_st
{
    /* The specific layer that to be cleared. */
    drv_lcd_layer_value layer;
    
    /* Color to be filled into screen. */
    uint16_t            color;
    
}drv_lcd_layer_clear_st, *drv_lcd_layer_clear_st_ptr;    
     
#define DRV_LCD_LAYER_CLEAR_ST_LEN    (sizeof(drv_lcd_layer_clear_st))     




     
     
     
     
     
     
     
     
     
     
     
     
     
     
     
     

/** @defgroup Exported_Constants
  * @{
  */ 




/** 
  * @brief  LCD Lines depending on the chosen fonts.  
  */
#define LCD_LINE_0               LINE(0)
#define LCD_LINE_1               LINE(1)
#define LCD_LINE_2               LINE(2)
#define LCD_LINE_3               LINE(3)
#define LCD_LINE_4               LINE(4)
#define LCD_LINE_5               LINE(5)
#define LCD_LINE_6               LINE(6)
#define LCD_LINE_7               LINE(7)
#define LCD_LINE_8               LINE(8)
#define LCD_LINE_9               LINE(9)
#define LCD_LINE_10              LINE(10)
#define LCD_LINE_11              LINE(11)
#define LCD_LINE_12              LINE(12)
#define LCD_LINE_13              LINE(13)
#define LCD_LINE_14              LINE(14)
#define LCD_LINE_15              LINE(15)
#define LCD_LINE_16              LINE(16)
#define LCD_LINE_17              LINE(17)
#define LCD_LINE_18              LINE(18)
#define LCD_LINE_19              LINE(19)
#define LCD_LINE_20              LINE(20)
#define LCD_LINE_21              LINE(21)
#define LCD_LINE_22              LINE(22)
#define LCD_LINE_23              LINE(23)
#define LCD_LINE_24              LINE(24)
#define LCD_LINE_25              LINE(25)
#define LCD_LINE_26              LINE(26)
#define LCD_LINE_27              LINE(27)
#define LCD_LINE_28              LINE(28)
#define LCD_LINE_29              LINE(29)
#define LCD_LINE_30              LINE(30)
#define LCD_LINE_31              LINE(31)
#define LCD_LINE_32              LINE(32)
#define LCD_LINE_33              LINE(33)
#define LCD_LINE_34              LINE(34)
#define LCD_LINE_35              LINE(35)
#define LCD_LINE_36              LINE(36)
#define LCD_LINE_37              LINE(37)
#define LCD_LINE_38              LINE(38)
#define LCD_LINE_39              LINE(39)

/* LCD default font. */ 
#define LCD_DEFAULT_FONT         Font16x24









/* Main object for lcd driver. */
extern drv_lcd_st LcdDriver;

#define DRV_LCD_PTR    ((drv_lcd_st_ptr)&LcdDriver)









#if 0

/** @defgroup Exported_Functions
  * @{
  */ 

void     LCD_ClearLine(uint16_t Line);
void     LCD_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width);
void     LCD_WindowModeDisable(void);
void     LCD_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width);
void     LCD_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void     LCD_DrawEllipse(int Xpos, int Ypos, int Radius, int Radius2);
void     LCD_DrawFullEllipse(int Xpos, int Ypos, int Radius, int Radius2);
void     LCD_DrawFullRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void     LCD_DrawFullCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius);
void     LCD_FillPolyLine(pPoint Points, uint16_t PointCount);
void     LCD_Triangle(pPoint Points, uint16_t PointCount);
void     LCD_FillTriangle(uint16_t x1, uint16_t x2, uint16_t x3, uint16_t y1, uint16_t y2, uint16_t y3);

/**
  * @}
  */
 #endif 


#ifdef __cplusplus
}
#endif

#endif /* _DRV_LCD_H_ */
