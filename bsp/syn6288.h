/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : syn6288.h
 @brief  : syn6288 head file
 @author : gexueyuan
 @history:
           2015-9-10    gexueyuan    Created file
           ...
******************************************************************************/



/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
void syn6288_volume(uint8_t vol);

int syn6288_play(char *txt);

void syn6288_stop(void);

void syn6288_pause(void);

void syn6288_continue(void);

void syn6288_set(uint8_t fg_vol,uint8_t bg_vol,uint8_t speed);

uint8_t syn6288_state(void);

uint8_t syn6288_hw_init(void);

