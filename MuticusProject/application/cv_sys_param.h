/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_sys_param.h
 @brief  : head file of vanet's param 
 @author : gexueyuan
 @history:
           2015-2-4    gexueyuan    Created file
           ...
******************************************************************************/

/*****************************************************************************
 * declaration of variables and functions                                    *
*****************************************************************************/
typedef struct _flash_param{
    char *key;
    char *value;
}flash_param, *flash_param_t;


typedef struct {

	cfg_param_t cus_param;
	cfg_param_t hw_param;
	cfg_param_t mt_param;
	cfg_param_t ct_param;

}vanet_param;


#ifdef USE_BOOT
#define PARAM_FLAG_ADDR     ((uint32_t)0x800C000)
#define PARAM_SECTOR         FLASH_Sector_3
#else
#define PARAM_SECTOR         FLASH_Sector_11
#define PARAM_FLAG_ADDR     ((uint32_t)0x80E0000)
#endif
#define PARAM_OFFSET        0x200


#define PARAM_MODE_ADDR     PARAM_FLAG_ADDR+0x10

#define PARAM_ADDR    		PARAM_MODE_ADDR+0x10



#define PARAM_ADDR_CUSTOM     (PARAM_ADDR)

#define PARAM_ADDR_HIGHWAY    (PARAM_ADDR+PARAM_OFFSET)

#define PARAM_ADDR_MOUNTAIN   (PARAM_ADDR+2*PARAM_OFFSET)

#define PARAM_ADDR_CITY       (PARAM_ADDR+3*PARAM_OFFSET)


#define name_to_str(name)  (#name)

uint16_t  mode_get(void);

const char* mode_string[]={

        "custom",
        "highway",
        "mountain",
        "city"

};


const uint16_t mode_array[] = {

        CUSTOM_MODE,
        HIGHWAY_MODE,
        MOUNTAIN_MODE,
        CITY_MODE
};

static const flash_param param_data[] = {

    {"ID","0"},
    {"vam.bsm_hops","0"},
    {"vam.bsm_boardcast_mode","0"},
    {"vam.bsm_boardcast_saftyfactor","1"},
    {"vam.bsm_pause_mode","0"},




};
