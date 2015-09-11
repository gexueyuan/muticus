/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_osal_dbg.h
 @brief  : This file realize the debug functions
 @author : wangyf
 @history:
           2014-11-14    wangyf    Created file
           ...
******************************************************************************/
#ifndef __CV_OSAL_DBG_H__
#define __CV_OSAL_DBG_H__

/**
 *  Definitions of debug level
 *
 */
#define OSAL_DEBUG_OFF          0
#define OSAL_DEBUG_ERROR        1
#define OSAL_DEBUG_WARN         2
#define OSAL_DEBUG_INFO         3
#define OSAL_DEBUG_TRACE        4
#define OSAL_DEBUG_LOUD         5

/** 
 *  Global debug control
 *
 */
#define OSAL_GLOBAL_DEBUG 1
#define OSAL_GLOBAL_DEBUG_LEVEL OSAL_DEBUG_WARN

/**
 *  Module debug control.
 *  Notice: some macroes can be defined outside of this file
 *
 */
#if (OSAL_GLOBAL_DEBUG) && (defined(OSAL_MODULE_DEBUG))

#if (!defined(OSAL_MODULE_DEBUG_LEVEL))
#define OSAL_MODULE_DEBUG_LEVEL OSAL_GLOBAL_DEBUG_LEVEL
#endif

static int osal_module_debug_level = OSAL_MODULE_DEBUG_LEVEL;
static char *osal_debug_level_str[] = {
    "", "ERR", "W", "I", "T", "L"
};

#define OSAL_MODULE_DBGPRT(module, level, fmt, ...)	                                         \
    do                                                                                       \
    {                                                                                        \
        if ((OSAL_DEBUG_OFF < level) && (level <= osal_module_debug_level))                  \
        {                                                                                    \
            osal_log("(%d.%03d)[%s]%s: "fmt, (osal_get_systemtime()>>10),                   \
               ((osal_get_systemtime()&0x3FF) > 999 ? 999:(osal_get_systemtime() & 0x3FF)), \
               osal_debug_level_str[level], module, ##__VA_ARGS__);                          \
        }                                                                                    \
    } while (0)

#define OSAL_DBGPRT(level, args...) OSAL_MODULE_DBGPRT("", level, ##args)


#define  OSAL_DEBUG_ENTRY_DECLARE(module) \
    extern void module##_set_debug_level(int); \
    const debug_entry_t debug_entry_##module = {#module, module##_set_debug_level}; \


#define  OSAL_DEBUG_ENTRY_DEFINE(module) \
    void module##_set_debug_level(int level) \
    { \
        osal_module_debug_level = level; \
    }

#else /* (OSAL_GLOBAL_DEBUG) && (defined OSAL_MODULE_DEBUG) */
#define OSAL_DBGPRT(level, fmt, ...)
#define OSAL_MODULE_DBGPRT(module, level, fmt, ...)
#define  OSAL_DEBUG_ENTRY_DECLARE(module)
#define  OSAL_DEBUG_ENTRY_DEFINE(module)
#endif /* (OSAL_GLOBAL_DEBUG) && (defined OSAL_MODULE_DEBUG) */

typedef struct _debug_entry{
	const char *module;
	void (*handler)(int);
} debug_entry_t;

extern int g_dbg_print_type;

void osal_dbg_dump_data(uint8_t *p, uint32_t len);
int osal_dbg_set_level(char *module, int level);
void osal_log(const char *fmt, ...);

#endif /* __CV_OSAL_DBG_H__ */

