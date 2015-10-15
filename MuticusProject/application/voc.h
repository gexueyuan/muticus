#ifndef __VOC_H__
#define __VOC_H__ 


typedef struct _voc_config {
    uint8_t fg_volume;
    uint8_t bg_volume;
    uint8_t speed;
} voc_config_t;


typedef void (*voc_handler)(void);

typedef struct _voc_session {
    uint32_t encode_type; /* 0 - pcm, 1 - adpcm */
    uint32_t sample_rate;
    uint8_t *src_data;
    uint32_t src_length;
    uint32_t played_length;
    voc_handler complete_callback;
} voc_session_t;


#define BUFFER_COUNT  2
#define BUFFER_SIZE   0x2000

#define VOC_ENCODE_PCM       0
#define VOC_ENCODE_ADPCM     1

#define VOC_STATUS_MASK      0xFFFF
#define VOC_STATUS_PLAYING   0x0001
#define VOC_STATUS_DEV_BUSY  0x0002
#define VOC_STATUS_STOP      0x0004


#define VOC_STATUS_SET(s) do {\
    osal_enter_critical(); \
    voc_status |= s; \
    osal_leave_critical(); \
    }while(0)                                  

#define VOC_STATUS_CLR(s) do {\
    osal_enter_critical(); \
    voc_status &= ~s; \
    osal_leave_critical(); \
    }while(0)                                  

#define VOC_STATUS_TST(s) (voc_status&(s))


int voc_play(uint32_t encode_type, uint8_t *data, uint32_t length, voc_handler complete);
void voc_stop(uint32_t b_wait);
void syn6288_stop(void);

#endif //__VOC_H__

