
#include "cv_osal.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include "gps.h"


t_gps_buff __GPSBuff;
gps_data_callback gps_recv_cb = NULL;
uint8_t _GPSBuffer[GPS_BUFF_SIZE];
uint8_t get_gps = 0 ;

osal_mutex_t* p_mutex_gps;


void gps_callback_register(gps_data_callback fp)
{
	gps_recv_cb = fp;
}


/* set host uart1 baudrate  */
void gps_set_host_baudrate(int baud)
{
    rt_device_t dev;
    struct serial_configure config;

    config.baud_rate = baud;
    config.bit_order = BIT_ORDER_LSB;
    config.data_bits = DATA_BITS_8;
    config.parity    = PARITY_NONE;
    config.stop_bits = STOP_BITS_1;
    config.invert    = NRZ_NORMAL;     
    
    dev = rt_device_find(RT_GPS_DEVICE_NAME);
    rt_device_control(dev, RT_DEVICE_CTRL_CONFIG, (void *)&config);
}


static void gps_read_data(rt_device_t dev)
{
    uint8_t tmp = 0 ;

    
    while(1)
    {
        if(rt_device_read(dev, 0, &tmp, 1) == 1)
        {

            if (tmp == '$') 
            {
                memset(__GPSBuff.PpBuf[__GPSBuff.Pipe].Buf, 0x0, GPS_BUFF_SIZE);
                 
                __GPSBuff.PpBuf[__GPSBuff.Pipe].Len = 1;
                __GPSBuff.PpBuf[__GPSBuff.Pipe].Buf[0] = '$';
                __GPSBuff.PpBuf[__GPSBuff.Pipe].Flag = 1;	
            }
            else if (tmp == '\n') 
            {
                __GPSBuff.PpBuf[__GPSBuff.Pipe].Buf[__GPSBuff.PpBuf[__GPSBuff.Pipe].Len] = '\n';
                __GPSBuff.PpBuf[__GPSBuff.Pipe].Len++;
                __GPSBuff.PpBuf[__GPSBuff.Pipe].Len %= GPS_BUFF_SIZE;
                __GPSBuff.PpBuf[__GPSBuff.Pipe].Flag = 0;

                        {   
                            if(p_cms_envar->vam.queue_vam ){
                            
                                sys_msg_t *p_msg;
                                p_msg = osal_malloc(sizeof(sys_msg_t));
                                if (p_msg) {
                                    p_msg->id = VAM_MSG_GPSDATA;
                                    p_msg->len = __GPSBuff.PpBuf[__GPSBuff.Pipe].Len;
                                    p_msg->argv = &__GPSBuff.PpBuf[__GPSBuff.Pipe].Buf;
                                    if (OSAL_STATUS_SUCCESS != vam_add_event_queue_2(&p_cms_envar->vam, p_msg)){
                                        osal_free(p_msg);
                                    }
                                }
                            }
                        }

                /* Change the pipe index and get new frame. */
                __GPSBuff.Pipe++;
                __GPSBuff.Pipe %= GPS_PIPE;
                
                get_gps = 1 ;
            }
            else 
            {
                if (__GPSBuff.PpBuf[__GPSBuff.Pipe].Flag == 1) 
                {
                    __GPSBuff.PpBuf[__GPSBuff.Pipe].Buf[__GPSBuff.PpBuf[__GPSBuff.Pipe].Len] = tmp;
                    __GPSBuff.PpBuf[__GPSBuff.Pipe].Len++;
                    __GPSBuff.PpBuf[__GPSBuff.Pipe].Len %= GPS_BUFF_SIZE;
                }
            }
        }
        else
        {
            break ;
        }
    }
}


void rt_gps_thread_entry (void *parameter) 
{
    rt_device_t dev ;
    int time = 0;
    uint8_t cfg_flag = 0;
    
    
    dev = rt_device_find(RT_GPS_DEVICE_NAME);
    rt_device_open(dev, RT_DEVICE_OFLAG_RDWR);

    while(1)
    {
        gps_read_data(dev);
        
        if(!cfg_flag)
        {
            if (get_gps)
            {
                gps_chip_config(5);
                
                /* config host uart baudrate to 115200. */
                gps_set_host_baudrate(BAUD_RATE_115200);

                osal_delay(1);                
                cfg_flag = 1;
                time = 0;
            }
            else
            {
                if (time++ > SECOND_TO_TICK(6))
                {
                    /* 初始设备串口波特率9600而PC的GPS模拟工具115200,
                       time>6s 未能解析出正确GPS NMEA则判断为PC GPS工具输入, 
                       set host uart1 baud to 115200 */
                    gps_set_host_baudrate(BAUD_RATE_115200);
                    cfg_flag = 1;
                }           
            }
        }
        
        osal_delay(1);
	}
}

//*******************************************************************************
// 函数名称    : void gps_init(void)
// 功能描述    : 初始化gps模块，例如屏蔽不需要的信息，天线断开延时
// 输入        : None
// 输出        : None
// 返回        : None
//******************************************************************************/
void gps_init(void)
{
    osal_task_t *tid;

	memset(&__GPSBuff, 0, sizeof(__GPSBuff));
	gps_recv_cb = NULL;


	tid = osal_task_create("t-gps",
                           rt_gps_thread_entry, RT_NULL,
                           RT_GPS_THREAD_STACK_SIZE, RT_GPS_THREAD_PRIORITY);
    osal_assert(tid != RT_NULL)
}

void gps_deinit(void)
{
	memset(&__GPSBuff, 0, sizeof(__GPSBuff));
	gps_recv_cb = NULL;	
}

/* shell cmd for debug */
//FINSH_FUNCTION_EXPORT(gps_cfg_rate, debug: set gps rate);

