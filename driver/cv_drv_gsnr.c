/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : cv_drv_gsnr.c
 @brief  : gsensor driver and ebd detected implement
 @author : wanglei
 @history:
           2014-8-11    wanglei       Created file
           ...
******************************************************************************/
#include "cv_osal.h"
#define OSAL_MODULE_DEBUG
#define OSAL_MODULE_DEBUG_LEVEL OSAL_DEBUG_INFO
#define MODULE_NAME "gsnr"
#include "cv_osal_dbg.h"

#include "components.h"
#include "cv_vam.h"
#include "cv_cms_def.h"
#include <math.h>
#include "gsensor.h"
#include "gps.h"

#include "bma250e.h"

static gsnr_log_level_t gsnr_log_lvl = GSNR_NOTICE;

GSENSOR_INFO g_info, Acce_Sum, Acce_V, gSensor_Static, Acce_Ahead, Acce_K;
uint8_t drivint_step = 0;	//Ϊ1ʱ��ʾ�Ѽ������̬ʱxyz����ļ��ٶ�, 2��ȷ����ͷ���� 

int32_t s_cnt = 0;
int32_t rd_cnt = 0 ;

float   STATIC_ACC_THR          =   0.4;  //obd: 0.2    ����shell����param_set(20, 4)����
float	SHARP_RIGHT_THRESOLD    =	5.5;
uint8_t	SHARP_RIGHT_CNT			= 	6;
float	SHARP_LEFT_THRESOLD		=	-5.5;
uint8_t	SHARP_LEFT_CNT			= 	6;
float	SHARP_SLOWDOWN_THRESOLD	=   -3.0; //obd: -5.5  ����shell����param_set(21, -30)����
uint8_t	SHARP_SLOWDOWN_CNT		=	2;    //obd: 3     ����shell����param_set(22, 2)����
float	SHARP_SPEEDUP_THRESOLD	=	1.8;
uint8_t	SHARP_SPEEDUP_CNT		=	6;

uint8_t	AHEAD_CNT				=	20;  //obd: 30
uint8_t	STATIC_GSENSOR_CNT		=	20;  //obd: 30
float	AHEAD_SPEED_THRESOD		=	15.0;
float   VEHICLE_ACCLE_VALE      =   0.1 ;
float   VEHICLE_ANGLE	        =   5.0 ;


extern int8_t  gsnr_param_set(uint8_t gsnr_cal_step, int32_t AcceV_x, int32_t AcceV_y, int32_t AcceV_z,
                                 int32_t AcceAhead_x, int32_t AcceAhead_y, int32_t AcceAhead_z);


static void printAcc(gsnr_log_level_t level, char *des, float x, float y, float z)
{
    if(level <= gsnr_log_lvl)
    {
        char buf[3][20] = {{0}, {0}, {0}};
        sprintf(buf[0], "%.6f", x); 
        sprintf(buf[1], "%.6f", y); 
        sprintf(buf[2], "%.6f", z); 

        osal_printf("%s(%s, %s, %s)\r\n",\
                           des, buf[0], buf[1], buf[2]);
    }
}


//��Gesensorȡ��3����ٶ���ֵ�������д���
void GsensorReadAcc(float* pfData)
{
    gsnr_get_acc(pfData);
    printAcc(GSNR_DEBUG, "raw_xyz", pfData[0], pfData[1], pfData[2]);    

    g_info.x = pfData[0];
    g_info.y = pfData[1];
    g_info.z = pfData[2];

}

/*************************************************
  Function:       MaxA
  Description:    ȡx��y��z������ٶȵ����ֵ
  Input:          x��y��z������ٶ�
  Output:         None
  Return:         x��y��z������ٶȵ����ֵ
  Others:         None
*************************************************/
float MaxA(float x, float y, float z)
{
	float temp;
	(x>y)?(temp = x):(temp = y);
	(temp>z)?temp:(temp = z);

	return temp;
}

/*************************************************
  Function:       VectorSum
  Description:    ����������ģ
  Input:          ������x,y,z����ֵ
  Output:         None
  Return:         ������ģ
  Others:         None
*************************************************/
float VectorSum(GSENSOR_INFO gsensor_date)
{
	float temp_a = 0.0;

	temp_a = sqrt((gsensor_date.x * gsensor_date.x) + (gsensor_date.y * gsensor_date.y) + (gsensor_date.z * gsensor_date.z));

	return temp_a;
}

/*************************************************
  Function:       VectorDotMul
  Description:    ������ˣ�������a������b=xa*xb+ya*yb+za*zb
  Input:          �������������x,y,z����ֵ
  Output:         None
  Return:         ������˵Ľ��
  Others:         None
*************************************************/
float VectorDotMul(GSENSOR_INFO gsensor_dateA, GSENSOR_INFO gsensor_dateB)
{
	float temp_a = 0.0;

	temp_a = (gsensor_dateA.x * gsensor_dateB.x) + (gsensor_dateA.y * gsensor_dateB.y) + (gsensor_dateA.z * gsensor_dateB.z);

	return temp_a;
}

/*************************************************
  Function:       VectorCrossMul
  Description:    ������ˣ�������a X ����b = (ya*zb-yb*za,za*xb-zb*xa,xa*yb-xb*ya)
  Input:          �������������x,y,z����ֵ
  Output:         None
  Return:         ������˵Ľ��
  Others:         None
*************************************************/
GSENSOR_INFO VectorCrossMul(GSENSOR_INFO gsensor_dateA, GSENSOR_INFO gsensor_dateB)
{
	GSENSOR_INFO temp_vector;

	temp_vector.x = gsensor_dateA.y * gsensor_dateB.z - gsensor_dateB.y * gsensor_dateA.z;
	temp_vector.y = gsensor_dateA.z * gsensor_dateB.x - gsensor_dateB.z * gsensor_dateA.x;
	temp_vector.z = gsensor_dateA.x * gsensor_dateB.y - gsensor_dateB.x * gsensor_dateA.y;

	return temp_vector;
}

/*************************************************
  Function:       CalAng
  Description:    ����x��y��z������ˮƽ��ļн�
  Input:          x��y��z������ٶ�
  Output:         None
  Return:         None
  Others:         None
*************************************************/
float CalAng(GSENSOR_INFO dataA, GSENSOR_INFO dataB)
{
	float ang = 0.0;
	ang = acos(VectorDotMul(dataA, dataB) / (VectorSum(dataA) * VectorSum(dataB))); 

	return ang;	
}

/*************************************************
  Function:       GetStaticVal
  Description:    ���㾲̬ʱxyz����ļ��ٶ�ֵ
  Input:          x��y��z������ٶ�
  Output:         None
  Return:         None
  Others:         ͣ��̬ʱ��ȡ30��xyz��ļ��ٶ���ƽ��
*************************************************/
float last_static_x = 0.0 ;
float last_static_y = 0.0 ;
float last_static_z = 0.0 ;

int32_t GetStaticVal(GSENSOR_INFO gsensor_dat)
{
    if(G_Action.carRun == 0)		   //ͣ��̬(����������ͣ��̬���ж�����)�����㾲̬ʱxyz����ļ��ٶ�ֵ
	{
	    gSensor_Static.x += gsensor_dat.x;
		gSensor_Static.y += gsensor_dat.y;
		gSensor_Static.z += gsensor_dat.z;
		if((fabs(last_static_x-gsensor_dat.x)>STATIC_ACC_THR) || 
            (fabs(last_static_y-gsensor_dat.y)>STATIC_ACC_THR) ||  
            (fabs(last_static_z-gsensor_dat.z)>STATIC_ACC_THR))
		{
      		printAcc(GSNR_NOTICE, "ͣ��̬���ٶȲ�������xyz\r\n", fabs(last_static_x-gsensor_dat.x), 
                fabs(last_static_y-gsensor_dat.y), 
                fabs(last_static_z-gsensor_dat.z));
			last_static_x = gsensor_dat.x ;
			last_static_y = gsensor_dat.y ;
			last_static_z = gsensor_dat.z ;
			gSensor_Static.x = 0;
			gSensor_Static.y = 0;
			gSensor_Static.z = 0;
			s_cnt = 0 ;
			return -1 ;
		}
		GSNR_LOG(GSNR_INFO, "ͣ��̬���㾲ֹ�������ٶȷ�������s_cnt=%d", s_cnt);
		printAcc(GSNR_INFO, "xyz\r\n",gSensor_Static.x, gSensor_Static.y, gSensor_Static.z);
		s_cnt++;
		last_static_x = gsensor_dat.x ;
		last_static_y = gsensor_dat.y ;
		last_static_z = gsensor_dat.z ;
		
	}
	else		   //�г�̬
	{
		gSensor_Static.x = 0;
		gSensor_Static.y = 0;
		gSensor_Static.z = 0;
		s_cnt = 0;
	 	return -1; 
	}


	if(s_cnt == STATIC_GSENSOR_CNT)			//��̬ʱ��ȡ30��������ٶ�ֵ
	{
		gSensor_Static.x /= STATIC_GSENSOR_CNT;
		gSensor_Static.y /= STATIC_GSENSOR_CNT;
		gSensor_Static.z /= STATIC_GSENSOR_CNT;

		/***************��ֱ�����ϵĵ�λ����*******************/
		gSensor_Static.sum = VectorSum(gSensor_Static);
		Acce_V.x = gSensor_Static.x / gSensor_Static.sum;
		Acce_V.y = gSensor_Static.y / gSensor_Static.sum;
		Acce_V.z = gSensor_Static.z / gSensor_Static.sum;
		/******************************************************/

		s_cnt = 0;
		printAcc(GSNR_NOTICE, "��ȷ���������ٶȷ���: gSensor_Static", gSensor_Static.x, gSensor_Static.y, gSensor_Static.z);
		printAcc(GSNR_NOTICE, "��ֱ�����ϵĵ�λ����: Acce_V", Acce_V.x, Acce_V.y, Acce_V.z);

		return 1 ;
	}

	return 0 ;
}

/*************************************************
  Function:       RecDirection
  Description:    ȷ����ͷ����
  Input:          x��y��z������ٶ�
  Output:         None
  Return:         None
  Others:         ͣ��̬ʱ��ȡ30��xyz��ļ��ٶ���ƽ��
*************************************************/
int32_t RecDirection(GSENSOR_INFO gsensor_dat)
{
	GSENSOR_INFO temp_acce_v;	 //�ϳ������ڴ�ֱ�����ϵķ���
 
	if((G_Action.speed > AHEAD_SPEED_THRESOD) && (rd_cnt<AHEAD_CNT) && 
		(G_Action.vehicle_accel_value > VEHICLE_ACCLE_VALE) && 
		(G_Action.diff_angle < VEHICLE_ANGLE) &&
		(G_Action.is_locate == __TRUE))
	{
		Acce_Sum.x += gsensor_dat.x;
		Acce_Sum.y += gsensor_dat.y;
		Acce_Sum.z += gsensor_dat.z;
        GSNR_LOG(GSNR_DEBUG, "cnt[%d]car_speed[%d] acce[%d], anle[%d] loc[%d]\r\n", 
                rd_cnt, G_Action.speed, G_Action.vehicle_accel_value, G_Action.diff_angle, G_Action.is_locate);
		printAcc(GSNR_DEBUG, "RecDirection", Acce_Sum.x,Acce_Sum.y,Acce_Sum.z);
		rd_cnt++;
	}
	else if (rd_cnt == AHEAD_CNT)
	{
		Acce_Sum.x /= AHEAD_CNT;
		Acce_Sum.y /= AHEAD_CNT;
		Acce_Sum.z /= AHEAD_CNT;
        
		printAcc(GSNR_INFO, "Acce_Sum", Acce_Sum.x, Acce_Sum.y, Acce_Sum.z);
		Acce_Sum.sum = VectorSum(Acce_Sum);	   //��ֱ�����복ͷ����ĺϳ���
	
		/*************��ֱ���������******************/
		temp_acce_v.sum = VectorDotMul(Acce_Sum, Acce_V);	  //����a������b=xa*xb+ya*yb+za*zb	 �ϳ��� * ��ֱ����ĵ�λ����
		temp_acce_v.x = (temp_acce_v.sum) * (Acce_V.x);
		temp_acce_v.y = (temp_acce_v.sum) * (Acce_V.y);
		temp_acce_v.z = (temp_acce_v.sum) * (Acce_V.z);
		/**********************************************/

		/************��ͷ���������*******************/
		Acce_Ahead.x = Acce_Sum.x - temp_acce_v.x;			 //�ϳ��� - ��ֱ����ķ��� 
		Acce_Ahead.y = Acce_Sum.y - temp_acce_v.y;
		Acce_Ahead.z = Acce_Sum.z - temp_acce_v.z;
		/*********************************************/

		/************��ͷ����ĵ�λ����*******************/
		Acce_Ahead.sum = VectorSum(Acce_Ahead);
		Acce_Ahead.x = Acce_Ahead.x / Acce_Ahead.sum;
		Acce_Ahead.y = Acce_Ahead.y / Acce_Ahead.sum;
		Acce_Ahead.z = Acce_Ahead.z / Acce_Ahead.sum;
	    /*********************************************/

		/************��������ĵ�λ����*******************/
		Acce_K = VectorCrossMul(Acce_Ahead, Acce_V);
		/**********************************************/

		rd_cnt = 0;
		Acce_Sum.x = 0;
		Acce_Sum.y = 0;
		Acce_Sum.z = 0;
		Acce_Sum.sum = 0;
		printAcc(GSNR_NOTICE, "�ó���ͷ����ĵ�λ����: Acce_Ahead", Acce_Ahead.x, Acce_Ahead.y, Acce_Ahead.z);
		printAcc(GSNR_NOTICE, "��������ĵ�λ����: Acce_K", Acce_K.x, Acce_K.y, Acce_K.z);

		return 1 ;
	}
	else 
	{
        GSNR_LOG(GSNR_INFO, "������Ҫ��,������: car_speed[%d] acce[%d], angle[%d] loc[%d]\r\n", 
                 G_Action.speed, G_Action.vehicle_accel_value, G_Action.diff_angle, G_Action.is_locate);
		return -1;
	}
	return 0;
}

/*************************************************
  Function:       AcceDetect
  Description:    ���Ӽ��ټ��
  Input:          x��y��z������ٶ�
  Output:         None
  Return:         None
  Others:         xyz����ٶȵ��������趨����ֵ���Ƚϣ��ж��Ƿ������Ӽ���
*************************************************/
void AcceDetect(float acce_ahead, float acce_k, float acce_k_x)
{
	static int32_t cnt = 0 ;
	sys_envar_t *p_sys = &p_cms_envar->sys;
	static uint8_t key_press = 0;

	if(acce_k > SHARP_RIGHT_THRESOLD)	
	{
		printAcc(GSNR_INFO, "��תxyz", acce_ahead, acce_k, acce_k_x);
		cnt++;
		if(cnt >= SHARP_RIGHT_CNT)		  //��ת
		{
			GSNR_LOG(GSNR_INFO, "��������ת\r\n\n");
			cnt = 0 ;
		}
	}
	else if(acce_k < SHARP_LEFT_THRESOLD)
	{
		printAcc(GSNR_INFO, "��תxyz",acce_ahead, acce_k, acce_k_x);
		cnt++;
		if(cnt >= SHARP_LEFT_CNT)		  //��ת
		{	
			GSNR_LOG(GSNR_INFO, "��������ת\r\n\n");
			cnt = 0 ;
		}
	}
	else if(acce_ahead >= SHARP_SPEEDUP_THRESOLD)
	{
		printAcc(GSNR_INFO, "����xyz",acce_ahead, acce_k, acce_k_x);
		cnt++;
		if(cnt >= SHARP_SPEEDUP_CNT)
		{
			GSNR_LOG(GSNR_INFO, "����������\r\n\n");
			cnt = 0 ;
		}
	}
	else if(acce_ahead < SHARP_SLOWDOWN_THRESOLD)
	{
		printAcc(GSNR_NOTICE, "����xyz",acce_ahead, acce_k, acce_k_x);
		cnt++;
		if(cnt >= SHARP_SLOWDOWN_CNT)
		{
            GSNR_LOG(GSNR_WARNING, "����������\r\n\n");
            /* ֪ͨvsaģ�鴦�� */
            vam_gsnr_ebd_detected(1);
            cnt = 0 ;
		}
	}
	else
	{
		cnt = 0 ;
	}
    
    if(acce_k_x <= -5)
    {
        if(0 == key_press)
        {
            printAcc(GSNR_INFO, "��תxyz",acce_ahead, acce_k, acce_k_x);
            sys_add_event_queue(p_sys,SYS_MSG_KEY_PRESSED,0,2,NULL);		
            key_press = 1;
        }
    }
    else if(acce_k_x > 3)
    {
        if(key_press == 1)
        {
            printAcc(GSNR_INFO, "��תxyz",acce_ahead, acce_k, acce_k_x);
            sys_add_event_queue(p_sys,SYS_MSG_KEY_PRESSED,0,2,NULL);			
            key_press = 0;
        }
    }

}
void AcceHandle(GSENSOR_INFO gsensor_data)
{
	GSENSOR_INFO temp_acce_v;	     //�ϳ������ڴ�ֱ�����ϵķ���
	GSENSOR_INFO temp_acce_ahead;	 //�ϳ������ڳ�ͷ�����ϵķ���
	GSENSOR_INFO temp_acce_k;	     //�ϳ������ڳ������ҷ���ķ���
	
	temp_acce_v.sum = VectorDotMul(gsensor_data, Acce_V);	          //����a������b=xa*xb+ya*yb+za*zb	//��ֱ���������
	temp_acce_ahead.sum = VectorDotMul(gsensor_data, Acce_Ahead);	  //����a������b=xa*xb+ya*yb+za*zb	//��ͷ��������
	temp_acce_k.sum = VectorDotMul(gsensor_data, Acce_K);			  //����a������b=xa*xb+ya*yb+za*zb	//���ҷ�������

	printAcc(GSNR_INFO, "xyz", temp_acce_ahead.sum, temp_acce_k.sum, temp_acce_v.sum);

    lip_update_local_acc(temp_acce_ahead.sum, temp_acce_k.sum, temp_acce_v.sum);
    AcceDetect(temp_acce_ahead.sum, temp_acce_k.sum, temp_acce_v.sum);   
}


void GsensorDataSave(uint8_t stepflag, GSENSOR_INFO acceV, GSENSOR_INFO acceAhead)
{
	int32_t temp_x = 0, temp_y = 0, temp_z = 0;
	int32_t temp1_x = 0, temp1_y = 0, temp1_z = 0;


	temp_x = (int32_t)(acceV.x * 10000);
	temp_y = (int32_t)(acceV.y * 10000);
	temp_z = (int32_t)(acceV.z * 10000);

	temp1_x = (int32_t)(acceAhead.x * 10000);
	temp1_y = (int32_t)(acceAhead.y * 10000);
	temp1_z = (int32_t)(acceAhead.z * 10000);

	gsnr_param_set(stepflag, temp_x, temp_y, temp_z, temp1_x, temp1_y, temp1_z);
}

uint8_t GsensorDataRead(gsnr_config_t *p_gsnr)
{
	uint8_t flag = 0;
   
    flag = p_gsnr->gsnr_cal_step;

    if(flag >= 1)
    {
        Acce_V.x = p_gsnr->AcceV_x / 10000.0f;
        Acce_V.y = p_gsnr->AcceV_y / 10000.0f;
    	Acce_V.z = p_gsnr->AcceV_z / 10000.0f;
		printAcc(GSNR_NOTICE, "��ȡ��ֱ����ĵ�λ����: Acce_V", Acce_V.x, Acce_V.y, Acce_V.z);
    }

    if(flag == 2)
    {
    	Acce_Ahead.x = p_gsnr->AcceAhead_x / 10000.0f;
    	Acce_Ahead.y = p_gsnr->AcceAhead_y / 10000.0f;
    	Acce_Ahead.z = p_gsnr->AcceAhead_z / 10000.0f;

        /************��������ĵ�λ����**************/
        Acce_K = VectorCrossMul(Acce_Ahead, Acce_V);
        /**********************************************/

		printAcc(GSNR_NOTICE, "��ȡ��ͷ����ĵ�λ����: Acce_Ahead", Acce_Ahead.x, Acce_Ahead.y, Acce_Ahead.z);
		printAcc(GSNR_NOTICE, "��������ĵ�λ����: Acce_K", Acce_K.x, Acce_K.y, Acce_K.z);
    }
   
	return flag;
}

static void gsnr_thread_entry(void *parameter)
{
	float   pfData[3]={0};
    GsensorReadAcc(pfData);
	while(1) {
        GsensorReadAcc(pfData);
		if(drivint_step == 0)
		{
	    	if(GetStaticVal(g_info) == 1)
	    	{
        		drivint_step = 1;	  //��ȷ���������ٶȷ���
                GsensorDataSave(drivint_step, Acce_V, Acce_Ahead);
	    	}
            
		}
		else if(drivint_step == 1)
		{
    		if(RecDirection(g_info) == 1)
    		{
                drivint_step = 2;	  //��ȷ����ͷ����
                GsensorDataSave(drivint_step, Acce_V, Acce_Ahead);
    		}
		}
		else if(drivint_step == 2)
		{
			AcceHandle(g_info);
		}
        osal_delay(GSNR_POLL_TIME_INTERVAL);
    } 
}

void gsnr_init()
{
    osal_task_t *gsnr_thread;
    /* load gsnr param from flash */
	gsnr_config_t *p_gsnr_param = NULL;		
    p_gsnr_param = &p_cms_param->gsnr;

    gsnr_drv_init();
        
    STATIC_ACC_THR = p_gsnr_param->gsnr_cal_thr/10.0f;
    SHARP_SLOWDOWN_THRESOLD = p_gsnr_param->gsnr_ebd_thr/10.0f;
    SHARP_SLOWDOWN_CNT = p_gsnr_param->gsnr_ebd_cnt;

    if(p_gsnr_param->gsnr_cal_step == 3)
    {
		GSNR_LOG(GSNR_NOTICE, "Use gps data to caculate acceleration. stop gsnr thread.\r\n");
        return;
    }


    drivint_step = GsensorDataRead(p_gsnr_param);
       
    gsnr_thread = osal_task_create("t-gsnr",
                                    gsnr_thread_entry, RT_NULL,
                                    RT_MEMS_THREAD_STACK_SIZE, RT_MEMS_THREAD_PRIORITY);
    osal_assert(gsnr_thread != RT_NULL) 
}


