/*****************************************************************************
 Copyright(C) Beijing Carsmart Technology Co., Ltd.
 All rights reserved.
 
 @file   : bsp_gnss_MAX7Q.c
 @brief  : This file include the bsp functions for the GNSS module ublox MAX7Q.
 @author : wangxianwen
 @history:
           2015-6-09    wangxianwen    Created file
           ...
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "bsp_gnss_MAX7Q.h"

#include "cv_osal.h"
#include "components.h"
#include "gps.h"




/*****************************************************************************
 * implementation of functions                                               *
*****************************************************************************/
static void ubx_pkt_checknum_calc(uint8_t *buf, int len, uint8_t *cknumA, uint8_t *cknumB)
{
    uint16_t ckA=0, ckB=0;
    int i=0;

    
    for (i=0; i<len; i++)
    {
        ckA = ckA + buf[i];
        ckB = ckB + ckA;
    }
    
    *cknumA = ckA;
    *cknumB = ckB;
}

static void ubx_cfg_msg_std_nmea(ubx_cfg_msg_nmea_id nmea_id, uint8_t enable)
{
    rt_device_t dev; 
    int len = UBX_PKT_HDR_ST_LEN;
    uint8_t buf[20] = { 0 };

 
    ubx_pkt_hdr_st pkt  = { UBX_SYSN_CHAR1, UBX_SYSN_CHAR2, 0x06, 0x01, UBX_CFG_MSG_ST_LEN };
    ubx_cfg_msg_st cfg = { 0 };

       
    memcpy(buf, &pkt, len);


    /* standard NMEA messages. */
    cfg.nmeaClass = 0xF0;
    cfg.nmeaid = nmea_id;
    cfg.portOn[1] = enable;

    memcpy(buf+len, &cfg, pkt.length);
    len += pkt.length;

    ubx_pkt_checknum_calc(buf+2, len-2, &buf[len], &buf[len+1]); 
    len += 2;

    dev = rt_device_find(RT_GPS_DEVICE_NAME);
    rt_device_write(dev, 0, buf, len);    
}


/* UBX-CFG MSG. disable GPGGA/GPGLL/GPGSV/GPVTG msg */
static void ubx_cfg_needed_nmea(void)
{
    ubx_cfg_msg_std_nmea(STD_NMEA_ID_GGA, 0);
    ubx_cfg_msg_std_nmea(STD_NMEA_ID_GLL, 0);
    ubx_cfg_msg_std_nmea(STD_NMEA_ID_GSV, 0);
    ubx_cfg_msg_std_nmea(STD_NMEA_ID_VTG, 0);
}


/* UBX-CFG-PRT: set gps port baudrate to 115200 */
static void ubx_cfg_uart_port(void)
{
   /* set baut rate = 115200 */
    uint8_t cfg_pkt[] = 
    {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0,
        0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x03, 0x00, 0x03, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0xBC, 0x5E
    };
    rt_device_t dev ;

    
    dev = rt_device_find(RT_GPS_DEVICE_NAME);
    rt_device_write(dev, 0, cfg_pkt, sizeof(cfg_pkt));
}

void ubx_cfg_nmea_freq(uint8_t freq)
{
	rt_device_t dev; 
    uint8_t cfg_pkt[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xF4, 0x01, 
                         0x01, 0x00, 0x01, 0x00, 0x0B, 0x77};

    switch (freq)
    {
        case 1:
        {
            cfg_pkt[6] = 0xE8;
            cfg_pkt[7] = 0x03;
            cfg_pkt[12] = 0x01;
            cfg_pkt[13] = 0x39;
            break;
        }
        case 2:
        {
            cfg_pkt[6] = 0xF4;
            cfg_pkt[7] = 0x01;
            cfg_pkt[12] = 0x0B;
            cfg_pkt[13] = 0x77;
            break;
        }
        case 5:  //5Hz
        {
            cfg_pkt[6] = 0xC8;
            cfg_pkt[7] = 0x00;
            cfg_pkt[12] = 0xDE;
            cfg_pkt[13] = 0x6A;
            break;
        }
        default:
        {
            break;
        }
    }
    dev = rt_device_find(RT_GPS_DEVICE_NAME);
    rt_device_write(dev, 0, cfg_pkt, sizeof(cfg_pkt));    
}


void gps_chip_config(int freq)
{
    /* get gps nmea. config needed nmea */
    ubx_cfg_needed_nmea();
    osal_delay(1);	
    
    /* conifg ublox gps rate 5Hz */
    ubx_cfg_nmea_freq(freq);
    osal_delay(1);
    
    /* config gps baud to 115200 */
    ubx_cfg_uart_port();
    osal_delay(1);
}


#if 0

extern struct _GPS_Real_buf
{
  char data[256];                      //����GPS���뻺����
volatile unsigned short rx_pc;    //����ָ��
} GPS_Real_buf; //GPS�������ݻ�����


extern struct _GPS_Information
{
  unsigned char Real_Locate;            //ʵʱ��λ��Чλ
  unsigned char Located;                //��λ��Чλ
  unsigned char Locate_Mode;         //��λģʽ��2D��3D
  char UTC_Time[7];                       //ʱ��
  char UTC_Date[7];                      //����
  char Latitude[10];                       //γ��
  char NS_Indicator;                       //N=������S=�ϰ���
  char Longitude[11];                     //����
  char EW_Indicator;                      //E=������W=����
  double Speed;                            //��������
  double Course;                            //���溽��
  double PDOP;                             //λ�þ���
  double HDOP;                             //ˮƽ����
  double VDOP;                             //��ֱ����
  double MSL_Altitude;                   //MSL���θ߶�
  unsigned char Use_EPH_Sum;       //ʹ�õ���������
  unsigned char User_EPH[12];        //��ǰ�����������Ǳ��
  unsigned short EPH_State[12][4]; //��ǰʹ�õ�12�����ǵı�š����ǡ���λ�ǡ������
} GPS_Information; 



/********************************************************************************************************
**������Ϣ ��void USART2_IRQHandler(void) 
**�������� ��UART2�жϷ�����
**������� ����
**������� ����
********************************************************************************************************/
void USART2_IRQHandler( void ) //����2�жϷ������
{
  unsigned char Recv;
  if ( USART_GetITStatus( USART2, USART_IT_RXNE ) != RESET ) //�����ж�
  {
    Recv = USART_ReceiveData( USART2 );         //��������
    if ( Recv == '$' )
    {
      GPS_Real_buf.rx_pc = 0;
    }
    else
    {
      if ( GPS_Real_buf.rx_pc < sizeof( GPS_Real_buf.data ) - 1 )
      {
        GPS_Real_buf.rx_pc++;
      }
    }
    GPS_Real_buf.data[GPS_Real_buf.rx_pc] = Recv;
    if ( Recv == '\r' )    //����������Ϊ0x0Dʱ��ʼ����GPS����
    {
      if ( Real_Process_Enabled == Valid )
      {
        if ( Calc_GPS_Sum( GPS_Real_buf.data ) == Valid )
        {
          Creat_DH_Index( GPS_Real_buf.data );
          Real_GPS_Command_Process();
        }
      }
    }
  }

  if ( USART_GetFlagStatus( USART2, USART_FLAG_ORE ) == SET ) //����ж�
  {
    USART_ClearFlag( USART2, USART_FLAG_ORE );   //�����λ
    USART_ReceiveData( USART2 );                        //��DR
  }
}


/********************************************************************************************************
**������Ϣ ��void Creat_DH_Index( char* buffer )
**�������� ��Ѱ�����ж��ŵ�λ�ã���������
**������� ������GPS���ݵĻ�����
**������� ������ȫ�ֱ��������еĶ���������ԭGPS�����еĶ��Ž��ᱻ0x00���
********************************************************************************************************/
unsigned char DH_id_sep[32];//ȫ�ֱ������飬��ദ��32����������
void Creat_DH_Index( char* buffer )
{
  unsigned short i, len;
  unsigned char idj;

  memset( DH_id_sep, 0, sizeof( DH_id_sep ) );
  len = strlen( buffer );
  for ( i = 0, idj = 0; i < len; i++ )
  {
    if ( buffer[i] == ',' )
    {
      DH_id_sep[idj] = i;
      idj++;
      buffer[i] = 0x00;
    }
  }
}

/********************************************************************************************************
**������Ϣ ��char* Real_Process_DH( char* buffer, unsigned char num )
**�������� ������GPS���ݵ�N��������ƫ��
**������� ������������Ľ���GPS���ݻ�����
**������� �����ص�N��","֮�����Ϣ����Ҫ*buffer��Ч������������ſ���ִ��
********************************************************************************************************/
char* Real_Process_DH( char* buffer, unsigned char num )
{
  if ( num < 1 )
    return  &buffer[0];
  return  &buffer[ DH_id_sep[num - 1] + 1];
}


/********************************************************************************************************
**������Ϣ ��void Real_GPS_Command_Process( void )
**�������� ���������ò������������ݲ�����GPS���ݽṹ��
**������� ��
**������� ��
********************************************************************************************************/
void Real_GPS_Command_Process( void )
{
  char* temp;
  unsigned char i, j, zhen;

  if ( strstr( GPS_Real_buf.data, "GPGGA" ) )//$GPGGA,112118.000,3743.5044,N,11540.5393,E,1,06,1.6,15.3,M,-9.1,M,,0000*7E
  {
    GPS_Information.Use_EPH_Sum = atof( Real_Process_DH( GPS_Real_buf.data, 7 ) ); //��7������Ϊ����ʹ������
    GPS_Information.MSL_Altitude = atof( Real_Process_DH( GPS_Real_buf.data, 9 ) ); //��9������Ϊ���θ߶�
    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPGSA" ) )  //$GPGSA,A,3,28,17,11,09,08,07,,,,,,,3.4,1.6,3.0*3B
  {
    temp = Real_Process_DH( GPS_Real_buf.data, 2 ); //��2������Ϊ��λģʽ
   if ( ( *temp == '2' ) || ( *temp == '3' ) )
      GPS_Information.Locate_Mode = *temp;
    else
      GPS_Information.Locate_Mode = Invalid;

    for ( i = 0; i < 12; i++ ) //�ܹ����Ϊ12����
    {
      GPS_Information.User_EPH[i] = atof( Real_Process_DH( GPS_Real_buf.data, i + 3 ) ); //�ӵ�3��������ʼΪ��ʹ�õ����Ǳ��
    }

    GPS_Information.PDOP = atof( Real_Process_DH( GPS_Real_buf.data, 15 ) ); //��15������Ϊλ�þ���
    GPS_Information.HDOP = atof( Real_Process_DH( GPS_Real_buf.data, 16 ) ); //��16������Ϊˮƽ����
    GPS_Information.VDOP = atof( Real_Process_DH( GPS_Real_buf.data, 17 ) ); //��17������Ϊ��ֱ����

    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPRMC" ) )//$GPRMC,112118.000,A,3743.5044,N,11540.5393,E,0.25,198.81,130613,,,A*67
  {
    temp = Real_Process_DH( GPS_Real_buf.data, 1 ); //��1������Ϊʱ��
    if ( *temp != 0 )
      memcpy( GPS_Information.UTC_Time, temp, 6 );

    if ( *( Real_Process_DH( GPS_Real_buf.data, 2 ) ) == 'A' ) //��2������
    {
      GPS_Information.Real_Locate = Valid; //ʵʱ������Ч
      GPS_Information.Located = Valid;
    }
    else
    {
      GPS_Information.Real_Locate = Invalid;  //ʵʱ������Ч
    }

    temp = Real_Process_DH( GPS_Real_buf.data, 3 ); //��3������Ϊγ��
    if ( ( *temp >= 0x31 ) && ( *temp <= 0x39 ) )
    {
      memcpy( GPS_Information.Latitude, temp, 9 );
      GPS_Information.Latitude[9] = 0;
    }
    else
    {
      GPS_Information.Latitude[0] = '0';
      GPS_Information.Latitude[1] = 0;
    }

    GPS_Information.NS_Indicator = *( Real_Process_DH( GPS_Real_buf.data, 4 ) ); //��4������Ϊ�ϱ�

    temp = Real_Process_DH( GPS_Real_buf.data, 5 ); //��5������Ϊ����
    if ( ( *temp >= 0x31 ) && ( *temp <= 0x39 ) )
    {
      memcpy( GPS_Information.Longitude, temp, 10 );
      GPS_Information.Longitude[10] = 0;
    }
    else
    {
      GPS_Information.Longitude[0] = '0';
      GPS_Information.Longitude[1] = 0;
    }

    GPS_Information.EW_Indicator = *( Real_Process_DH( GPS_Real_buf.data, 6 ) ); //��6������Ϊ����

    GPS_Information.Speed = atof( Real_Process_DH( GPS_Real_buf.data, 7 ) ); //��7������Ϊ�ٶ�

    GPS_Information.Course = atof( Real_Process_DH( GPS_Real_buf.data, 8 ) ); //��8������Ϊ����

    temp = Real_Process_DH( GPS_Real_buf.data, 9 ); //��9������Ϊ����
    if ( *temp != 0 )
    {
      memcpy( GPS_Information.UTC_Date, temp, 6 );
    }

    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPGSV" ) )//$GPGSV,3,1,11,28,73,321,32,17,39,289,43,11,38,053,17,09,37,250,41*78
  {
    zhen = atof( Real_Process_DH( GPS_Real_buf.data, 2 ) ); //ȡ��ǰ֡��
    if ( ( zhen <= 3 ) && ( zhen != 0 ) )
    {
      for ( i = ( zhen - 1 ) * 4, j = 4; i < zhen * 4; i++ )
      {
        GPS_Information.EPH_State[i][0] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //ȡ�Ǻ�
        GPS_Information.EPH_State[i][1] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //ȡ����
        GPS_Information.EPH_State[i][2] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //ȡ��λ��
        GPS_Information.EPH_State[i][3] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //ȡ�����
      }
    }
    return;
  }
}

/********************************************************************************************************
**������Ϣ ��unsigned char Calc_GPS_Sum( const char* Buffer )
**�������� ������GPS��У���
**������� ��������ɵ�GPS����
**������� ��
********************************************************************************************************/
#include 
unsigned char gps_sum = 0;
unsigned char Calc_GPS_Sum( const char* Buffer )
{
  unsigned char i, j, k, sum;

  sum = 0;

  for ( i = 1; i < 255; i++ ) //i��1��ʼ������$��ʼ��
  {
    if ( ( Buffer[i] != '*' ) && ( Buffer[i] != 0x00 ) ) //�жϽ�����
    {
      sum ^= Buffer[i];//GPSУ����㷨ΪXOR
    }
    else
    {
      break;
    }
  }
  j = Buffer[i + 1];//ȡ����������λ�ַ�
  k = Buffer[i + 2];

  if ( isalpha( j ) ) //�ж��ַ��Ƿ�ΪӢ����ĸ��ΪӢ����ĸʱ���ط���ֵ�����򷵻���
  {
    if ( isupper( j ) ) //�ж��ַ�Ϊ��дӢ����ĸʱ�����ط���ֵ�����򷵻���
    {
      j -= 0x37;//ǿ��ת��Ϊ16����
    }
    else
    {
      j -= 0x57;//ǿ��ת��Ϊ16����
    }
  }
  else
  {
    if ( ( j >= 0x30 ) && ( j <= 0x39 ) )
    {
      j -= 0x30;//ǿ��ת��Ϊ16����
    }
  }

  if ( isalpha( k ) ) //�ж��ַ��Ƿ�ΪӢ����ĸ��ΪӢ����ĸʱ���ط���ֵ�����򷵻���
  {
    if ( isupper( k ) ) //�ж��ַ�Ϊ��дӢ����ĸʱ�����ط���ֵ�����򷵻���
    {
      k -= 0x37;//ǿ��ת��Ϊ16����
    }
    else
    {
      k -= 0x57;//ǿ��ת��Ϊ16����
    }
  }
  else
  {
    if ( ( k >= 0x30 ) && ( k <= 0x39 ) )
    {
      k -= 0x30;//ǿ��ת��Ϊ16����
    }
  }

  j = ( j << 4 ) + k; //ǿ�ƺϲ�Ϊ16����
  gps_sum = j;

  if ( sum == j )
  {
    return Valid; //У�������
  }
  else
  {
    return Invalid; //У��ʹ���
  }
}#include 
#include 

extern struct _GPS_Real_buf
{
  char data[256];                      //����GPS���뻺����
volatile unsigned short rx_pc;    //����ָ��
} GPS_Real_buf; //GPS�������ݻ�����


extern struct _GPS_Information
{
  unsigned char Real_Locate;            //ʵʱ��λ��Чλ
  unsigned char Located;                //��λ��Чλ
  unsigned char Locate_Mode;         //��λģʽ��2D��3D
  char UTC_Time[7];                       //ʱ��
  char UTC_Date[7];                      //����
  char Latitude[10];                       //γ��
  char NS_Indicator;                       //N=������S=�ϰ���
  char Longitude[11];                     //����
  char EW_Indicator;                      //E=������W=����
  double Speed;                            //��������
  double Course;                            //���溽��
  double PDOP;                             //λ�þ���
  double HDOP;                             //ˮƽ����
  double VDOP;                             //��ֱ����
  double MSL_Altitude;                   //MSL���θ߶�
  unsigned char Use_EPH_Sum;       //ʹ�õ���������
  unsigned char User_EPH[12];        //��ǰ�����������Ǳ��
  unsigned short EPH_State[12][4]; //��ǰʹ�õ�12�����ǵı�š����ǡ���λ�ǡ������
} GPS_Information; //GPS��Ϣ



/********************************************************************************************************
**������Ϣ ��void USART2_IRQHandler(void) 
**�������� ��UART2�жϷ�����
**������� ����
**������� ����
********************************************************************************************************/
void USART2_IRQHandler( void ) //����2�жϷ������
{
  unsigned char Recv;
  if ( USART_GetITStatus( USART2, USART_IT_RXNE ) != RESET ) //�����ж�
  {
    Recv = USART_ReceiveData( USART2 );         //��������
    if ( Recv == '$' )
    {
      GPS_Real_buf.rx_pc = 0;
    }
    else
    {
      if ( GPS_Real_buf.rx_pc < sizeof( GPS_Real_buf.data ) - 1 )
      {
        GPS_Real_buf.rx_pc++;
      }
    }
    GPS_Real_buf.data[GPS_Real_buf.rx_pc] = Recv;
    if ( Recv == '\r' )    //����������Ϊ0x0Dʱ��ʼ����GPS����
    {
      if ( Real_Process_Enabled == Valid )
      {
        if ( Calc_GPS_Sum( GPS_Real_buf.data ) == Valid )
        {
          Creat_DH_Index( GPS_Real_buf.data );
          Real_GPS_Command_Process();
        }
      }
    }
  }

  if ( USART_GetFlagStatus( USART2, USART_FLAG_ORE ) == SET ) //����ж�
  {
    USART_ClearFlag( USART2, USART_FLAG_ORE );   //�����λ
    USART_ReceiveData( USART2 );                        //��DR
  }
}


/********************************************************************************************************
**������Ϣ ��void Creat_DH_Index( char* buffer )
**�������� ��Ѱ�����ж��ŵ�λ�ã���������
**������� ������GPS���ݵĻ�����
**������� ������ȫ�ֱ��������еĶ���������ԭGPS�����еĶ��Ž��ᱻ0x00���
********************************************************************************************************/
unsigned char DH_id_sep[32];//ȫ�ֱ������飬��ദ��32����������
void Creat_DH_Index( char* buffer )
{
  unsigned short i, len;
  unsigned char idj;

  memset( DH_id_sep, 0, sizeof( DH_id_sep ) );
  len = strlen( buffer );
  for ( i = 0, idj = 0; i < len; i++ )
  {
    if ( buffer[i] == ',' )
    {
      DH_id_sep[idj] = i;
      idj++;
      buffer[i] = 0x00;
    }
  }
}

/********************************************************************************************************
**������Ϣ ��char* Real_Process_DH( char* buffer, unsigned char num )
**�������� ������GPS���ݵ�N��������ƫ��
**������� ������������Ľ���GPS���ݻ�����
**������� �����ص�N��","֮�����Ϣ����Ҫ*buffer��Ч������������ſ���ִ��
********************************************************************************************************/
char* Real_Process_DH( char* buffer, unsigned char num )
{
  if ( num < 1 )
    return  &buffer[0];
  return  &buffer[ DH_id_sep[num - 1] + 1];
}


/********************************************************************************************************
**������Ϣ ��void Real_GPS_Command_Process( void )
**�������� ���������ò������������ݲ�����GPS���ݽṹ��
**������� ��
**������� ��
********************************************************************************************************/
void Real_GPS_Command_Process( void )
{
  char* temp;
  unsigned char i, j, zhen;

  if ( strstr( GPS_Real_buf.data, "GPGGA" ) )//$GPGGA,112118.000,3743.5044,N,11540.5393,E,1,06,1.6,15.3,M,-9.1,M,,0000*7E
  {
    GPS_Information.Use_EPH_Sum = atof( Real_Process_DH( GPS_Real_buf.data, 7 ) ); //��7������Ϊ����ʹ������
    GPS_Information.MSL_Altitude = atof( Real_Process_DH( GPS_Real_buf.data, 9 ) ); //��9������Ϊ���θ߶�
    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPGSA" ) )  //$GPGSA,A,3,28,17,11,09,08,07,,,,,,,3.4,1.6,3.0*3B
  {
    temp = Real_Process_DH( GPS_Real_buf.data, 2 ); //��2������Ϊ��λģʽ
   if ( ( *temp == '2' ) || ( *temp == '3' ) )
      GPS_Information.Locate_Mode = *temp;
    else
      GPS_Information.Locate_Mode = Invalid;

    for ( i = 0; i < 12; i++ ) //�ܹ����Ϊ12����
    {
      GPS_Information.User_EPH[i] = atof( Real_Process_DH( GPS_Real_buf.data, i + 3 ) ); //�ӵ�3��������ʼΪ��ʹ�õ����Ǳ��
    }

    GPS_Information.PDOP = atof( Real_Process_DH( GPS_Real_buf.data, 15 ) ); //��15������Ϊλ�þ���
    GPS_Information.HDOP = atof( Real_Process_DH( GPS_Real_buf.data, 16 ) ); //��16������Ϊˮƽ����
    GPS_Information.VDOP = atof( Real_Process_DH( GPS_Real_buf.data, 17 ) ); //��17������Ϊ��ֱ����

    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPRMC" ) )//$GPRMC,112118.000,A,3743.5044,N,11540.5393,E,0.25,198.81,130613,,,A*67
  {
    temp = Real_Process_DH( GPS_Real_buf.data, 1 ); //��1������Ϊʱ��
    if ( *temp != 0 )
      memcpy( GPS_Information.UTC_Time, temp, 6 );

    if ( *( Real_Process_DH( GPS_Real_buf.data, 2 ) ) == 'A' ) //��2������
    {
      GPS_Information.Real_Locate = Valid; //ʵʱ������Ч
      GPS_Information.Located = Valid;
    }
    else
    {
      GPS_Information.Real_Locate = Invalid;  //ʵʱ������Ч
    }

    temp = Real_Process_DH( GPS_Real_buf.data, 3 ); //��3������Ϊγ��
    if ( ( *temp >= 0x31 ) && ( *temp <= 0x39 ) )
    {
      memcpy( GPS_Information.Latitude, temp, 9 );
      GPS_Information.Latitude[9] = 0;
    }
    else
    {
      GPS_Information.Latitude[0] = '0';
      GPS_Information.Latitude[1] = 0;
    }

    GPS_Information.NS_Indicator = *( Real_Process_DH( GPS_Real_buf.data, 4 ) ); //��4������Ϊ�ϱ�

    temp = Real_Process_DH( GPS_Real_buf.data, 5 ); //��5������Ϊ����
    if ( ( *temp >= 0x31 ) && ( *temp <= 0x39 ) )
    {
      memcpy( GPS_Information.Longitude, temp, 10 );
      GPS_Information.Longitude[10] = 0;
    }
    else
    {
      GPS_Information.Longitude[0] = '0';
      GPS_Information.Longitude[1] = 0;
    }

    GPS_Information.EW_Indicator = *( Real_Process_DH( GPS_Real_buf.data, 6 ) ); //��6������Ϊ����

    GPS_Information.Speed = atof( Real_Process_DH( GPS_Real_buf.data, 7 ) ); //��7������Ϊ�ٶ�

    GPS_Information.Course = atof( Real_Process_DH( GPS_Real_buf.data, 8 ) ); //��8������Ϊ����

    temp = Real_Process_DH( GPS_Real_buf.data, 9 ); //��9������Ϊ����
    if ( *temp != 0 )
    {
      memcpy( GPS_Information.UTC_Date, temp, 6 );
    }

    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPGSV" ) )//$GPGSV,3,1,11,28,73,321,32,17,39,289,43,11,38,053,17,09,37,250,41*78
  {
    zhen = atof( Real_Process_DH( GPS_Real_buf.data, 2 ) ); //ȡ��ǰ֡��
    if ( ( zhen <= 3 ) && ( zhen != 0 ) )
    {
      for ( i = ( zhen - 1 ) * 4, j = 4; i < zhen * 4; i++ )
      {
        GPS_Information.EPH_State[i][0] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //ȡ�Ǻ�
        GPS_Information.EPH_State[i][1] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //ȡ����
        GPS_Information.EPH_State[i][2] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //ȡ��λ��
        GPS_Information.EPH_State[i][3] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //ȡ�����
      }
    }
    return;
  }
}

/********************************************************************************************************
**������Ϣ ��unsigned char Calc_GPS_Sum( const char* Buffer )
**�������� ������GPS��У���
**������� ��������ɵ�GPS����
**������� ��
********************************************************************************************************/
#include 
unsigned char gps_sum = 0;
unsigned char Calc_GPS_Sum( const char* Buffer )
{
  unsigned char i, j, k, sum;

  sum = 0;

  for ( i = 1; i < 255; i++ ) //i��1��ʼ������$��ʼ��
  {
    if ( ( Buffer[i] != '*' ) && ( Buffer[i] != 0x00 ) ) //�жϽ�����
    {
      sum ^= Buffer[i];//GPSУ����㷨ΪXOR
    }
    else
    {
      break;
    }
  }
  j = Buffer[i + 1];//ȡ����������λ�ַ�
  k = Buffer[i + 2];

  if ( isalpha( j ) ) //�ж��ַ��Ƿ�ΪӢ����ĸ��ΪӢ����ĸʱ���ط���ֵ�����򷵻���
  {
    if ( isupper( j ) ) //�ж��ַ�Ϊ��дӢ����ĸʱ�����ط���ֵ�����򷵻���
    {
      j -= 0x37;//ǿ��ת��Ϊ16����
    }
    else
    {
      j -= 0x57;//ǿ��ת��Ϊ16����
    }
  }
  else
  {
    if ( ( j >= 0x30 ) && ( j <= 0x39 ) )
    {
      j -= 0x30;//ǿ��ת��Ϊ16����
    }
  }

  if ( isalpha( k ) ) //�ж��ַ��Ƿ�ΪӢ����ĸ��ΪӢ����ĸʱ���ط���ֵ�����򷵻���
  {
    if ( isupper( k ) ) //�ж��ַ�Ϊ��дӢ����ĸʱ�����ط���ֵ�����򷵻���
    {
      k -= 0x37;//ǿ��ת��Ϊ16����
    }
    else
    {
      k -= 0x57;//ǿ��ת��Ϊ16����
    }
  }
  else
  {
    if ( ( k >= 0x30 ) && ( k <= 0x39 ) )
    {
      k -= 0x30;//ǿ��ת��Ϊ16����
    }
  }

  j = ( j << 4 ) + k; //ǿ�ƺϲ�Ϊ16����
  gps_sum = j;

  if ( sum == j )
  {
    return Valid; //У�������
  }
  else
  {
    return Invalid; //У��ʹ���
  }
}

#endif


