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
  char data[256];                      //定义GPS输入缓冲区
volatile unsigned short rx_pc;    //接收指针
} GPS_Real_buf; //GPS接收数据缓冲区


extern struct _GPS_Information
{
  unsigned char Real_Locate;            //实时定位有效位
  unsigned char Located;                //定位有效位
  unsigned char Locate_Mode;         //定位模式，2D或3D
  char UTC_Time[7];                       //时间
  char UTC_Date[7];                      //日期
  char Latitude[10];                       //纬度
  char NS_Indicator;                       //N=北半球，S=南半球
  char Longitude[11];                     //经度
  char EW_Indicator;                      //E=东经，W=西经
  double Speed;                            //地面速率
  double Course;                            //地面航向
  double PDOP;                             //位置精度
  double HDOP;                             //水平精度
  double VDOP;                             //垂直精度
  double MSL_Altitude;                   //MSL海拔高度
  unsigned char Use_EPH_Sum;       //使用的卫星数量
  unsigned char User_EPH[12];        //当前搜索到的卫星编号
  unsigned short EPH_State[12][4]; //当前使用的12颗卫星的编号、仰角、方位角、信噪比
} GPS_Information; 



/********************************************************************************************************
**函数信息 ：void USART2_IRQHandler(void) 
**功能描述 ：UART2中断服务函数
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void USART2_IRQHandler( void ) //串口2中断服务程序
{
  unsigned char Recv;
  if ( USART_GetITStatus( USART2, USART_IT_RXNE ) != RESET ) //接收中断
  {
    Recv = USART_ReceiveData( USART2 );         //接收数据
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
    if ( Recv == '\r' )    //当接收数据为0x0D时开始处理GPS数据
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

  if ( USART_GetFlagStatus( USART2, USART_FLAG_ORE ) == SET ) //溢出中断
  {
    USART_ClearFlag( USART2, USART_FLAG_ORE );   //清溢出位
    USART_ReceiveData( USART2 );                        //读DR
  }
}


/********************************************************************************************************
**函数信息 ：void Creat_DH_Index( char* buffer )
**功能描述 ：寻找所有逗号的位置，创建索引
**输入参数 ：接收GPS数据的缓冲区
**输出参数 ：创建全局变量数组中的逗号索引，原GPS数据中的逗号将会被0x00替代
********************************************************************************************************/
unsigned char DH_id_sep[32];//全局变量数组，最多处理32个逗号索引
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
**函数信息 ：char* Real_Process_DH( char* buffer, unsigned char num )
**功能描述 ：查找GPS数据第N个参数的偏移
**输入参数 ：创建索引后的接收GPS数据缓冲区
**输出参数 ：返回第N个","之后的信息，需要*buffer有效并创建索引后才可以执行
********************************************************************************************************/
char* Real_Process_DH( char* buffer, unsigned char num )
{
  if ( num < 1 )
    return  &buffer[0];
  return  &buffer[ DH_id_sep[num - 1] + 1];
}


/********************************************************************************************************
**函数信息 ：void Real_GPS_Command_Process( void )
**功能描述 ：处理做好参数索引的数据并填入GPS数据结构中
**输入参数 ：
**输出参数 ：
********************************************************************************************************/
void Real_GPS_Command_Process( void )
{
  char* temp;
  unsigned char i, j, zhen;

  if ( strstr( GPS_Real_buf.data, "GPGGA" ) )//$GPGGA,112118.000,3743.5044,N,11540.5393,E,1,06,1.6,15.3,M,-9.1,M,,0000*7E
  {
    GPS_Information.Use_EPH_Sum = atof( Real_Process_DH( GPS_Real_buf.data, 7 ) ); //第7个参数为卫星使用数量
    GPS_Information.MSL_Altitude = atof( Real_Process_DH( GPS_Real_buf.data, 9 ) ); //第9个参数为海拔高度
    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPGSA" ) )  //$GPGSA,A,3,28,17,11,09,08,07,,,,,,,3.4,1.6,3.0*3B
  {
    temp = Real_Process_DH( GPS_Real_buf.data, 2 ); //第2个参数为定位模式
   if ( ( *temp == '2' ) || ( *temp == '3' ) )
      GPS_Information.Locate_Mode = *temp;
    else
      GPS_Information.Locate_Mode = Invalid;

    for ( i = 0; i < 12; i++ ) //总共最多为12颗星
    {
      GPS_Information.User_EPH[i] = atof( Real_Process_DH( GPS_Real_buf.data, i + 3 ) ); //从第3个参数开始为所使用的卫星编号
    }

    GPS_Information.PDOP = atof( Real_Process_DH( GPS_Real_buf.data, 15 ) ); //第15个参数为位置精度
    GPS_Information.HDOP = atof( Real_Process_DH( GPS_Real_buf.data, 16 ) ); //第16个参数为水平精度
    GPS_Information.VDOP = atof( Real_Process_DH( GPS_Real_buf.data, 17 ) ); //第17个参数为垂直精度

    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPRMC" ) )//$GPRMC,112118.000,A,3743.5044,N,11540.5393,E,0.25,198.81,130613,,,A*67
  {
    temp = Real_Process_DH( GPS_Real_buf.data, 1 ); //第1个参数为时间
    if ( *temp != 0 )
      memcpy( GPS_Information.UTC_Time, temp, 6 );

    if ( *( Real_Process_DH( GPS_Real_buf.data, 2 ) ) == 'A' ) //第2个参数
    {
      GPS_Information.Real_Locate = Valid; //实时数据有效
      GPS_Information.Located = Valid;
    }
    else
    {
      GPS_Information.Real_Locate = Invalid;  //实时数据无效
    }

    temp = Real_Process_DH( GPS_Real_buf.data, 3 ); //第3个参数为纬度
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

    GPS_Information.NS_Indicator = *( Real_Process_DH( GPS_Real_buf.data, 4 ) ); //第4个参数为南北

    temp = Real_Process_DH( GPS_Real_buf.data, 5 ); //第5个参数为经度
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

    GPS_Information.EW_Indicator = *( Real_Process_DH( GPS_Real_buf.data, 6 ) ); //第6个参数为东西

    GPS_Information.Speed = atof( Real_Process_DH( GPS_Real_buf.data, 7 ) ); //第7个参数为速度

    GPS_Information.Course = atof( Real_Process_DH( GPS_Real_buf.data, 8 ) ); //第8个参数为航向

    temp = Real_Process_DH( GPS_Real_buf.data, 9 ); //第9个参数为日期
    if ( *temp != 0 )
    {
      memcpy( GPS_Information.UTC_Date, temp, 6 );
    }

    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPGSV" ) )//$GPGSV,3,1,11,28,73,321,32,17,39,289,43,11,38,053,17,09,37,250,41*78
  {
    zhen = atof( Real_Process_DH( GPS_Real_buf.data, 2 ) ); //取当前帧号
    if ( ( zhen <= 3 ) && ( zhen != 0 ) )
    {
      for ( i = ( zhen - 1 ) * 4, j = 4; i < zhen * 4; i++ )
      {
        GPS_Information.EPH_State[i][0] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //取星号
        GPS_Information.EPH_State[i][1] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //取仰角
        GPS_Information.EPH_State[i][2] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //取方位角
        GPS_Information.EPH_State[i][3] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //取信噪比
      }
    }
    return;
  }
}

/********************************************************************************************************
**函数信息 ：unsigned char Calc_GPS_Sum( const char* Buffer )
**功能描述 ：计算GPS的校验和
**输入参数 ：接收完成的GPS数据
**输出参数 ：
********************************************************************************************************/
#include 
unsigned char gps_sum = 0;
unsigned char Calc_GPS_Sum( const char* Buffer )
{
  unsigned char i, j, k, sum;

  sum = 0;

  for ( i = 1; i < 255; i++ ) //i从1开始是闪过$开始符
  {
    if ( ( Buffer[i] != '*' ) && ( Buffer[i] != 0x00 ) ) //判断结束符
    {
      sum ^= Buffer[i];//GPS校验和算法为XOR
    }
    else
    {
      break;
    }
  }
  j = Buffer[i + 1];//取结束符后两位字符
  k = Buffer[i + 2];

  if ( isalpha( j ) ) //判断字符是否为英文字母，为英文字母时返回非零值，否则返回零
  {
    if ( isupper( j ) ) //判断字符为大写英文字母时，返回非零值，否则返回零
    {
      j -= 0x37;//强制转换为16进制
    }
    else
    {
      j -= 0x57;//强制转换为16进制
    }
  }
  else
  {
    if ( ( j >= 0x30 ) && ( j <= 0x39 ) )
    {
      j -= 0x30;//强制转换为16进制
    }
  }

  if ( isalpha( k ) ) //判断字符是否为英文字母，为英文字母时返回非零值，否则返回零
  {
    if ( isupper( k ) ) //判断字符为大写英文字母时，返回非零值，否则返回零
    {
      k -= 0x37;//强制转换为16进制
    }
    else
    {
      k -= 0x57;//强制转换为16进制
    }
  }
  else
  {
    if ( ( k >= 0x30 ) && ( k <= 0x39 ) )
    {
      k -= 0x30;//强制转换为16进制
    }
  }

  j = ( j << 4 ) + k; //强制合并为16进制
  gps_sum = j;

  if ( sum == j )
  {
    return Valid; //校验和正常
  }
  else
  {
    return Invalid; //校验和错误
  }
}#include 
#include 

extern struct _GPS_Real_buf
{
  char data[256];                      //定义GPS输入缓冲区
volatile unsigned short rx_pc;    //接收指针
} GPS_Real_buf; //GPS接收数据缓冲区


extern struct _GPS_Information
{
  unsigned char Real_Locate;            //实时定位有效位
  unsigned char Located;                //定位有效位
  unsigned char Locate_Mode;         //定位模式，2D或3D
  char UTC_Time[7];                       //时间
  char UTC_Date[7];                      //日期
  char Latitude[10];                       //纬度
  char NS_Indicator;                       //N=北半球，S=南半球
  char Longitude[11];                     //经度
  char EW_Indicator;                      //E=东经，W=西经
  double Speed;                            //地面速率
  double Course;                            //地面航向
  double PDOP;                             //位置精度
  double HDOP;                             //水平精度
  double VDOP;                             //垂直精度
  double MSL_Altitude;                   //MSL海拔高度
  unsigned char Use_EPH_Sum;       //使用的卫星数量
  unsigned char User_EPH[12];        //当前搜索到的卫星编号
  unsigned short EPH_State[12][4]; //当前使用的12颗卫星的编号、仰角、方位角、信噪比
} GPS_Information; //GPS信息



/********************************************************************************************************
**函数信息 ：void USART2_IRQHandler(void) 
**功能描述 ：UART2中断服务函数
**输入参数 ：无
**输出参数 ：无
********************************************************************************************************/
void USART2_IRQHandler( void ) //串口2中断服务程序
{
  unsigned char Recv;
  if ( USART_GetITStatus( USART2, USART_IT_RXNE ) != RESET ) //接收中断
  {
    Recv = USART_ReceiveData( USART2 );         //接收数据
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
    if ( Recv == '\r' )    //当接收数据为0x0D时开始处理GPS数据
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

  if ( USART_GetFlagStatus( USART2, USART_FLAG_ORE ) == SET ) //溢出中断
  {
    USART_ClearFlag( USART2, USART_FLAG_ORE );   //清溢出位
    USART_ReceiveData( USART2 );                        //读DR
  }
}


/********************************************************************************************************
**函数信息 ：void Creat_DH_Index( char* buffer )
**功能描述 ：寻找所有逗号的位置，创建索引
**输入参数 ：接收GPS数据的缓冲区
**输出参数 ：创建全局变量数组中的逗号索引，原GPS数据中的逗号将会被0x00替代
********************************************************************************************************/
unsigned char DH_id_sep[32];//全局变量数组，最多处理32个逗号索引
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
**函数信息 ：char* Real_Process_DH( char* buffer, unsigned char num )
**功能描述 ：查找GPS数据第N个参数的偏移
**输入参数 ：创建索引后的接收GPS数据缓冲区
**输出参数 ：返回第N个","之后的信息，需要*buffer有效并创建索引后才可以执行
********************************************************************************************************/
char* Real_Process_DH( char* buffer, unsigned char num )
{
  if ( num < 1 )
    return  &buffer[0];
  return  &buffer[ DH_id_sep[num - 1] + 1];
}


/********************************************************************************************************
**函数信息 ：void Real_GPS_Command_Process( void )
**功能描述 ：处理做好参数索引的数据并填入GPS数据结构中
**输入参数 ：
**输出参数 ：
********************************************************************************************************/
void Real_GPS_Command_Process( void )
{
  char* temp;
  unsigned char i, j, zhen;

  if ( strstr( GPS_Real_buf.data, "GPGGA" ) )//$GPGGA,112118.000,3743.5044,N,11540.5393,E,1,06,1.6,15.3,M,-9.1,M,,0000*7E
  {
    GPS_Information.Use_EPH_Sum = atof( Real_Process_DH( GPS_Real_buf.data, 7 ) ); //第7个参数为卫星使用数量
    GPS_Information.MSL_Altitude = atof( Real_Process_DH( GPS_Real_buf.data, 9 ) ); //第9个参数为海拔高度
    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPGSA" ) )  //$GPGSA,A,3,28,17,11,09,08,07,,,,,,,3.4,1.6,3.0*3B
  {
    temp = Real_Process_DH( GPS_Real_buf.data, 2 ); //第2个参数为定位模式
   if ( ( *temp == '2' ) || ( *temp == '3' ) )
      GPS_Information.Locate_Mode = *temp;
    else
      GPS_Information.Locate_Mode = Invalid;

    for ( i = 0; i < 12; i++ ) //总共最多为12颗星
    {
      GPS_Information.User_EPH[i] = atof( Real_Process_DH( GPS_Real_buf.data, i + 3 ) ); //从第3个参数开始为所使用的卫星编号
    }

    GPS_Information.PDOP = atof( Real_Process_DH( GPS_Real_buf.data, 15 ) ); //第15个参数为位置精度
    GPS_Information.HDOP = atof( Real_Process_DH( GPS_Real_buf.data, 16 ) ); //第16个参数为水平精度
    GPS_Information.VDOP = atof( Real_Process_DH( GPS_Real_buf.data, 17 ) ); //第17个参数为垂直精度

    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPRMC" ) )//$GPRMC,112118.000,A,3743.5044,N,11540.5393,E,0.25,198.81,130613,,,A*67
  {
    temp = Real_Process_DH( GPS_Real_buf.data, 1 ); //第1个参数为时间
    if ( *temp != 0 )
      memcpy( GPS_Information.UTC_Time, temp, 6 );

    if ( *( Real_Process_DH( GPS_Real_buf.data, 2 ) ) == 'A' ) //第2个参数
    {
      GPS_Information.Real_Locate = Valid; //实时数据有效
      GPS_Information.Located = Valid;
    }
    else
    {
      GPS_Information.Real_Locate = Invalid;  //实时数据无效
    }

    temp = Real_Process_DH( GPS_Real_buf.data, 3 ); //第3个参数为纬度
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

    GPS_Information.NS_Indicator = *( Real_Process_DH( GPS_Real_buf.data, 4 ) ); //第4个参数为南北

    temp = Real_Process_DH( GPS_Real_buf.data, 5 ); //第5个参数为经度
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

    GPS_Information.EW_Indicator = *( Real_Process_DH( GPS_Real_buf.data, 6 ) ); //第6个参数为东西

    GPS_Information.Speed = atof( Real_Process_DH( GPS_Real_buf.data, 7 ) ); //第7个参数为速度

    GPS_Information.Course = atof( Real_Process_DH( GPS_Real_buf.data, 8 ) ); //第8个参数为航向

    temp = Real_Process_DH( GPS_Real_buf.data, 9 ); //第9个参数为日期
    if ( *temp != 0 )
    {
      memcpy( GPS_Information.UTC_Date, temp, 6 );
    }

    return;
  }

  if ( strstr( GPS_Real_buf.data, "GPGSV" ) )//$GPGSV,3,1,11,28,73,321,32,17,39,289,43,11,38,053,17,09,37,250,41*78
  {
    zhen = atof( Real_Process_DH( GPS_Real_buf.data, 2 ) ); //取当前帧号
    if ( ( zhen <= 3 ) && ( zhen != 0 ) )
    {
      for ( i = ( zhen - 1 ) * 4, j = 4; i < zhen * 4; i++ )
      {
        GPS_Information.EPH_State[i][0] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //取星号
        GPS_Information.EPH_State[i][1] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //取仰角
        GPS_Information.EPH_State[i][2] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //取方位角
        GPS_Information.EPH_State[i][3] = atof( Real_Process_DH( GPS_Real_buf.data, j++ ) ); //取信噪比
      }
    }
    return;
  }
}

/********************************************************************************************************
**函数信息 ：unsigned char Calc_GPS_Sum( const char* Buffer )
**功能描述 ：计算GPS的校验和
**输入参数 ：接收完成的GPS数据
**输出参数 ：
********************************************************************************************************/
#include 
unsigned char gps_sum = 0;
unsigned char Calc_GPS_Sum( const char* Buffer )
{
  unsigned char i, j, k, sum;

  sum = 0;

  for ( i = 1; i < 255; i++ ) //i从1开始是闪过$开始符
  {
    if ( ( Buffer[i] != '*' ) && ( Buffer[i] != 0x00 ) ) //判断结束符
    {
      sum ^= Buffer[i];//GPS校验和算法为XOR
    }
    else
    {
      break;
    }
  }
  j = Buffer[i + 1];//取结束符后两位字符
  k = Buffer[i + 2];

  if ( isalpha( j ) ) //判断字符是否为英文字母，为英文字母时返回非零值，否则返回零
  {
    if ( isupper( j ) ) //判断字符为大写英文字母时，返回非零值，否则返回零
    {
      j -= 0x37;//强制转换为16进制
    }
    else
    {
      j -= 0x57;//强制转换为16进制
    }
  }
  else
  {
    if ( ( j >= 0x30 ) && ( j <= 0x39 ) )
    {
      j -= 0x30;//强制转换为16进制
    }
  }

  if ( isalpha( k ) ) //判断字符是否为英文字母，为英文字母时返回非零值，否则返回零
  {
    if ( isupper( k ) ) //判断字符为大写英文字母时，返回非零值，否则返回零
    {
      k -= 0x37;//强制转换为16进制
    }
    else
    {
      k -= 0x57;//强制转换为16进制
    }
  }
  else
  {
    if ( ( k >= 0x30 ) && ( k <= 0x39 ) )
    {
      k -= 0x30;//强制转换为16进制
    }
  }

  j = ( j << 4 ) + k; //强制合并为16进制
  gps_sum = j;

  if ( sum == j )
  {
    return Valid; //校验和正常
  }
  else
  {
    return Invalid; //校验和错误
  }
}

#endif


