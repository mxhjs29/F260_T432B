#include "ANO_GCS_DT.h"
#include "HARDWARE_uart.h"

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define MYHWADDR	0x05
#define SWJADDR		0xAF

#define PARNUM		100
//s32 ParValList[100];		//参数列表

//dt_flag_t f;					//需要发送数据的标志
u8 data_to_send[50];	//发送数据缓存
u8 checkdata_to_send,checksum_to_send;

//=====解析数据=====
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==MYHWADDR)
	{
		if(*(data_buf+3)==0XE0)			//命令E0
		{
			switch(*(data_buf+5))		//FUN
			{
				case 0x01:
//					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x01)
//						sensor.acc_CALIBRATE = 1;
//					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x02)
//						sensor.gyr_CALIBRATE = 1;
//					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x04)
//						mag.mag_CALIBRATE = 1;
//					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xB0)//读取版本信息
//						f.send_version = 1;
					break;
				case 0x02:
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAA)	//恢复默认PID
					{
//						PID_Rest();
//						All_PID_Init();
//						data_save();
					}
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAB)	//恢复默认参数
					{
//						Parame_Reset();
//						data_save();
					}
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAF)	//恢复所有参数
					{
//						PID_Rest();
//						All_PID_Init();
//						Parame_Reset();
//						data_save();
					}
					break;
				case 0xE1:
//					f.paraToSend = (u16)(*(data_buf+6)<<8)|*(data_buf+7);	//读取参数
					break;
				case 0x10:
					Program_Ctrl_DataAnl(data_buf+5);
					break;
				case 0x11:
					break;
				default:
					break;
			}
			ANO_DT_SendCmd(SWJADDR,*(data_buf+5),(u16)(*(data_buf+6)<<8)|*(data_buf+7),(u16)(*(data_buf+8)<<8)|*(data_buf+9),(u16)(*(data_buf+10)<<8)|*(data_buf+11),(u16)(*(data_buf+12)<<8)|*(data_buf+13),(u16)(*(data_buf+14)<<8)|*(data_buf+15));
		}
		else if(*(data_buf+3)==0XE1)	//设置参数
		{
//			u16 _paraNum = (u16)(*(data_buf+5)<<8)|*(data_buf+6);
//			s32 _paraVal = (s32)(((*(data_buf+7))<<24) + ((*(data_buf+8))<<16) + ((*(data_buf+9))<<8) + (*(data_buf+10)));
//			ANO_DT_GetParame(_paraNum,_paraVal);
		}
	}
}

static void ANO_DT_SendCmd(u8 dest, u8 fun, u16 cmd1, u16 cmd2, u16 cmd3, u16 cmd4, u16 cmd5)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=dest;
	data_to_send[_cnt++]=0xE0;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=fun;
	data_to_send[_cnt++]=BYTE1(cmd1);
	data_to_send[_cnt++]=BYTE0(cmd1);
	data_to_send[_cnt++]=BYTE1(cmd2);
	data_to_send[_cnt++]=BYTE0(cmd2);
	data_to_send[_cnt++]=BYTE1(cmd3);
	data_to_send[_cnt++]=BYTE0(cmd3);
	data_to_send[_cnt++]=BYTE1(cmd4);
	data_to_send[_cnt++]=BYTE0(cmd4);
	data_to_send[_cnt++]=BYTE1(cmd5);
	data_to_send[_cnt++]=BYTE0(cmd5);
	
	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

/////////////////////////////////////////////////////////////////////////////////////
//=====接收数据=====
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
static u8 DT_RxBuffer[256],DT_data_cnt = 0,ano_dt_data_ok;

//查询一组数据是否接收完成，然后解析。
void ANO_DT_Data_Receive_Anl_Task()
{
	if(ano_dt_data_ok)
	{
		ANO_DT_Data_Receive_Anl(DT_RxBuffer,DT_data_cnt+6);
		ano_dt_data_ok = 0;
	}
}

//BYTE数据接收，可放入串口接收中断
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)	//帧头0xAA
	{
		state=1;
		DT_RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)	//数据源，0xAF表示数据来自上位机
	{
		state=2;
		DT_RxBuffer[1]=data;
	}
	else if(state==2)		//数据目的地
	{
		state=3;
		DT_RxBuffer[2]=data;
	}
	else if(state==3)		//功能字
	{
		state=4;
		DT_RxBuffer[3]=data;
	}
	else if(state==4)		//数据长度
	{
		state = 5;
		DT_RxBuffer[4]=data;
		_data_len = data;
		DT_data_cnt = 0;
	}
	else if(state==5&&_data_len>0)
	{
		_data_len--;
		DT_RxBuffer[5+DT_data_cnt++]=data;
		if(_data_len==0)
			state = 6;
	}
	else if(state==6)
	{
		state = 0;
		DT_RxBuffer[5+DT_data_cnt]=data;
		ano_dt_data_ok = 1;//ANO_DT_Data_Receive_Anl(DT_RxBuffer,DT_data_cnt+5);
	}
	else
		state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//=====发送数据=====
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
static void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{
	//usb发送
#ifdef ANO_DT_USE_USB_HID
	Usb_Hid_Adddata(data_to_send,length);
#endif
	//串口发送
#ifdef ANO_DT_USE_USART2
    USART_TX(&UsartGroup[UART_A3], data_to_send, length);
	//Usart2_Send(data_to_send, length);
#endif
	
}
