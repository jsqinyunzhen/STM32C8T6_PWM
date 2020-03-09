#include "USART.h"
#include "ADC.h"
#include "delay.h"
#include "MPU6050.h"

void USART_init(void)           //串口初始化
{
	   GPIO_InitTypeDef  Usart_IOinit;                        //配置IO口
	   USART_InitTypeDef  Usart_init;                         //配置串口
	   NVIC_InitTypeDef  NVIC_init;                           //配置中断
	
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能端口A时钟
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //使能串口1时钟
	
	   Usart_IOinit.GPIO_Pin=GPIO_Pin_9;                      //配置GPIOA9  TX
	   Usart_IOinit.GPIO_Speed=GPIO_Speed_50MHz;
	   Usart_IOinit.GPIO_Mode=GPIO_Mode_AF_PP;
	   GPIO_Init(GPIOA,&Usart_IOinit);
	
	   Usart_IOinit.GPIO_Pin=GPIO_Pin_10;                     //配置GPIOA10  RX
	   Usart_IOinit.GPIO_Speed=GPIO_Speed_50MHz;
	   Usart_IOinit.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	   GPIO_Init(GPIOA,&Usart_IOinit);
	
	   Usart_init.USART_BaudRate=9600;                        //配置串口
	   Usart_init.USART_WordLength=USART_HardwareFlowControl_None;
	   Usart_init.USART_StopBits=USART_StopBits_1;
	   Usart_init.USART_Parity=USART_Parity_No;
	   Usart_init.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	   Usart_init.USART_HardwareFlowControl=USART_WordLength_8b;
	   USART_Init(USART1,&Usart_init);
	   
	   USART_Cmd(USART1,ENABLE);                             //使能串口1
	   
	   USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);          //选择串口中断
	   
	   NVIC_init.NVIC_IRQChannel=USART1_IRQn;                //配置中断函数
	   NVIC_init.NVIC_IRQChannelCmd=ENABLE;
	   NVIC_init.NVIC_IRQChannelPreemptionPriority=1;
	   NVIC_init.NVIC_IRQChannelSubPriority=1;
	   NVIC_Init(&NVIC_init);
}

//串口1发送1个字符 
//c:要发送的字符
void usart1_send_data(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	USART_SendData(USART1,c);  
} 

void usart1_niming_send(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_data(send_buf[i]);	//发送数据到串口1 
}

//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_send(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_send(0XAF,tbuf,28);//飞控显示帧,0XAF
}

void USART1_IRQHandler(void)                         //串口1中断函数
{
	   if(USART_GetITStatus(USART1,USART_IT_RXNE))
	   {
		    u16 data;
			  //short GX,GY,GZ;
			  //data = MPU_INIT();
			  //MPU_Get_TLY(&GX,&GY,&GZ);
		    data=USART_ReceiveData(USART1);
		    //data=Get_Adc_Average(0,10);
			  //data1=data>>8;
			  USART_SendData(USART1,data);
			  //delay_ms(100);
			 //USART_SendData(USART1, MPU_Read_Byte(MPU_DEVICE_ID));
			 //delay_ms(100);
			  //USART_SendData(USART1, data);
			 //delay_ms(100);
			  //USART_SendData(USART1, GX);
			 //delay_ms(100);
			  //USART_SendData(USART1, GY);
			 //delay_ms(100);
			 // USART_SendData(USART1, GZ);
			  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
			  //USART_ClearFlag(USART1,ENABLE);
	   }
}


