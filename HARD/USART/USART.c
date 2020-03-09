#include "USART.h"
#include "ADC.h"
#include "delay.h"
#include "MPU6050.h"

void USART_init(void)           //���ڳ�ʼ��
{
	   GPIO_InitTypeDef  Usart_IOinit;                        //����IO��
	   USART_InitTypeDef  Usart_init;                         //���ô���
	   NVIC_InitTypeDef  NVIC_init;                           //�����ж�
	
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ�ܶ˿�Aʱ��
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //ʹ�ܴ���1ʱ��
	
	   Usart_IOinit.GPIO_Pin=GPIO_Pin_9;                      //����GPIOA9  TX
	   Usart_IOinit.GPIO_Speed=GPIO_Speed_50MHz;
	   Usart_IOinit.GPIO_Mode=GPIO_Mode_AF_PP;
	   GPIO_Init(GPIOA,&Usart_IOinit);
	
	   Usart_IOinit.GPIO_Pin=GPIO_Pin_10;                     //����GPIOA10  RX
	   Usart_IOinit.GPIO_Speed=GPIO_Speed_50MHz;
	   Usart_IOinit.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	   GPIO_Init(GPIOA,&Usart_IOinit);
	
	   Usart_init.USART_BaudRate=9600;                        //���ô���
	   Usart_init.USART_WordLength=USART_HardwareFlowControl_None;
	   Usart_init.USART_StopBits=USART_StopBits_1;
	   Usart_init.USART_Parity=USART_Parity_No;
	   Usart_init.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	   Usart_init.USART_HardwareFlowControl=USART_WordLength_8b;
	   USART_Init(USART1,&Usart_init);
	   
	   USART_Cmd(USART1,ENABLE);                             //ʹ�ܴ���1
	   
	   USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);          //ѡ�񴮿��ж�
	   
	   NVIC_init.NVIC_IRQChannel=USART1_IRQn;                //�����жϺ���
	   NVIC_init.NVIC_IRQChannelCmd=ENABLE;
	   NVIC_init.NVIC_IRQChannelPreemptionPriority=1;
	   NVIC_init.NVIC_IRQChannelSubPriority=1;
	   NVIC_Init(&NVIC_init);
}

//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_data(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
	USART_SendData(USART1,c);  
} 

void usart1_niming_send(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0X88;	//֡ͷ
	send_buf[1]=fun;	//������
	send_buf[2]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
	for(i=0;i<len+4;i++)usart1_send_data(send_buf[i]);	//�������ݵ�����1 
}

//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart1_send(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//��0
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
	usart1_niming_send(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
}

void USART1_IRQHandler(void)                         //����1�жϺ���
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


