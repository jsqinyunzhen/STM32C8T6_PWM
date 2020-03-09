#include "MPU6050.h"
#include "delay.h"
#include "sys.h"
#include "USART.h"

void MPU_IIC_Init(void)            //��ʼ��IIC��IO��	
{
     GPIO_InitTypeDef GPIO_InitStructure;
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��GPIOBʱ��
	   
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_Init(GPIOB, &GPIO_InitStructure);
	   GPIO_SetBits(GPIOB,GPIO_Pin_9|GPIO_Pin_10); 	//PB8,PB9 �����
}

void MPU_IIC_Start(void)//����IIC��ʼ�ź�
{
		 MPU_SDA_OUT();     //sda�����
		 MPU_IIC_SDA=1;	  	  
		 MPU_IIC_SCL=1;
		 delay_us(4);
		 MPU_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
		 delay_us(4);
		 MPU_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

void MPU_IIC_Stop(void)//����IICֹͣ�ź�
{
		 MPU_SDA_OUT();//sda�����
		 MPU_IIC_SCL=0;
		 MPU_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	   delay_us(4);
		 MPU_IIC_SCL=1;
		 delay_us(4); 
		 MPU_IIC_SDA=1;//����I2C���߽����ź�						   	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 MPU_IIC_Wait_Ack(void)
{
	   u8 waitTime=0;
	   MPU_SDA_IN();      //SDA����Ϊ����  
	   //IIC_SDA=1;delay_us(1);	   
	   MPU_IIC_SCL=1;
	   delay_us(1);	 
	   while(MPU_READ_SDA)
	   {
		       waitTime++;
		       if(waitTime>250)
		       {
			             MPU_IIC_Stop();
			             return 1;
		       }
	   }
	   MPU_IIC_SCL=0;//ʱ�����0 	   
	   return 0;  
} 

//����ACKӦ��
void MPU_IIC_Ack(void)
{
	   MPU_IIC_SCL=0;
	   MPU_SDA_OUT();
	   MPU_IIC_SDA=0;
	   delay_us(2);
	   MPU_IIC_SCL=1;
	   delay_us(2);
	   MPU_IIC_SCL=0;
}

//������ACKӦ��		    
void MPU_IIC_NAck(void)
{
	   MPU_IIC_SCL=0;
	   MPU_SDA_OUT();
	   MPU_IIC_SDA=1;
	   delay_us(2);
	   MPU_IIC_SCL=1;
	   delay_us(2);
	   MPU_IIC_SCL=0;
}

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
     u8 t;   
	   MPU_SDA_OUT(); 	    
     MPU_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
     for(t=0;t<8;t++)
     {              
            MPU_IIC_SDA=(txd&0x80)>>7;
            txd<<=1; 	  
		        delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		        MPU_IIC_SCL=1;
		        delay_us(2); 
		        MPU_IIC_SCL=0;	
		        delay_us(2);
     }	 
}

u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	   unsigned char i,read=0;
	   MPU_SDA_IN();//SDA����Ϊ����
     for(i=0;i<8;i++ )
	   {
            MPU_IIC_SCL=0; 
            delay_us(2);
		        MPU_IIC_SCL=1;
            read<<=1;
            if(MPU_READ_SDA)read++;   
            delay_us(1); 
     }					 
     if (!ack)
            MPU_IIC_NAck();//����nACK
     else
            MPU_IIC_Ack(); //����ACK   
            return read;
}

//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 addr,u8 data) 				 
{ 
      MPU_IIC_Start(); 
	    MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	    if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	    {
		        MPU_IIC_Stop();		 
		        return 1;		
	    }
      MPU_IIC_Send_Byte(addr);	//д�Ĵ�����ַ
      MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	    MPU_IIC_Send_Byte(data);//��������
	    if(MPU_IIC_Wait_Ack())	//�ȴ�ACK
	    {
		        MPU_IIC_Stop();	 
		        return 1;		 
	    }		 
      MPU_IIC_Stop();	 
	    return 0;
}

//IIC��һ���ֽ� 
//addr:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte(u8 addr)
{
	     u8 data;
       MPU_IIC_Start(); 
	     MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	     MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
       MPU_IIC_Send_Byte(addr);	//д�Ĵ�����ַ
       MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
       MPU_IIC_Start();
	     MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������	
       MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	     data=MPU_IIC_Read_Byte(0);//��ȡ����,����nACK 
       MPU_IIC_Stop();			//����һ��ֹͣ���� 
	     return data;		
}

//����MPU6050�����Ǵ����������̷�Χ
//num:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_TLY_LCH(u8 num)
{
	     return MPU_Write_Byte(MPU_TLY,num<<3);//���������������̷�Χ  
}

//����MPU6050���ٶȴ����������̷�Χ
//num:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_SENSOR_LCH(u8 num)
{
	     return MPU_Write_Byte(MPU_SENSOR,num<<3);//���ü��ٶȴ����������̷�Χ  
}

//����MPU6050�����ֵ�ͨ�˲���
//dtlb:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_DTLB(u16 dtlb)
{
	    u8 data=0;
	    if(dtlb>=188)      data=1;
	    else if(dtlb>=98)  data=2;
	    else if(dtlb>=42)  data=3;
	    else if(dtlb>=20)  data=4;
	    else if(dtlb>=10)  data=5;
	    else data=6; 
	    return MPU_Write_Byte(MPU_DIRVE,data);//�������ֵ�ͨ�˲���  
}

//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//cyl:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 cyl)
{
	    u8 data;
	    if(cyl>1000)cyl=1000;
	    if(cyl<4)cyl=4;
	    data=1000/cyl-1;
	    data=MPU_Write_Byte(MPU_CYFBL,data);	//�������ֵ�ͨ�˲���
 	    return MPU_Set_DTLB(cyl/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temp(void)
{
      u8 buf[2]; 
      short raw;
	    float temp;
	    MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH,2,buf); 
      raw=((u16)buf[0]<<8)|buf[1];  
      temp=36.53+((double)raw)/340;  
      return temp*100;;
}

//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_TLY(short *gx,short *gy,short *gz)
{
      u8 buf[6],res;  
	    res=MPU_Read_Len(MPU_ADDR,MPU_TLY_XOUTH,6,buf);
	    if(res==0)
	    {
		  *gx=((u16)buf[0]<<8)|buf[1];  
		  *gy=((u16)buf[2]<<8)|buf[3];  
		  *gz=((u16)buf[4]<<8)|buf[5];
	    } 	
      return res;;
}

//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Sensor(short *ax,short *ay,short *az)
{
      u8 buf[6],res;  
	    res=MPU_Read_Len(MPU_ADDR,MPU_SENSOR_XOUTH,6,buf);
	    if(res==0)
	    {
		  *ax=((u16)buf[0]<<8)|buf[1];  
		  *ay=((u16)buf[2]<<8)|buf[3];  
		  *az=((u16)buf[4]<<8)|buf[5];
	    } 	
      return res;;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	    u8 i; 
      MPU_IIC_Start(); 
	    MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	    if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	    {
		         MPU_IIC_Stop();		 
		         return 1;		
	    }
      MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
      MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
	    for(i=0;i<len;i++)
	    {
		         MPU_IIC_Send_Byte(buf[i]);	//��������
		         if(MPU_IIC_Wait_Ack())		//�ȴ�ACK
		         {
			              MPU_IIC_Stop();	 
			              return 1;		 
		         }		
	    }    
      MPU_IIC_Stop();	 
	    return 0;	
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	     MPU_IIC_Start(); 
	     MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	     if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	     {
		           MPU_IIC_Stop();		 
		           return 1;		
	     }
       MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
       MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
       MPU_IIC_Start();
	     MPU_IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
       MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	     while(len)
	     {
		          if(len==1)*buf=MPU_IIC_Read_Byte(0);//������,����nACK 
		          else *buf=MPU_IIC_Read_Byte(1);		//������,����ACK  
		          len--;
		          buf++; 
	     }    
       MPU_IIC_Stop();	//����һ��ֹͣ���� 
	     return 0;	
}

u8 MPU_INIT(void)
{
	     u8 res;
	     MPU_IIC_Init();
	     MPU_Write_Byte(MPU_POWER1,0X80);	//��λMPU6050
	     delay_ms(100);
	     MPU_Write_Byte(MPU_POWER1,0X00);	//����MPU6050
	     MPU_Set_TLY_LCH(3);					//�����Ǵ�����,��2000dps
	     MPU_Set_SENSOR_LCH(0);					//���ٶȴ�����,��2g
	     MPU_Set_Rate(50);						//���ò�����50Hz
	     MPU_Write_Byte(MPU_INT_EN,0X00);	//�ر������ж�
	     MPU_Write_Byte(MPU_USER_CTRL,0X00);	//I2C��ģʽ�ر�
	     MPU_Write_Byte(MPU_FIFO_EN,0X00);	//�ر�FIFO
	     MPU_Write_Byte(MPU_INTBP_CFG,0X80);	//INT���ŵ͵�ƽ��Ч
	     res=MPU_Read_Byte(MPU_DEVICE_ID);
	     if(res==MPU_ADDR)  //����ID��ȷ
	     {
				      MPU_Write_Byte(MPU_POWER1,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
				      MPU_Write_Byte(MPU_POWER2,0X00);	//���ٶ��������Ƕ�����
				      MPU_Set_Rate(50);						//���ò�����Ϊ50Hz
			 }
	     else return 1;
	     return 0;
}

//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
void MPU6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	     u8 tbuf[12]; 
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
	     usart1_niming_send(0XA1,tbuf,12);//�Զ���֡,0XA1
}







