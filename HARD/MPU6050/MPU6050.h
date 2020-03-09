#ifndef _MPU6050_H
#define _MPU6050_H

#include "stm32f10x.h"

#define MPU_SDA_IN()   {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=(u32)8<<8;}
#define MPU_SDA_OUT()  {GPIOB->CRH&=0XFFFFF0FF;GPIOB->CRH|=(u32)3<<8;}

#define MPU_IIC_SCL    PBout(9) //SCL
#define MPU_IIC_SDA    PBout(10) //SDA	 
#define MPU_READ_SDA   PBin(10)  //����SDA 

#define  MPU_ADDR			 0X68

#define  MPU_POWER1    0X6B    //��Դ����Ĵ���1
#define  MPU_POWER2    0X6C    //��Դ����Ĵ���2
#define  MPU_TLY       0X1B    //������
#define  MPU_SENSOR    0X1C    //���ٶȴ�����
#define  MPU_FIFO_EN   0X23
#define  MPU_CYFBL     0X19    //�����ֱ���
#define  MPU_DIRVE     0X1A

#define MPU_TLY_XOUTH		0X43	//������ֵ,X���8λ�Ĵ���
#define MPU_TLY_XOUTL		0X44	//������ֵ,X���8λ�Ĵ���
#define MPU_TLY_YOUTH		0X45	//������ֵ,Y���8λ�Ĵ���
#define MPU_TLY_YOUTL		0X46	//������ֵ,Y���8λ�Ĵ���
#define MPU_TLY_ZOUTH		0X47	//������ֵ,Z���8λ�Ĵ���
#define MPU_TLY_ZOUTL		0X48	//������ֵ,Z���8λ�Ĵ���

#define MPU_SENSOR_XOUTH		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_SENSOR_XOUTL		0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_SENSOR_YOUTH		0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_SENSOR_YOUTL		0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_SENSOR_ZOUTH		0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU_SENSOR_ZOUTL		0X40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define MPU_TEMP_OUTH		0X41	//�¶�ֵ�߰�λ�Ĵ���
#define MPU_TEMP_OUTL		0X42	//�¶�ֵ��8λ�Ĵ���

#define MPU_INT_EN			0X38	//�ж�ʹ�ܼĴ���
#define MPU_USER_CTRL		0X6A	//�û����ƼĴ���
#define MPU_INTBP_CFG		0X37	//�ж�/��·���üĴ���
#define MPU_DEVICE_ID		0X75	//����ID�Ĵ���

void MPU_IIC_Init(void);
void MPU_IIC_Start(void);
void MPU_IIC_Stop(void);
u8 MPU_IIC_Wait_Ack(void);
void MPU_IIC_Ack(void);
void MPU_IIC_NAck(void);
void MPU_IIC_Send_Byte(u8 txd);
u8 MPU_IIC_Read_Byte(unsigned char ack);
u8 MPU_INIT(void);
u8 MPU_Write_Byte(u8 addr,u8 data);
u8 MPU_Read_Byte(u8 addr);
u8 MPU_Set_TLY_LCH(u8 num);
u8 MPU_Set_SENSOR_LCH(u8 num);
u8 MPU_Set_Rate(u16 cyl);
short MPU_Get_Temp(void);
u8 MPU_Get_TLY(short *gx,short *gy,short *gz);
u8 MPU_Get_Sensor(short *ax,short *ay,short *az);
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);
void MPU6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz);





#endif



