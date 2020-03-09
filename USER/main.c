#include "stm32f10x.h"
#include "USART.h"
#include "SYS.h"
#include "delay.h"
#include "WORK.h"
#include "PWM.h"
#include "ADC.h"
#include "MPU6050.h"
#include "inv_mpu.h"

int main()
{    
	   u16 ch1,ch2,ch3,ch4;
	   //u8 pitch1,pitch2,roll1,roll2,yaw1,yaw2; 		//欧拉角
	   //float pitch,roll,yaw;
	   //short aacx,aacy,aacz;		//加速度传感器原始数据
	   //short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	   //TIME4_init(4999,7199);
	   TIME4_PWM_init(132000); 
	   Adc_Init();
			//LED_IO_INIT();
	   delay_init();
	   //USART_init();
	   //PBout(8)=0;	
	   //mpu_dmp_init();
	   //MPU_INIT();
	   //PWM_init(1500,999);//48的频率， 72000000 /1000/1500 =48
     PWM_init(1500,999);
       
while(1)
     {
		 //mpu_dmp_get_data(float pitch,float roll,float yaw);
		 //u8 data1,data2;
		 //if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)/7
		 //{MPU_Get_Sensor(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		 //MPU_Get_TLY(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
	   //MPU6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
	   //usart1_send(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
		 //}
		 //USART_SendData(USART1,mpu_dmp_get_data(&pitch,&roll,&yaw) );
		 ch1=ch2=ch3=ch4=Get_Adc_Average(1,10)/5+60;
		 //ch1=USART_ReceiveData(USART1);
		 TIM_SetCompare1(TIM3,62);
		 TIM_SetCompare2(TIM3,ch2);
		 TIM_SetCompare3(TIM3,ch3);
		 TIM_SetCompare4(TIM3,ch4);
     //if(ch1>900)ch1=500;			 
		 //delay_ms(1000);
		 //USART_SendData(USART1,ch1);
		 //delay_ms(1000);
		 //ch1=ch1+90;
		 //if(ch1==900)ch1=540;
			//pitch1 = *pitch >> 8;
			//pitch2 = *pitch;
		  //USART_SendData(USART1,pitch1 );
			//delay_ms(100);
			//USART_SendData(USART1,pitch2 );
			//delay_ms(1000);  
		}			
}
 

