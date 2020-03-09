#include "WORK.h"
#include "SYS.h"
#include "delay.h"

void LED_IO_INIT(void)
{
    GPIO_InitTypeDef  GPIO_LEDinit;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC  | RCC_APB2Periph_AFIO, ENABLE);
	
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); 
	
	  GPIO_LEDinit.GPIO_Pin=GPIO_Pin_2;
	  GPIO_LEDinit.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_LEDinit.GPIO_Mode=GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOA,&GPIO_LEDinit);
	
	  GPIO_LEDinit.GPIO_Pin=GPIO_Pin_13;
	  GPIO_LEDinit.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_LEDinit.GPIO_Mode=GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOA,&GPIO_LEDinit);
	  
	  GPIO_LEDinit.GPIO_Pin=GPIO_Pin_12;
	  GPIO_LEDinit.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_LEDinit.GPIO_Mode=GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOB,&GPIO_LEDinit);
	
	  GPIO_LEDinit.GPIO_Pin=GPIO_Pin_8;
	  GPIO_LEDinit.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_LEDinit.GPIO_Mode=GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOB,&GPIO_LEDinit);
	  
	  GPIO_LEDinit.GPIO_Pin=GPIO_Pin_13;
	  GPIO_LEDinit.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_LEDinit.GPIO_Mode=GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOB,&GPIO_LEDinit);
		
		GPIO_LEDinit.GPIO_Pin=GPIO_Pin_13;
	  GPIO_LEDinit.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_LEDinit.GPIO_Mode=GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOC,&GPIO_LEDinit);
		
		GPIO_LEDinit.GPIO_Pin = GPIO_Pin_9;
	   GPIO_LEDinit.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	   GPIO_LEDinit.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_Init(GPIOB, &GPIO_LEDinit);
	
	   GPIO_LEDinit.GPIO_Pin = GPIO_Pin_10;
	   GPIO_LEDinit.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	   GPIO_LEDinit.GPIO_Speed = GPIO_Speed_50MHz;
	   GPIO_Init(GPIOB, &GPIO_LEDinit);
		
		PAout(2) = 1;
		PAout(13) = 1;
		PBout(12) = 1;
		PBout(13) = 1;
		PCout(13) = 1;
		PBout(8) = 0;
}

void TIME4_init(u16 arr,u16 psc)                      //定时器配置
{
	   TIM_TimeBaseInitTypeDef  TIM_init;
	   NVIC_InitTypeDef  NVIC_init;
	   
	   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	   
	   TIM_init.TIM_Period=arr;
	   TIM_init.TIM_Prescaler =psc;
	   TIM_init.TIM_ClockDivision=TIM_CKD_DIV1;
	   TIM_init.TIM_CounterMode=TIM_CounterMode_Up;
	   TIM_TimeBaseInit(TIM4,&TIM_init);
	
	   TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	
	   NVIC_init.NVIC_IRQChannel=TIM4_IRQn;
	   NVIC_init.NVIC_IRQChannelCmd=ENABLE;
	   NVIC_init.NVIC_IRQChannelPreemptionPriority=0;
	   NVIC_init.NVIC_IRQChannelSubPriority=0;
	   NVIC_Init(&NVIC_init);
	
	   TIM_Cmd(TIM4,ENABLE);
}
void TIME4_PWM_init(uint32_t fre)                      //定时器配置
{
        NVIC_InitTypeDef NVIC_InitStructure;
        TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
        TIM_OCInitTypeDef  TIM3_OCInitStructure;
        GPIO_InitTypeDef GPIO_InitStructure;
        uint16_t TIM4_Period;
        uint16_t TIM3_Period;
        uint32_t cur_time;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

        //TIM4_Period = (uint16_t)(SystemCoreClock /720/ fre) - 1;
        TIM4_Period = 255;
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//初始化与6的时钟
        TIM_DeInit(TIM4);   /*复位TIM1定时器*/
        
        TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
        TIM_TimeBaseStructure.TIM_Period = TIM4_Period;//
        //TIM_TimeBaseStructure.TIM_Prescaler = 72;//没有预分频
        //TIM_TimeBaseStructure.TIM_Period = 100;   /*时钟滴答的次数，够数中断这里是1ms中断一次*/     
        if(fre == 32000)
            TIM_TimeBaseStructure.TIM_Prescaler = (9-1);//720-1;    /* 分频720*/     
        else if(fre == 132000)
            TIM_TimeBaseStructure.TIM_Prescaler = 4;//720-1;    /* 分频720*/   
        else 
            TIM_TimeBaseStructure.TIM_Prescaler = (9-1);
        //TIM4_Period = (uint16_t)(SystemCoreClock /TIM_Prescaler/ fre) - 1;
        //TIM_Prescaler=SystemCoreClock/((TIM4_Period+1)*fre)=72000000/((255+1)*132000)=
        TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;//时钟不分频
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//增计数
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;                       //PB6复用为TIM4的通道1
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
      
        TIM3_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;              //PWM模式2 TIM3_CCMR1[14:12]=111 在向上计数时，
                                            //一旦TIMx_CNT<TIMx_CCR1时通道1为无效电平，否则为有效电平
        TIM3_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;    //输入/捕获2输出允许  OC2信号输出到对应的输出引脚PB5
        TIM3_OCInitStructure.TIM_Pulse = TIM4_Period/2;                         //确定占空比，这个值决定了有效电平的时间。
        TIM3_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;      //输出极性  低电平有效 TIM3_CCER[5]=1;
        TIM_OC1Init(TIM4, &TIM3_OCInitStructure);
        TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
        
    //  TIM_ARRPreloadConfig(TIM2,ENABLE);
        TIM_Cmd(TIM4, ENABLE);//打开TIM4

}

void TIM4_IRQHandler(void)                           //定时器中断函数
{
	   if(TIM_GetITStatus(TIM4,TIM_IT_Update)==1)
	   {
		     PBout(8)=!PBout(8);
			   //PBout(12)=!PBout(12);
			   //delay_ms(200);
			   //PBout(8)=1;
	   }
	   TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
}
void pcm_data_tim3_reable(void)
{
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);//打开TIM3
	TIM_Cmd(TIM3, ENABLE);//打开TIM3
}
void set_tim4_pwm_duty(uint8_t num)
{
	uint16_t ChannelxPulse;
	
	ChannelxPulse  = num;
	//printf("f=%s l=%d\r\n",__FILE__,__LINE__);

	//TIM_ARRPreloadConfig(TIMx,DISABLE);
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Disable);
	
	//TIMx->ARR=TIM2_Period;                //更新预装载值 
	TIM4->CCR1 =ChannelxPulse;//(ChannelxPulse<<4);
	//
	//TIM_ARRPreloadConfig(TIMx,ENABLE);
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);

 
}


u8 pcm_read_index=0;

void TIM3_IRQHandler(void)                           //定时器中断函数
{
	   if(TIM_GetITStatus(TIM3,TIM_IT_Update)==1)
	   {
        /* Clear the RTC Second interrupt */
        TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
#if 1
        if(pcm_read_index < 256)
        {
            set_tim4_pwm_duty(254);
            pcm_read_index ++;
        }
        else
        {
            pcm_read_index = 0;
            TIM_Cmd(TIM3, DISABLE);
            TIM_ITConfig(TIM3,TIM_IT_Update, DISABLE);
        }
#endif
    }
}




