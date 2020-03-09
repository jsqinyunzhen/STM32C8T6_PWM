#include "PWM.h"
#include "sys.h"

void PWM_init(u16 arr,u16 psc)                       //PWM配置
{
	  GPIO_InitTypeDef  GPIO_PWMinit;
	  TIM_TimeBaseInitTypeDef  TIM_PWMinit;
	  TIM_OCInitTypeDef  TIM_OCInit;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t TIM3_Period;
    
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	  //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
	
	  GPIO_PWMinit.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;                 //配置PWM的输出IO
	  GPIO_PWMinit.GPIO_Mode=GPIO_Mode_AF_PP;
	  GPIO_PWMinit.GPIO_Speed=GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA,&GPIO_PWMinit);
	  
	  GPIO_PWMinit.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1;                 //配置PWM的输出IO
	  GPIO_PWMinit.GPIO_Mode=GPIO_Mode_AF_PP;
	  GPIO_PWMinit.GPIO_Speed=GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB,&GPIO_PWMinit);

    TIM3_Period = (uint16_t)(SystemCoreClock /36/ (16*1000)) - 1;
    TIM_PWMinit.TIM_Period=TIM3_Period;
    TIM_PWMinit.TIM_Prescaler =36-1;//psc;
	  TIM_PWMinit.TIM_ClockDivision=TIM_CKD_DIV1;
	  TIM_PWMinit.TIM_CounterMode=TIM_CounterMode_Up;
    TIM_PWMinit.TIM_RepetitionCounter = 0;
	  TIM_TimeBaseInit(TIM3,&TIM_PWMinit);
		
	  TIM_OCInit.TIM_OCMode=TIM_OCMode_PWM1;
	  TIM_OCInit.TIM_OCPolarity=TIM_OCPolarity_High;
	  TIM_OCInit.TIM_OutputState=TIM_OutputState_Enable;
	  TIM_OC1Init(TIM3,&TIM_OCInit);
		
		TIM_OCInit.TIM_OCMode=TIM_OCMode_PWM2;
	  TIM_OCInit.TIM_OCPolarity=TIM_OCPolarity_High;
	  TIM_OCInit.TIM_OutputState=TIM_OutputState_Enable;
	  TIM_OC2Init(TIM3,&TIM_OCInit);
		
		TIM_OCInit.TIM_OCMode=TIM_OCMode_PWM2;
	  TIM_OCInit.TIM_OCPolarity=TIM_OCPolarity_High;
	  TIM_OCInit.TIM_OutputState=TIM_OutputState_Enable;
	  TIM_OC3Init(TIM3,&TIM_OCInit);
		
		TIM_OCInit.TIM_OCMode=TIM_OCMode_PWM2;
	  TIM_OCInit.TIM_OCPolarity=TIM_OCPolarity_High;
	  TIM_OCInit.TIM_OutputState=TIM_OutputState_Enable;
	  TIM_OC4Init(TIM3,&TIM_OCInit);
	  
		TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);         //使能预装载寄存器
		TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
		TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	  TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);        

#if 1
      /* Clear TIM1 update pending flag  清除TIM1溢出中断标志]  */
      TIM_ClearFlag(TIM3, TIM_FLAG_Update);
      //TIM_ITConfig(TIM3,TIM_IT_Update, ENABLE);//打开TIM3
      //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
      NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
#endif

      pcm_data_tim3_reable();

      
	  //TIM_Cmd(TIM3,ENABLE);
}



