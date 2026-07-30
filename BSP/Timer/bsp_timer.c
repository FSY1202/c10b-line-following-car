#include "bsp_timer.h"


u16 timer_delay_cnt = 0;

void my_delay_10ms(u16 time)
{
	timer_delay_cnt = time;
	while(timer_delay_cnt != 0);
}

/**************************************************************************
�������ܣ�TIM6��ʼ������ʱ10���� 1ms��ʱ�� function: TIM6 initialization, timing 10 milliseconds 1ms timer
��ڲ������� input parameter: none
����  ֵ���� output value: none
**************************************************************************/
void TIM6_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //ʹ�ܶ�ʱ����ʱ��
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;			 // Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_Period = 9;				 //�趨�������Զ���װֵ
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);                //���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;			  //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);							  //��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM6, ENABLE);
}

u8 timer_1ms = 0;
volatile uint32_t g_system_tick_ms = 0;
// TIM6�ж� TIM6 interrupt
void TIM6_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //���TIM�����жϷ������ Check whether TIM update interrupt occurs
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);    //���TIMx�����жϱ�־ Clear TIMx update interrupt flag

		if(timer_delay_cnt != 0)
		{
			timer_delay_cnt --;
		}
        timer_1ms++;
        g_system_tick_ms++;

       
        if(timer_1ms%10==0)//10ms��ʱ�� 10ms timer
        { 
            Encoder_Update_Count();
            Motion_Handle();
        }
        
        if(timer_1ms>200)
        {
            timer_1ms = 0;
        }
        			
	}
    
}


/**************************************************************************
�������ܣ�TIM7��ʼ������ʱ10us
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM7_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //ʹ�ܶ�ʱ����ʱ��
	TIM_TimeBaseStructure.TIM_Prescaler = 71;			 // Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_Period = 9;				 //�趨�������Զ���װֵ
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);               //���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;			  //TIM�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  //�����ȼ�1��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);							  //��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM7, ENABLE);
}


// TIM7�ж�
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) //���TIM�����жϷ������
	{
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);    //���TIMx�����жϱ�־	
	}
    
}



