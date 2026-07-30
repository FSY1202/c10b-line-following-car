#include "bsp_encoder.h"

int g_Encoder_L_Now = 0;
int g_Encoder_R_Now = 0;

// 左轮编码器: TIM8 CH1/CH2 = PC6/PC7 (默认映射, 无需重映射)
static void encode_L_TIM8(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(ENC_L_A_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = ENC_L_A_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENC_L_A_PORT, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(ENC_L_B_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = ENC_L_B_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENC_L_B_PORT, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM8, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM8, TIM_FLAG_Update);
    TIM_SetCounter(TIM8, 0x7fff);
    TIM_Cmd(TIM8, ENABLE);
}

// 右轮编码器: TIM4 CH1/CH2 = PB6/PB7 (默认映射, 无需重映射)
static void encode_R_TIM4(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(ENC_R_A_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = ENC_R_A_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENC_R_A_PORT, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(ENC_R_B_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = ENC_R_B_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENC_R_B_PORT, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_SetCounter(TIM4, 0x7fff);
    TIM_Cmd(TIM4, ENABLE);
}

void motor_encode_init(void)
{
    encode_L_TIM8();
    encode_R_TIM4();
}

/**
 * @brief 读取编码器计数增量(相对上次读取), 读完自动复位计数器到0x7fff中点
 * @note  正负号是否对应"前进"需要上电后转动车轮实测确认, 如果符号相反直接在这里取负即可
 */
static int16_t Encoder_Read_CNT(uint8_t Motor_id)
{
    int16_t Encoder_TIM = 0;
    switch (Motor_id)
    {
    case Motor_L:
        Encoder_TIM = 0x7fff - (short)TIM8->CNT;
        TIM8->CNT = 0x7fff;
        break;
    case Motor_R:
        Encoder_TIM = 0x7fff - (short)TIM4->CNT;
        TIM4->CNT = 0x7fff;
        break;
    default:
        break;
    }
    return Encoder_TIM;
}

// 返回控制周期内累计的编码器计数(左轮/右轮各1路)
int Encoder_Get_Count_Now(uint8_t Motor_id)
{
    if (Motor_id == Motor_L) return g_Encoder_L_Now;
    if (Motor_id == Motor_R) return g_Encoder_R_Now;
    return 0;
}

void Encoder_Get_ALL(int* Encoder_all)
{
    Encoder_all[Motor_L] = g_Encoder_L_Now;
    Encoder_all[Motor_R] = g_Encoder_R_Now;
}

// 更新编码器计数值, 由10ms定时器周期调用
void Encoder_Update_Count(void)
{
    g_Encoder_L_Now += Encoder_Read_CNT(Motor_L);
    g_Encoder_R_Now -= Encoder_Read_CNT(Motor_R);
}
