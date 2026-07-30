#include "grayscale_sensor.h"

static void _delay_us(volatile uint32_t us)
{
    delay_us(us);
}

// 选择传感器通道 (CD4051地址线 AD0/AD1/AD2)
static void _select_channel(uint8_t channel)
{
    SENSOR_AD0_WRITE((channel >> 0) & 0x01);  // bit0 -> AD0
    SENSOR_AD1_WRITE((channel >> 1) & 0x01);  // bit1 -> AD1
    SENSOR_AD2_WRITE((channel >> 2) & 0x01);  // bit2 -> AD2
}

// 读取OUT引脚的值
static uint16_t Read_OUT_value(void)
{
    return SENSOR_OUT_READ();
}

// 初始化灰度传感器所需的GPIO(C10B闲置IO: PA2/PA3/PA8作地址选通输出, PA11作数据输入)
void Grayscale_Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // AD0(PA2) AD1(PA3) AD2(PA8) 推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // OUT(PA11) 浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 读取所有8个通道的数字量 (X1..X8 依次对应 channel 0..7)
void Grayscale_Sensor_Read_All(uint16_t* sensor_values)
{
    uint8_t i;
    for (i = 0; i < GRAYSCALE_SENSOR_CHANNELS; i++)
    {
        _select_channel(i);
        _delay_us(50);
        sensor_values[i] = Read_OUT_value();
    }
}

// 读取单个指定通道
uint16_t Grayscale_Sensor_Read_Single(uint8_t channel)
{
    if (channel >= GRAYSCALE_SENSOR_CHANNELS)
    {
        return 0;
    }
    _select_channel(channel);
    _delay_us(50);
    return Read_OUT_value();
}
