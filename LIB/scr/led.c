#include"stm32f10x_gpio.h"
#include"led.h"

//led1 - pa5
static void led1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

//led2 - pc13
//led3 - pc14
//led4 - pc15
static void led234_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
}

//led5 - pb2
static void led5_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void led_init(void)
{
	led1_init();
	led234_init();
	led5_init();
}

int led_on(unsigned char led)
{
	if(led<1&&led>5){
		return 1;
	}
	switch (led){
		case 1: GPIO_ResetBits(GPIOA,GPIO_Pin_5); break;
		case 2: GPIO_ResetBits(GPIOC,GPIO_Pin_13);break;
		case 3: GPIO_ResetBits(GPIOC,GPIO_Pin_14);break;
		case 4: GPIO_ResetBits(GPIOC,GPIO_Pin_15);break;
		case 5: GPIO_ResetBits(GPIOB,GPIO_Pin_2); break;
	}
	return 0;
}

int led_off(unsigned char led)
{
	if(led<1&&led>5){
		return 1;
	}
	switch (led){
		case 1: GPIO_SetBits(GPIOA,GPIO_Pin_5);  break;
		case 2: GPIO_SetBits(GPIOC,GPIO_Pin_13); break;
		case 3: GPIO_SetBits(GPIOC,GPIO_Pin_14); break;
		case 4: GPIO_SetBits(GPIOC,GPIO_Pin_15); break;
		case 5: GPIO_SetBits(GPIOB,GPIO_Pin_2);  break;
	}
	return 0;
}
