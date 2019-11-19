#ifndef __KEY_H__
#define __KEY_H__
#define KEY_ON GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)
#define SW_ON  GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define SW_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_6)
void key_init(void);
void key_on(void);
void key_off(void);
#endif
