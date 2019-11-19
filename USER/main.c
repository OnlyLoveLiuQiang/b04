#include"stm32f10x.h"
#include"key.h"
#include"exti.h"
#include"led.h"
#include"usart.h"
#include"bmp280.h"
#include"delay.h"

int main(){
	long temp = 0;
	unsigned long press = 0;
	unsigned char i = 0;
	unsigned char id = 0;
//	unsigned char temp[3] = "";
//	unsigned char press[3] = "";
	key_init();
	led_init();
//	led_on(1);
	key_on();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	exti0_init();
	delay_init();
	uart3_init(115200);
	spi2_init();
	bmp280_init();
	id = bmp280_read(ID);
	delay_ms(1000);
	USART_SendData(USART3,id);
	delay_ms(1000);
	while(1){
		temp = bmp280_gettemperature();
		press = bmp280_getcompensate();
		printf("temp = %ld press = %ld",temp,press);
		delay_ms(1000);
		delay_ms(1000);
	}
}
