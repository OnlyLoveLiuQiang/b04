#ifndef __BMP280_H__
#define __BMP280_H__
#define BMP280_DISENABLE  GPIOA->BSRR = GPIO_Pin_8;//Ƭѡ
#define BMP280_ENABLE  	  GPIOA->BRR = GPIO_Pin_8;

#define TEMP_XLSB	0xFC
#define TEMP_LSB	0xFB
#define TEMP_MSB	0xFA
#define PRESS_XLSB	0xF9
#define PRESS_LSB	0xF8
#define PRESS_MSB	0xF7
#define CONFIG		0xF5
#define CTRL_MEAS	0xF4
#define STATUS		0xF3
#define BMP280_RESET		0xE0
#define ID			0xD0

void spi2_init(void);
unsigned char bmp280_write(unsigned char addr,unsigned char value);
unsigned char bmp280_read(unsigned char addr);
short bmp280_multiplereadtwo(unsigned char addr);
long bmp280_multiplereadthree(unsigned char addr);
void bmp280_init(void);
void read_bmp280_temp(unsigned char *buf);
void read_bmp280_press(unsigned char *buf);
long bmp280_gettemperature(void);
unsigned long bmp280_getcompensate(void);
#endif
