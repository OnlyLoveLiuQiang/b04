#include"bmp280.h"
#include"stm32f10x_gpio.h"
#include"stm32f10x_spi.h"
#include"delay.h"

short dig_t2,dig_t3,dig_p2,dig_p3,dig_p4,dig_p5,dig_p6,dig_p7,dig_p8,dig_p9;
unsigned short dig_t1,dig_p1;
long t_fine;

void spi2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	//PA8(BMP280片选) -  推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//PB13(SCK) - 推挽复用输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//PB14(MISO) - 浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//PB15(MOSI) - 推挽复用输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
}

/***************************************************************/
//SPIx 
//TxData:发送一个字节
//返回值:data
/***************************************************************/
static unsigned char SPI2_ReadWriteByte(unsigned char TxData)
{		
	unsigned char retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //等待SPI发送标志位空
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI2, TxData); //发送数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //等待SPI接收标志位空
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI2); //接收数据					    
}

unsigned char bmp280_write(unsigned char addr,unsigned char value)
{
	unsigned char status = 0;
	BMP280_ENABLE;
	status = SPI2_ReadWriteByte(addr);
	SPI2_ReadWriteByte(value);
	BMP280_DISENABLE;
	return status;
}

unsigned char bmp280_read(unsigned char addr)
{
	unsigned char val = 0;
	BMP280_ENABLE;
	SPI2_ReadWriteByte(addr|0x80);
	val = SPI2_ReadWriteByte(0xff);
	BMP280_DISENABLE;
	return val;
}

short bmp280_multiplereadtwo(unsigned char addr)
{
	unsigned char msb,lsb;
	short temp = 0;
	lsb = bmp280_read(addr);
	msb = bmp280_read(addr+1);
	temp = (short)msb<<8;
	temp |= lsb;
	
	return temp;
}

long bmp280_multiplereadthree(unsigned char addr)
{
	unsigned char msb,lsb,xlsb;
	long temp = 0;
	msb = bmp280_read(addr);
	lsb = bmp280_read(addr+1);
	xlsb= bmp280_read(addr+2);
	
	temp = (long)( ((unsigned long)msb<<12) | ((unsigned long)lsb<<4) | ((unsigned long)xlsb>>4) );
	return temp;
}

static unsigned char bmp280_data_read(unsigned char addr,unsigned char *buf)
{
	BMP280_ENABLE;
	SPI2_ReadWriteByte(addr|0x80);
	buf[0] = SPI2_ReadWriteByte(0xff);
	buf[1] = SPI2_ReadWriteByte(0xff);
	buf[2] = SPI2_ReadWriteByte(0xff);
	BMP280_DISENABLE;
	return 0;
}

void bmp280_init(void)
{
	//开机复位(写入0xB6进行复位)
	bmp280_write(BMP280_RESET,0xB6);
	//设置ctrl_meas为0xff(温度、压强采样20bit、电源正常模式)
	bmp280_write(CTRL_MEAS,0xff);
	bmp280_write(0xF5,0x00);
	delay_ms(200);
	
	dig_t1 = bmp280_multiplereadtwo(0x88);
	dig_t2 = bmp280_multiplereadtwo(0x8A);
	dig_t3 = bmp280_multiplereadtwo(0x8C);
	dig_p1 = bmp280_multiplereadtwo(0x8E);
	dig_p2 = bmp280_multiplereadtwo(0x90);
	dig_p3 = bmp280_multiplereadtwo(0x92);
	dig_p4 = bmp280_multiplereadtwo(0x94);
	dig_p5 = bmp280_multiplereadtwo(0x96);
	dig_p6 = bmp280_multiplereadtwo(0x98);
	dig_p7 = bmp280_multiplereadtwo(0x9A);
	dig_p8 = bmp280_multiplereadtwo(0x9C);
	dig_p9 = bmp280_multiplereadtwo(0x9E);
	
	delay_ms(200);
}

void read_bmp280_temp(unsigned char *buf)
{
	bmp280_data_read(0xFA,buf);
}

void read_bmp280_press(unsigned char *buf)
{
	bmp280_data_read(0xF7,buf);
}

long bmp280_gettemperature(void)
{
	long adc_t;
	long var1,var2,T;
	
	adc_t = bmp280_multiplereadthree(0xFA);
	var1 = ((((adc_t>>3) - ((long)dig_t1<<1))) * ((long)dig_t2)) >> 11;
	var2 = (((((adc_t>>4) - ((long)dig_t1)) * ((adc_t>>4) - ((long)dig_t1))) >> 12) * ((long)dig_t3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

unsigned long bmp280_getcompensate(void)
{
	long adc_p;
	long var1,var2;
	unsigned long p;
	
	adc_p = bmp280_multiplereadthree(0xF7);
	var1 = (((long)t_fine)>>1) - (long)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11) * ((long)dig_p6);
	var2 = var2 + ((var1 * ((long)dig_p5))<<1);
	var2 = (var2>>2)+(((long)dig_p4)<<16);
	var1 = (((dig_p3 * (((var1>>2) * (var1>>2)) >> 13)) >>3) + ((((long)dig_p2) * var1) >>1)) >> 18;
	var1 = ((((32768+var1))*((long)dig_p1))>>15);
	if(var1 == 0){
		return 0;
	}
	p = (((unsigned long)(((long)1048576)-adc_p)-(var2>>12)))*3125;
	if(p < 0x80000000){
		p = (p << 1)/((unsigned long)var1);
	}else{
		p = (p/(unsigned long)var1) * 2;
	}
	var1 = (((long)dig_p9) * ((long)(((p>>3) * (p>>3))>>13))) >> 12;
	var2 = (((long)(p>>2)) * ((long)dig_p8)) >> 13;
	p = (unsigned long)((long)p + ((var1 + var2 + dig_p7) >> 4));
	return p;
}
