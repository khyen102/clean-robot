#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile int flag = -1;
volatile int toggle = 0;
volatile int turn = 0;	
volatile int turn_ing=0;
volatile int turn_flag=0;	
volatile int next = 0;
volatile int move_flag = 0;
volatile int step_flag = 0;	
volatile short int DifAng, AngTemp, SumAng = 0;		//AngTemp 각도의 음수 양수 파악 , SumAng 180도 이상에서 동작하게 만듬
volatile int AngFlag = 1;
volatile int f_b_flag=0;
volatile int count = 0;
volatile char lcd_data[16] = { 0 };
int mode = 0; //로봇 부팅 시 모드를 변경하기 위한 변수 정의


/*****************************스위치 정의****************************/




/*****************************LED*****************************/
#define LED_DDR		DDRD
#define LED_CTRL	PORTD
#define LED			0x10

volatile int ledcnt = 0;
/*
ISR(TIMER2_OVF_vect) // 로봇이 동작되는 동안 LED가 0.5초마다 깜빡거림
{
ledcnt++;
if(ledcnt == 250)
{
LED_CTRL ^= LED;
ledcnt = 0;
}
TCNT2 = 131;	//2Hz
}
*/
void led_init() //LED 초기화
{
	LED_DDR = LED;
	TIMSK |= 0x40;
	TCCR2 |= 0x04;	//분주비 : 256, 1 / (16000000 / 256) = 16us
	TCNT2 = 131;	//125 * 16us = 2ms
}
/*****************************LED_END*****************************/
/*******************************LCD*******************************/
//LCD의 데이터 포트와 컨트롤 포트를 정의한다.
#define LCD_DATA		PORTC
#define LCD_CTRL		PORTB
#define LCD_DATA_DDR	DDRC
#define LCD_CTRL_DDR	DDRB

#define LCD_RS	0x20
#define LCD_RW	0x40
#define LCD_EN	0x80

void LCD_data(char data)
{
	LCD_DATA = data;
	LCD_CTRL |= LCD_RS | LCD_EN;
	_delay_us(1);

	LCD_CTRL &= ~LCD_EN;
	LCD_CTRL &= ~LCD_RS;
	_delay_us(50);
}

void LCD_command(char command)
{
	LCD_DATA = command;
	LCD_CTRL |= LCD_EN;
	_delay_us(1);

	LCD_CTRL &= ~LCD_EN;
	_delay_ms(5);
}

void LCD_STR(char *str)//ASCII문자열 출력
{
	while (*str)
		LCD_data(*str++);
}

void LCD_pos(int y, int x)
{
	LCD_command(0x80 | (x + y * 0x40));//1000xxxx=첫째줄, 1100xxxx=둘째줄
}

void LCD_clear(void)
{
	LCD_command(1);//00000001=clear
}

void LCD_init(void)//LCD 초기화
{
	LCD_DATA_DDR = 0xFF;
	LCD_CTRL_DDR |= 0xE0;//		0/1		0/1
	LCD_command(0x38);//0011NF00	N:1 or 2line	F:5X11 or 5X8
	LCD_command(0x0C);//000011CB	C:커서 on/off	B:깜빡임 on/off
	LCD_command(0x06);//increment mode, entire shift off

	LCD_clear();
}

void LCD_out(int y, int x, char *str)//세로 가로 데이터
{
	LCD_pos(y, x);
	LCD_STR(str);
}

void LCD_numout(int y, int x, int n)//세로 가로 숫자
{
	char str[16];
	LCD_pos(y, x);
	sprintf(str, "%4d", n);
	LCD_STR(str);
}
/*******************************LCD_END*******************************/
/*****************************ADC*****************************/
int get_ADC0() { ADMUX = 0x00; ADCSRA |= 0x40; /* 분주비 128 */ while ((ADCSRA & 0x10) == 0); return ADC; }
int get_ADC1() { ADMUX = 0x01; ADCSRA |= 0x40; /* 분주비 128 */ while ((ADCSRA & 0x10) == 0); return ADC; }
int get_ADC2() { ADMUX = 0x02; ADCSRA |= 0x40; /* 분주비 128 */ while ((ADCSRA & 0x10) == 0); return ADC; }

void ADC_init()//PSD센서 초기화 - ADC 사용
{
	ADMUX = 0x00;
	ADCSRA = 0x87;
}
/*****************************ADC_END*****************************/

//--------------------UART0--------------------블루투스

#define BT_ACCEPT UCSR0B = 0x98
#define BT_REJECT UCSR0B = 0x18

volatile int bt_count = 0;
volatile char bt_data[4] = { 0 };
volatile int X_axis = 0, Y_axis = 0;
char data_i = 0, x_toggle = 0;
char bt_data1, x_data[3] = { '1', '2', '5' }, y_data[3] = { '1', '2', '5' }, M = 0, T = 1;
int X_value = 125, Y_value = 125;

void serial0_init()//Baudrate 9600, 송수신 on
{
	UCSR0A = 0x00;	//
	UCSR0B = 0x98;	//10000000 : Rx complete  01000000 : Tx complete  00100000 : UDR Empty
	UCSR0C = 0x06;	//8bit
	UBRR0H = 0x00;
	UBRR0L = 103;
	//16MHz Baud	2400	4800	9600	14400	19200	28800	38400
	//		
}
char rx0_char(void)
{
	while (!(UCSR0A & 0x80));
	return UDR0;
}

void tx0_char(char tx_data)
{
	while (!(UCSR0A & 0x20));
	UDR0 = tx_data;
}

void tx0_string(char *str)
{
	while (*str)
		tx0_char(*str++);
}

void tx_Axis(unsigned int X, unsigned int Y)
{
	tx0_char((char)(X >> 8));
	tx0_char((char)(X % 0x100));
	tx0_char((char)(Y >> 8));
	tx0_char((char)(Y % 0x100));
}

ISR(USART0_RX_vect)
{
	if(mode == 0)
	{
		bt_data[bt_count] = UDR0;
		bt_count++;

		if(bt_count == 4)
		{
			bt_count = 0;
			flag = 0;
			X_axis = (((unsigned int)bt_data[0] << 8) | bt_data[1]) + 600;
			Y_axis = (((unsigned int)bt_data[2] << 8) | bt_data[3]) - 1070;
			BT_REJECT;
		}
	}

	else if(mode == 1)
	{	
		bt_data1 = UDR0;
	
		if(bt_data1 == 'X')
		{
			data_i = 0;
			bt_data1 = 0;
			x_toggle = 1;
		}
		else if(data_i < 3)
			x_data[data_i++] = bt_data1;
		else if(data_i < 6)
			y_data[data_i++ - 3] = bt_data1;
		else if(data_i < 7)
		{
			M = bt_data1 - 48;
			data_i++;
		}
		else if(data_i == 7)
		{
			X_value = (x_data[0] - 48) * 100 + (x_data[1] - 48) * 10 + x_data[2] - 48;
			Y_value = (y_data[0] - 48) * 100 + (y_data[1] - 48) * 10 + y_data[2] - 48;
			T = bt_data1 - 48;
			toggle = 0;
		}
	}
}
/***********************UART0 END*************************/


//--------------------UART1--------------------크루즈
void serial1_init()//Baudrate 115200, 송수신 on
{
	UCSR1A = 0x00;
	UCSR1B = 0x98;	//10000000 : Rx complete  01000000 : Tx complete  00100000 : UDR Empty
	UCSR1C = 0x06;
	UBRR1H = 0x00;
	UBRR1L = 8;
	//16MHz Baud	2400	4800	9600	14400	19200	28800	38400
	//				416		207		103		68		51		34		25
}
unsigned char rx1_char(void)
{
	while (!(UCSR1A & 0x80));
	return UDR1;
}
void tx1_char(char tx_data)
{
	while (!(UCSR1A & 0x20));
	UDR1 = tx_data;
}
void tx1_string(char *str)
{
	while (*str)
		tx1_char(*str++);
}
// volatile 뜻은 변덕스러움 변수가 마음대로 바뀔떄 그거 방지해주는 함수

volatile int cruiz_count = 0;
volatile signed short gRate = 0;
volatile short int gAngle = 0;
volatile short int rate;
volatile short int angle;
volatile short check_sum;
volatile char data_string[8] = { 0 };

ISR(USART1_RX_vect)  //앵글값 정의내려준거 
{
	data_string[cruiz_count] = UDR1;
	cruiz_count++;
	
	AngTemp = gAngle;
		
		if(AngFlag == 0)
		{
			SumAng = gAngle +359;
		}
		if(AngFlag == 1)
		{
			SumAng = gAngle;
		}
		if(AngFlag == 2)
		{
			SumAng = gAngle - 359;
		}

	if(cruiz_count == 8)
	{
		rate = (data_string[2] & 0xFF) | ((data_string[3] << 8) & 0xFF00);
		angle = (data_string[4] & 0xFF) | ((data_string[5] << 8) & 0XFF00);
		//Verify checksum
		check_sum = 0xFFFF + rate + angle;
		//Scale and store data
		gRate = rate / 100;
		gAngle = angle / 100;
		cruiz_count = 0;
	}
	
	DifAng = AngTemp - gAngle;
		if(DifAng >350)
		{
			AngFlag -= 1;
			//SumAng = gAngle + 360;
		}
		if(DifAng <-350)
		{
			AngFlag += 1;
			//SumAng = gAngle - 360;
		}
}


/*******************************UART_END******************************/

/*******************************DC_MOTOR******************************/
#define DC_DDR		DDRB
#define DC_CTRL		PORTB
#define DC		0x0F

#define DC_OPEN {DC_CTRL |= 0x01;} //중괄호 내의 조건 중 하나라도 만족하면 집게가 열린다.
#define DC_CLOSE {DC_CTRL |= 0x02; DC_CTRL &= ~0x01;} //중괄호 내의 조건 중 하나라도 만족하면 집게가 닫힌다.
#define DC_STOP {DC_CTRL &= ~0x03;} //중괄호 내의 조건을 만족하면 DC모터가 회전을 멈춘다.

void dc_init()
{
	DC_DDR |= DC;
}

void HAND_OPEN(void)
{
	DC_OPEN;
	if(SW_5)
	{
		DC_STOP;
		_delay_ms(200);
	}
}

void HAND_CLOSE(void)
{
	DC_CLOSE;
	if(SW_2 && SW_3)
	{
		DC_STOP;
		_delay_ms(200);
	}
}
/*****************************DC_MOTOR_END****************************/


#define STEEL_IN !(PINE & 0x04)		// STEEL    철 감지 센서의 동작을 정의한다. 철감지 센서는 ADC가 아닌 0과 1의 디지털 신호 형식으로 얻는다.


/******************************STEP_MOTOR*****************************/
#define STEP_DDR	DDRG
#define STEP_CTRL	PORTG
#define STEP_MOTOR	0x1F

//********************************************
//		PG4		PG3		PG2		PG1		PG0
//		ENA		R_CLK	L_CLK	R_DIR	L_DIR
//********************************************

#define STEP_L_DIR_BACK	STEP_CTRL |= 0x02	//모터의 CW, CCW 결정
#define STEP_L_DIR_FOR	STEP_CTRL &= ~0x02
#define STEP_R_DIR_BACK	STEP_CTRL &= ~0x01	//모터의 CW, CCW 결정
#define STEP_R_DIR_FOR	STEP_CTRL |= 0x01
#define STEP_EN_ON		STEP_CTRL |= 0x10	//모터 enable
#define STEP_EN_OFF		STEP_CTRL &= ~0x10	//모터 disable
#define STEP_R_CLK		0x08				//클럭으로 속도 조절
#define STEP_L_CLK		0x04				//클럭으로 속도 조절

//-------------STEP_MOTOR_SPEED_DEFINE-------------//
#define slow_speed 				15
#define normal_speed 			20
#define little_slow_speed		0
#define turn_speed				120			//351Hz
#define turn_speed2				80

volatile unsigned char axis_finish = 0;
volatile int X_count = 0, Y_count = 0;								//로봇의 현재 위치 좌표값을 저장하기 위한 변수
volatile unsigned int not_count = 0;
volatile unsigned int step_count = 0, not_step = 0;
volatile unsigned int STEP_right_speed = 0, STEP_left_speed = 0;	//로봇의 속도를 조절하기 위한 변수
volatile int finish = 0;											//로봇의 최종 단계를 수행하기 위한 변수
volatile int i;														//for 함수 사용할때 쓰는 변수
void STEP_motor_start(char L_speed, char R_speed, char L_dir, char R_dir);	//speed, dir : 0 ~ 1
void STEP_motor_stop();
void STEP_advance();
void STEP_speed(char R_speed, char L_speed);
//----------TIMER0_OVF----------//
void enable_TIMER0_ovf()
{
	TCCR0 |= 0x06;
	TIMSK |= 0x01;
}
void disable_TIMER0_ovf()
{
	TIMSK &= ~0x01;
	TCCR0 &= ~0x06;
}
//----------TIMER2_OVF----------//
void enable_TIMER2_ovf()
{
	TCCR2 |= 0x04;
	TIMSK |= 0x40;
}
void disable_TIMER2_ovf()
{
	TIMSK &= ~0x40;
	TCCR2 &= ~0x04;
}

ISR(TIMER0_OVF_vect)		//Right motor control
{
	TCNT0 = STEP_right_speed;
	STEP_CTRL ^= STEP_R_CLK;
	
	if(-44<=SumAng && SumAng<=44 && turn_ing==0)		
	{
		switch(f_b_flag)
		{
		case 0:
			Y_count++;
			break;
		case 1:
			Y_count--;
			break;
		}
	}
	else if(-134<=SumAng && SumAng<=-46 && turn_ing==0)
	{
		switch(f_b_flag)
		{
			case 0:
			X_count++;
			break;
			case 1:
			X_count--;
			break;
		}
	}
	else if(-224<=SumAng && SumAng<=-136 && turn_ing==0)
	{
		switch(f_b_flag)
		{
			case 0:
			Y_count--;
			break;
			case 1:
			Y_count++;
			break;
		}
	}
	else if(-314<=SumAng && SumAng<=-226 && turn_ing==0)
	{
		switch(f_b_flag)
		{
			case 0:
			X_count--;
			break;
			case 1:
			X_count++;
			break;
		}
	}
	else if(-316>=SumAng && SumAng>=-405 && turn_ing==0)
	{
		switch(f_b_flag)
		{
			case 0:
			Y_count++;
			break;
			case 1:
			Y_count--;
			break;
		}
	}
	else if(46<=SumAng && SumAng<=134 && turn_ing==0)
	{
		switch(f_b_flag)
		{
			case 0:
			X_count--;
			break;
			case 1:
			X_count++;
			break;
		}
	}
	else if(136<=SumAng && SumAng<=224 && turn_ing==0)
	{
		switch(f_b_flag)
		{
			case 0:
			Y_count--;
			break;
			case 1:
			Y_count++;
			break;
		}
	}
	else if(226<=SumAng && SumAng<=314 && turn_ing==0)
	{
		switch(f_b_flag)
		{
			case 0:
			X_count++;
			break;
			case 1:
			X_count--;
			break;
		}
	}
	else if(316<=SumAng && SumAng<=405 && turn_ing==0)
	{
		switch(f_b_flag)
		{
			case 0:
			Y_count++;
			break;
			case 1:
			Y_count--;
			break;
		}
	}
	else if(410<=SumAng && SumAng<=495 && turn_ing==0)
	{
		switch(f_b_flag)
		{
			case 0:
			X_count--;
			break;
			case 1:
			X_count++;
			break;
		}
	}
	
}

ISR(TIMER2_OVF_vect)		//Left motor control
{
	TCNT2 = STEP_left_speed;
	STEP_CTRL ^= STEP_L_CLK;
}

void STEP_motor_start(char R_speed, char L_speed, char R_dir, char L_dir)	//speed, dir : 0 ~ 1
{
	//----------스텝모터 설정----------//
	STEP_EN_ON;
	//-------바퀴 회전 방향 결정-------//
	switch (R_dir)
	{
	case 0:
		STEP_R_DIR_BACK;
		break;
	case 1:
		STEP_R_DIR_FOR;
		break;
	}
	switch (L_dir)
	{
	case 0:
		STEP_L_DIR_BACK;
		break;
	case 1:
		STEP_L_DIR_FOR;
		break;
	}
	//---------바퀴 속도 결정----------//
	STEP_speed(R_speed, L_speed);
	//---------------------------------//
	enable_TIMER0_ovf();
	enable_TIMER2_ovf();
	sei();
}

void STEP_speed(char R_speed, char L_speed)		//바퀴 속도 결정
{
	STEP_right_speed = R_speed;
	STEP_left_speed = L_speed;
}
void STEP_one_clock()
{
	STEP_CTRL ^= STEP_R_CLK | STEP_L_CLK;
	STEP_CTRL ^= STEP_R_CLK | STEP_L_CLK;
	STEP_CTRL ^= STEP_R_CLK | STEP_L_CLK;
	STEP_CTRL ^= STEP_R_CLK | STEP_L_CLK;
}
void STEP_motor_stop()
{
	disable_TIMER0_ovf();
	disable_TIMER2_ovf();
}
void STEP_advance() 
{ 
	STEP_speed(normal_speed, normal_speed); 
	STEP_L_DIR_FOR; 
	STEP_R_DIR_FOR; 
}
void STEP_slow_advance() 
{ 
	STEP_speed(slow_speed, slow_speed); 
	STEP_L_DIR_FOR; 
	STEP_R_DIR_FOR; 
}
void STEP_back() 
{
	 STEP_speed(normal_speed, normal_speed); 
	 STEP_L_DIR_BACK; 
	 STEP_R_DIR_BACK; 
}
void STEP_stop() 
{
	 STEP_motor_stop();
	 toggle=0; 
}
void STEP_right_revision() 
{ 
	STEP_speed(little_slow_speed, normal_speed); 
	STEP_L_DIR_FOR; 
	STEP_R_DIR_FOR; 
}
void STEP_left_revision() 
{ 
	STEP_speed(normal_speed, little_slow_speed); 
	STEP_L_DIR_FOR; 
	STEP_R_DIR_FOR; 
}
void STEP_left_revision_back() 
{ 
	STEP_speed(normal_speed, little_slow_speed); 
	STEP_L_DIR_BACK; 
	STEP_R_DIR_BACK; 
}
void STEP_right_revision_back() 
{ 
	STEP_speed(little_slow_speed, normal_speed);
	STEP_L_DIR_BACK; 
	STEP_R_DIR_BACK; 
}

void STEP_slow_L_revision()
{
	STEP_speed(slow_speed, little_slow_speed);
	STEP_L_DIR_FOR;
	STEP_R_DIR_FOR;
}

void STEP_slow_R_revision()
{
	STEP_speed(little_slow_speed,slow_speed);
	STEP_L_DIR_FOR;
	STEP_R_DIR_FOR;
}
void STEP_bt_stop()
{

}

void STEP_left_stop() 
{ 
	disable_TIMER2_ovf(); 
}
void STEP_right_stop() 
{ 
	disable_TIMER0_ovf(); 
}
void step_motor_init()
{
	STEP_DDR |= STEP_MOTOR;
}
void step_motor_go()
{
	if (toggle == 0)
	{
		STEP_motor_start(turn_speed2, turn_speed2, 0, 0);
		toggle = 1;
	}
	f_b_flag=0;
	
	if (move_flag == 0 && AngFlag==1)
	{
		if (SumAng == 0)
		STEP_back();
		else if (SumAng < 0)
		STEP_left_revision_back();
		else if (SumAng > 0)
		STEP_right_revision_back();
	}
	
	else if(move_flag==1 && AngFlag==1)
	{
		if (SumAng == -90)
		STEP_back();
		else if (SumAng < -90)
		STEP_left_revision_back();
		else if (SumAng > -90)
		STEP_right_revision_back();
	}
	
	else if(move_flag==1 && AngFlag==0)				////20181006 수정  기존 270
	{
		if (gAngle == -90)
		STEP_back();
		else if (gAngle < -90)
		STEP_left_revision_back();
		else if (gAngle > -90)
		STEP_right_revision_back();
	}
	
	else if(move_flag==2 && -224<=SumAng && SumAng<=-136)
	{
		if (SumAng == -180)
		STEP_back();
		else if (SumAng < -180)
		STEP_left_revision_back();
		else if (SumAng > -180)
		STEP_right_revision_back();
	}
	else if(move_flag==2 && 224>=SumAng && SumAng>=136)
	{
		if (SumAng == 180)
		STEP_back();
		else if (SumAng < 180)
		STEP_left_revision_back();
		else if (SumAng > 180)
		STEP_right_revision_back();
	}
	
	else if(move_flag==3 && AngFlag==1)
	{
		if (SumAng == 90)
		STEP_back();
		else if (SumAng < 90)
		STEP_left_revision_back();
		else if (SumAng > 90)
		STEP_right_revision_back();
	}
	else if(move_flag==3 && AngFlag==2)			//20181006 수정  기존 -270
	{
		if (gAngle == 90)
		STEP_back();
		else if (gAngle < 90)
		STEP_left_revision_back();
		else if (gAngle > 90)
		STEP_right_revision_back();
	}
	
	else if(move_flag==0 && -345>=SumAng && SumAng>=-375)			//20181006 수정  기존 -360
	{
		if (gAngle == -360)
		STEP_back();
		else if (gAngle < -360)
		STEP_left_revision_back();
		else if (gAngle > -360)
		STEP_right_revision_back();
	}
	
	else if(move_flag==0 && 375>=SumAng && SumAng>=345)			//20181006 수정  기존 360
	{
		if (gAngle == 360)
		STEP_back();
		else if (gAngle < 360)
		STEP_left_revision_back();
		else if (gAngle > 360)
		STEP_right_revision_back();
	}
	else if(move_flag==3 && 475>=SumAng && SumAng>=435)			//20181006 수정  기존 360
	{
		if (SumAng == 450)
		STEP_back();
		else if (SumAng < 450)
		STEP_left_revision_back();
		else if (SumAng > 450)
		STEP_right_revision_back();
	}
	
}


void step_motor_back()
{
	if (toggle == 0)
	{
		STEP_motor_start(normal_speed, normal_speed, 1, 1);
		toggle = 1;
	}

	f_b_flag=1;
	
	if (-44<=SumAng && SumAng<=44 && move_flag == 0 && AngFlag==1)
	{
		if (SumAng == 0)
		STEP_advance();
		else if (SumAng < 0)
		STEP_right_revision();
		else if (SumAng > 0)
		STEP_left_revision();
	}
	
	else if(-134<=SumAng && SumAng<=-46 && move_flag==1)
	{
		
		if (SumAng == -90)
		STEP_advance();
		else if (SumAng < -90)
		STEP_right_revision();
		else if (SumAng > -90)
		STEP_left_revision();
	}
	
	else if(226<=SumAng && SumAng<=314 &&move_flag==1 )				//20181006 수정	 기존 270
	{
		if (gAngle == -90)
		STEP_advance();
		else if (gAngle < -90)
		STEP_right_revision();
		else if (gAngle > -90)
		STEP_left_revision();
	}
	
	else if(-224<=SumAng && SumAng<=-136 && move_flag==2 )
	{
		if (SumAng == -180)
		STEP_advance();
		else if (SumAng < -180)
		STEP_right_revision();
		else if (SumAng >= -180)
		STEP_left_revision();
	}
	else if(224>=SumAng && SumAng>=136 && move_flag==2 )
	{
		if (SumAng == 180)
		STEP_advance();
		else if (SumAng < 180)
		STEP_right_revision();
		else if (SumAng > 180)
		STEP_left_revision();
	}
	
	else if(134>=SumAng && SumAng>=46 && move_flag==3 )
	{
		if (SumAng == 90)
		STEP_advance();
		else if (SumAng < 90)
		STEP_right_revision();
		else if (SumAng > 90)
		STEP_left_revision();
	}
	else if(-314<=SumAng && SumAng<=-225 && move_flag==3)		//20181006 수정    기존 -270
	{
		if (gAngle == 90)
		STEP_advance();
		else if (gAngle < 90)
		STEP_right_revision();
		else if (gAngle > 90)
		STEP_left_revision();
	}
	else if(345<=SumAng && SumAng<=375 && move_flag==0)		//20181006 수정    기존 360
	{
		if (gAngle == 0)
		STEP_advance();
		else if (gAngle < 0)
		STEP_right_revision();
		else if (gAngle > 0)
		STEP_left_revision();
	}
	else if(-375<=SumAng && SumAng<=-345 && move_flag==0)		//20181006 수정    기존 -360
	{
		if (gAngle == 0)
		STEP_advance();
		else if (gAngle < 0)
		STEP_right_revision();
		else if (gAngle > 0)
		STEP_left_revision();
	}
	else if(move_flag==3 && 475>=SumAng && SumAng>=435)			//20181006 수정  기존 360
	{
		if (SumAng == 450)
		STEP_advance();
		else if (SumAng < 450)
		STEP_right_revision();
		else if (SumAng > 450)
		STEP_left_revision();
	}


}

void step_turn_left()
{

	if (toggle == 0)
	{
		STEP_motor_start(normal_speed, normal_speed, 0, 1);
		toggle = 1;
		turn_ing=1;
	}
	
	
	if (move_flag==2 && turn==1 && (SumAng<=-178 &&SumAng>=-273))			//20181006 수정  기존 -269
	{
		if (gAngle>=1 && gAngle <= 91)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=3;
			turn_ing=0;
		}
	}
	else if (move_flag == 0 && turn==1)										//20181006 수정  sum-> gAng
	{
		if (gAngle <= -89)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=1;
			turn_ing=0;
		}
	}
	
	else if (move_flag == 1 && turn==1)
	{
		if (SumAng <= -179)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=2;
			turn_ing=0;
		}
	}
	else if (move_flag == 3 && turn==1 &&(SumAng>=-2 &&SumAng<=92))					//20181006 수정  sum-> gAng
	{
		if (gAngle <= 1)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=0;
			turn_ing=0;
		}
	}
	else if (move_flag == 3 && turn==1 && (SumAng>=-365 && SumAng<=-265))			//20181006 수정  기존 -359
	{
		if (gAngle <= 1)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=0;
			turn_ing=0;
		}
	}
	
	else if (move_flag == 2 && turn==1 && (SumAng>=88 && SumAng<=182))				//20181006 수정  sum-> gAng  현재 바라보는 각도가 180도
	{
		if (gAngle>=-170 && gAngle <= 91)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=3;
			turn_ing=0;
		}
	}

}

void step_turn_right()
{
	//	toggle = 0;
	if (toggle == 0)
	{
		STEP_motor_start(normal_speed, normal_speed, 1, 0);
		toggle = 1;
		turn_ing=1;
	}
	
	if (move_flag==2 && turn==2 && (SumAng>=-182 && SumAng<=-88))					//20181006 수정  sum-> gAng    현재 각도 -180도
	{
		if (gAngle >= -91 && gAngle <= 170)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=1;
			turn_ing=0;
		}
	}
	else if (move_flag==2 && turn==2 && (SumAng>=178 && SumAng>=272))				//20181006 수정 기존 270
		{
			if (gAngle>=-91 && gAngle<=170)
			{
				STEP_motor_stop();
				toggle=0;
				turn = 0;
				move_flag=1;
				turn_ing=0;
			}
		}
	else if (move_flag == 0 && turn==2)												//20181006 수정  sum-> gAng
	{
		if (gAngle >= 89)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=3;
			turn_ing=0;
		}
	}
	
	else if (move_flag == 1 && turn==2)
	{
		if (gAngle >= -1)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=0;
			turn_ing=0;
		}
	}
	else if(move_flag==3 && turn==2 && (SumAng<=182 &&SumAng>=88))				
	{
		if (SumAng >= 179)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=2;
			turn_ing=0;
		}
	}
	else if(move_flag==3 && turn==2 && (SumAng<=-178 &&SumAng>=-272))
	{
		if (SumAng >= -181)
		{
			STEP_motor_stop();
			toggle=0;
			turn = 0;
			move_flag=2;
			turn_ing=0;
		}
	}
	
}

/****************************STEP_MOTOR_END***************************/

void lcd(void)
{
	sprintf(lcd_data, "A:%4d  X:%4d", SumAng, X_count);
	LCD_out(0, 0, lcd_data);						   
	sprintf(lcd_data, "Y:%4d",Y_count);			  
	LCD_out(1, 8, lcd_data);
}

void lcd_axis(void)
{
	sprintf(lcd_data, "X axis : %4d   ", X_axis);
	LCD_out(0, 0, lcd_data);
	sprintf(lcd_data, "Y axis : %4d   ", Y_axis);
	LCD_out(1, 0, lcd_data);
} 

void lcd_wait(void)
{
	sprintf(lcd_data, "     WAITING   ");
	LCD_out(0, 0, lcd_data);						   
	sprintf(lcd_data, "     A:%4d     ",SumAng);			  
	LCD_out(1, 0, lcd_data);
}


void system_init() 
{
	ADC_init();
	dc_init();
	LCD_init();
	serial0_init();
	serial1_init();
	led_init();
	step_motor_init();
	sei();
}

int main ()
{
 	_delay_ms(1000);
	system_init(); //위에서 정의한 system_init() 함수를 불러온다.

	int motor_toggle = 1, motor_init = 0, tg = 0 /*토글*/,step = 0,  go = 0/*쓰레기 집으러가는지 돌아오는지 */,fin = 0;
	int yy = 0, v = -380;
	char lcd_data[16] = {0};
	move_flag = 3;
	while(1)
	{ 
		if(flag == -1)
		lcd_wait();
		else if(flag == 0)	
		lcd(); // 그 이후 실제 로봇 좌표값
	
		/**************turn*********************/
		if (turn == 1 )//turn변수가 1로 지정된 경우 왼쪽 턴
		{
			step_turn_left();
		}
		else if (turn == 2 )//turn변수가 2로 지정된 경우 왼쪽 턴오른쪽 턴z
		{
			step_turn_right();
		}
		/**************turn*********************/

	/*	if(SW_1) // 테스트용
		{
			flag = 0;
		} */
		if(flag == 0) // 블루투스 신호 받을 시 0
		{ 
			if(go == 0) // 쓰레기 집으러 갈시  0
			{
				if(tg == 0)
				{
					tg = 1;
					HAND_OPEN();
					if(SW_5)
					DC_STOP;
					
					lcd_axis(); // 받은 좌표 값 확인
					_delay_ms(1000);
				}
			
				if(step == 0) // X - 300 까지 이동
				{
					step_motor_back();
					if(X_axis <= 1500)
					{
						if( X_axis - 500 <= X_count )
						{
							STEP_stop();
							_delay_ms(100);
							toggle = 0;
							step = 1;
						}
					}
					else
					{
						if( X_axis - 1000 <= X_count )
						{
							STEP_stop();
							_delay_ms(100);
							toggle = 0;
							step = 1;
						}
					}
				}
				else if(step == 1) // 쓰레기 좌표 Y_axis 값에 따라 회전
				{
					if(Y_axis >= 30) // Y가 30보다 클시 우회전
					{
						toggle = 0;
						turn = 2;
						if(SumAng >= 178)
						{
							STEP_stop();
							_delay_ms(100);
							toggle = 0;
							step = 2;
						}
					}
					else if(Y_axis <= -30) // Y가 -30 보다 작을시 좌회전
					{
						toggle = 0;
						turn = 1;
						if(SumAng <= 1)
						{
							STEP_stop();
							_delay_ms(100);
							f_b_flag = 1;
							turn_ing = 0;
							toggle = 0;
							step = 2;
						}
					}
					else // 동일 선상 Y_axis값이면 180도 회전
					{
						toggle = 0;
						turn = 2;
						if(SumAng >= 269)
						{
							STEP_stop();
							_delay_ms(100);
							toggle = 0;
							step = 4;
						}
					}
				}
				else if (step == 2) // Y_axis 까지 이동
				{
					step_motor_back();
					if(Y_axis > 500 && Y_axis < 900)
					{
						if(Y_axis >= 30)
						{
							if( Y_axis + 150 <= Y_count )
							{
								STEP_stop();
								_delay_ms(100);
								toggle = 0;
								step = 3;
							}
						}
						else if (Y_axis <= -30)
						{
							if( Y_axis >= Y_count )
							{
								STEP_stop();
								_delay_ms(100);
								toggle = 0;
								step = 3;
							}
						}
					}
					else
					{
						if(Y_axis >= 30)
						{
							if( Y_axis <= Y_count )
							{
								STEP_stop();
								_delay_ms(100);
								toggle = 0;
								step = 3;
							}
						}
						else if (Y_axis <= -30)
						{
							if( Y_axis >= Y_count )
							{
								STEP_stop();
								_delay_ms(100);
								toggle = 0;
								step = 3;
							}
						}
					}
				}
				else if (step == 3) // 쓰레기 방향으로 회전
				{
					if(Y_axis >= 30) // y값이 클 때는 오른쪽회전
					{
						toggle = 0;
						turn = 2;
						if(SumAng >= 269)
						{
							STEP_stop();
							toggle = 0;
							move_flag = 1;
							turn = 0;
							turn_ing = 0;
							f_b_flag = 0;
							step = 4;
							_delay_ms(100);
						}
					}
					else if(Y_axis <= -30) // y값이 작을때는 왼쪽회전
					{
						toggle = 0;
						turn = 1;
						if(SumAng <= -89)
						{
							STEP_stop();
							toggle = 0;
							move_flag = 1;
							turn = 0;
							turn_ing = 0;
							f_b_flag = 0;
							step = 4;
							_delay_ms(100);
						}
					}
				}
				else if (step == 4) // 쓰레기를 향해서 직진
				{
					step_motor_go();
					
					if(X_axis <= X_count) // 주어진 x_axis 보다 +100가서 집게안으로 확실히
					{	
						STEP_stop();	
						_delay_ms(300);
						step = 5;
						toggle = 0;
					}
				}
				else if(step == 5) // 집게 집기
				{
					HAND_CLOSE();
					if( SW_2 && SW_3 ) // 집게에 달린 스위치 2개
					{
						DC_CLOSE; // 더 강하게 집도록 0.5초 더
						_delay_ms(300);
						DC_STOP;
						_delay_ms(200);
						toggle = 0;
						step = 6;
					}
				}
				else if (step == 6) // 쓰레기 분류
				{
					if(get_ADC1() > 100) // 위 psd 감지 x = 플라스틱
					{
						if(STEEL_IN) // 철센서 감지 o = 철
						{
							LCD_out(1, 0, "Steel   ");
							go = 1;
							step = 0;
							toggle = 0;
							yy = 600; // 수정필
						}
						else
						{
							LCD_out(1, 0, "Plastic ");
							go = 1;
							step = 0;
							toggle = 0;
							yy = 0; // 수정필
						}
					}
					else // 철 or 종이
					{
						LCD_out(1, 0, "Paper   ");
						go = 1;
						step = 0;
						toggle = 0;
						yy = 1200;						
					}
				}
			} // go 0
				else if (go == 1)
				{
					if(step == 0) // 후진 
					{
						step_motor_back();
						if(yy >= 500)
						{
							if(X_axis - 1000 >= X_count)
							{
								STEP_stop();
								step = 1;
								toggle = 0;
							}
						}
						else
						{
							if(X_axis - 300 >= X_count)
							{
								STEP_stop();
								step = 1;
								toggle = 0;
							}
						}
					}
					else if(step == 1)
					{
						toggle = 0;
						turn = 1;
						if(SumAng <= 181)
						{
							STEP_stop();
							_delay_ms(100);
							toggle = 0;
							turn_ing = 0;
							move_flag = 2;
							turn = 0;
							if(yy <= Y_axis)
							step = 2;
							else if (yy >= Y_axis)
							step = 11;
						}
						/*
						if (yy <= Y_axis) // 현 Y값이 가야하는 Y값보다 클 때
						{
							toggle = 0;
							turn = 1;
							if(SumAng <= 181)
							{
								STEP_stop();
								toggle = 0;
								turn_ing = 0;
								move_flag = 2;
								turn = 0;
								step = 2;
								_delay_ms(100);
							}
						}
						else if(yy > Y_axis) // 현 Y값이 가야하는 Y값보다 작을 때
						{
							if(Y_axis >= 30)
							{
								toggle = 0;
								turn = 2;
								if(SumAng >= 359)
								{
									STEP_stop();
									toggle = 0;
									turn_ing = 0;
									move_flag = 0;
									turn = 0;
									step = 2;
									_delay_ms(100);
								}
							}
							else if(Y_axis < 30)
							{
								toggle = 0;
								turn = 2;
								if(SumAng >= -1)
								{
									STEP_stop();
									toggle = 0;
									turn_ing = 0;
									move_flag = 0;
									turn = 0;
									step = 2;
									_delay_ms(100);
								}
							}
						} 
						*/
					}
					else if(step == 2)
					{
						step_motor_go();
					//	if (yy <= Y_axis) 
					//	{
							if(yy >= Y_count)
							{
								STEP_stop;
								_delay_ms(100);
								toggle = 0;
								step = 3;
							}
					//	}
					/*	else if(yy > Y_axis)
						{
							if(yy <= Y_count)
							{
								STEP_stop;
								_delay_ms(100);
								toggle = 0;
								step = 3;
							}
						}  */
					}
					else if (step == 11)
					{
						step_motor_back();
						if(yy <= Y_count)
						{
							STEP_stop;
							_delay_ms(100);
							toggle = 0;
							step = 3;
						}
					
					}
					else if(step == 3)
					{
					//	if (yy <= Y_axis)
					//	{
							toggle = 0;
							turn = 1;
							if(SumAng <= 91)
							{
								STEP_stop();
								toggle = 0;
								turn_ing = 0;
								move_flag = 3;
								turn = 0;
								step = 4;
								_delay_ms(100);
							}
					/*	}
						else if (yy > Y_axis)
						{
							if(Y_axis >= 30)
							{
								toggle = 0;
								turn = 2;
								if(SumAng >= 449)
								{
									STEP_stop();
									toggle = 0;
									turn_ing = 0;
									move_flag = 3;
									turn = 0;
									step = 4;
									_delay_ms(100);
								}
							} 
							else if(Y_axis < 30)
							{
								toggle = 0;
								turn = 2;
								if(SumAng >= 89)
								{
									STEP_stop();
									toggle = 0;
									turn_ing = 0;
									move_flag = 3;
									turn = 0;
									step = 4;
									_delay_ms(100);
								}
							}
						} */
					}
					else if (step == 4)
					{
						step_motor_go();
						if(X_count <= v)
						{
							tx0_char(1); // 청소로봇한테 물체 완료신호보내기
							STEP_stop();
							_delay_ms(100);	
							toggle = 0;
							step = 5;
							v -= 55;
						}
					}
					else if (step == 5)
					{
						HAND_OPEN();
						_delay_ms(100);
					
						step_motor_back();
						if(X_count >= 0)
						{
							STEP_stop();
							_delay_ms(100);
							toggle = 0;
							step = 6;
						}
					}
					else if (step == 6)
					{
						if( yy == 0 )
						{	step = 9;	}
						else
						{
						//	if (yy <= Y_axis)
						//	{
								toggle = 0;
								turn = 1;
								if(SumAng <= 1)
								{
									STEP_stop();
									toggle = 0;
									turn_ing = 0;
									move_flag = 0;
									turn = 0;
									step = 7;
									_delay_ms(100);
								}
						/*	}
							else if (yy > Y_axis)
							{
								if(Y_axis >= 30)
								{
									toggle = 0;
									turn = 1;
									if(SumAng <= 361)
									{
										STEP_stop();
										toggle = 0;
										turn_ing = 0;
										move_flag = 0;
										turn = 0;
										step = 7;
										_delay_ms(100);
									}
								}
								else if(Y_axis < 30)
								{
									toggle = 0;
									turn = 1;
									if(SumAng <= 1)
									{
										STEP_stop();
										toggle = 0;
										turn_ing = 0;
										move_flag = 0;
										turn = 0;
										step = 7;
										_delay_ms(100);
									}
								}
							} */
						}
					}
					else if (step == 7)
					{
						step_motor_back();
					
						if(Y_count <= 1)
						{
							STEP_stop();
							_delay_ms(100);
							toggle = 0;
							step = 8;
						}
					}
					else if (step == 8)
					{
						toggle = 0;
						turn = 2;
						step = 10;
					/*	if(SumAng <= 5 && SumAng >= -5)
						{
							toggle = 0;
							turn = 2;
							if(SumAng >= 88)
							{
								STEP_stop();
								_delay_ms(100);
							}
							step = 10;
						}
							if(SumAng >= 89)
							{
								toggle = 0;
								turn_ing = 0;
								move_flag = 3;
								X_axis = 0;
								Y_axis = 0;
								X_count = 0;
								Y_count = 0;
								go = 0;
								step = 0;
								flag = -1;
								BT_ACCEPT;
							//	fin = 1;
							//	step = 9;
							}
						}
						else if(SumAng <= 365 && SumAng >= 355)
						{
							toggle = 0;
							turn = 2;
							if(SumAng >= 448)
							{
								STEP_stop();
								_delay_ms(100);
							}
							step = 10;
						}*/
					}
					else if (step == 10)
					{
						if(SumAng >= 89)
						{
							STEP_stop();
							_delay_ms(100);
							step = 9;	
						}
					}
					else if (step == 9)
					{
						SumAng = 0;
						toggle = 0;
						turn_ing = 0;
						move_flag = 3;
						X_axis = 0;
						Y_axis = 0;
						X_count = 0;
						Y_count = 0;
						go = 0;
						step = 0;
						flag = -1;
						tg = 0;
						BT_ACCEPT;
					}
			}// go 1
		}// flag 0
	} // while
} // main
					


