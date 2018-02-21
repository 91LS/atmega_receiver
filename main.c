#include <avr/io.h>

#define UART_BAUD 9600
#define __UBRR ( F_CPU / 16.0 / UART_BAUD - 0.5 )

void PWM_Init();
void USART_Init(uint16_t ubrr);
void H_BRIDGE_Init();
uint8_t USART_Receive();
uint8_t Get_Not_Changed_PWM_Value(uint8_t fwdBack);
uint8_t Get_Changed_PWM_Value(uint8_t fwdBack, uint8_t leftRight);
void Set_PWM_Values(uint8_t fwdBackPower, uint8_t leftRightPower, uint8_t isFwd, uint8_t isLeft);
void Parse_Message_And_Set_PWM_Values();

int main(){
	PWM_Init();
	USART_Init(__UBRR);
	H_BRIDGE_Init();

	while(1){
		Parse_Message_And_Set_PWM_Values();
	}
}

void PWM_Init(){
	/*OCR0B (PD5) - prawy silnik; OCR0A (PD6) - lewy silnik;*/
	DDRD   |= (1<<PD5)    | (1<<PD6);    //timery: PD5 - OC0B oraz PD6 - OC0B; jako wyjœcia
	TCCR0A |= (1<<WGM01)  | (1<<WGM00);  //fast PWM
	TCCR0A |= (1<<COM0A1);               //clear OC0A at top
	TCCR0A |= (1<<COM0B1);               //clear OC0B at top
	TCCR0B |= (1<<CS00);                 //preskaler = 1 - czêstotliwoœæ: 31,250 kHz
}

void USART_Init(uint16_t ubrr)
{
	UBRR0H = (uint8_t)(ubrr>>8);
	UBRR0L = (uint8_t)ubrr;

	UCSR0B = (1<<RXEN0);//|(1<<TXEN0);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
}

void H_BRIDGE_Init(){
	//PC5, PC4 - PRAWY SILNIK; PC3, PC2 - LEWY SILNIK
	DDRC |= (1<<PC5) | (1<<PC4) | (1<<PC3) | (1<<PC2);
}

uint8_t USART_Receive()
{
	while ( !(UCSR0A & (1<<RXC0)) );
	return UDR0;
}

uint8_t Get_Not_Changed_PWM_Value(uint8_t fwdBack){
	if(fwdBack >= 7)
		return 255;
	else
		return 37 * fwdBack;
}

uint8_t Get_Changed_PWM_Value(uint8_t fwdBack, uint8_t leftRight){
	if (fwdBack >= 7 && leftRight == 0)
		return 255;
	else
		return 37 * fwdBack - (37 * fwdBack * leftRight) / 7;
}

void Set_PWM_Values(uint8_t fwdBackPower, uint8_t leftRightPower, uint8_t isFwd, uint8_t isLeft){
	//OCR0A - LEWY SILNIK
	//OCR0B - PRAWY SILNIK

	//przypisujemy zmiennym wartosci wedlug odczytanej wiadomosci
	uint8_t notChangedValue = Get_Not_Changed_PWM_Value(fwdBackPower);
	uint8_t changedValue = Get_Changed_PWM_Value(fwdBackPower, leftRightPower);

	if(isFwd){
		//do przodu - sygnal do mostku H
		PORTC |=  (1<<PC5);
		PORTC &= ~(1<<PC4);
		PORTC |=  (1<<PC3);
		PORTC &= ~(1<<PC2);
	}
	else{
		//do tylu - sygnal do mostku H
		PORTC &= ~(1<<PC5);
		PORTC |=  (1<<PC4);
		PORTC &= ~(1<<PC3);
		PORTC |=  (1<<PC2);
	}
	if(isLeft){
		OCR0A = changedValue;    //lewy
		OCR0B = notChangedValue; //prawy
	}
	else{
		OCR0A = notChangedValue; //lewy
		OCR0B = changedValue;    //prawy
	}
}

void Parse_Message_And_Set_PWM_Values(){
	uint8_t receivedData = USART_Receive();
	uint8_t fwdBackPower, leftRightPower, isFwd, isLeft = 0;

	fwdBackPower   = (receivedData & 0b11100000) >> 5;
	leftRightPower = (receivedData & 0b00011100) >> 2;
	isFwd          = (receivedData & 0b00000010) >> 1;
	isLeft         =  receivedData & 0b00000001;

	Set_PWM_Values(fwdBackPower, leftRightPower, isFwd, isLeft);
}
