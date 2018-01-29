#include <avr/io.h>
#include <string.h>
#include <stdio.h>
#define F_CPU 16000000
#include <avr/interrupt.h>
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)


void USART_init(void);
unsigned char USART_receive(void);
void USART_send( unsigned char data);
void USART_putstring(char* StringPtr);

volatile int overflow_count = 0;
volatile int pwm = 0;
volatile bool chave = false;
volatile int ST = 0, SL = 0;


ISR(TIMER0_OVF_vect) {

  overflow_count = overflow_count + 1; // 1 overflow é equivalente a 16ms
  if (overflow_count < 1830 ) {

    pwm = floor(overflow_count * (307.0 / 1830.0)) + 150.0;
  }
  else if (overflow_count >= 1830  && overflow_count < 3660 ) {

    pwm = 307.0 + 150;
  }
  else if (overflow_count >= 3660 && overflow_count < 5490 ) {

    pwm = floor(((460.0 / 1830.0) * overflow_count) - 613.0) + 150.0;
  }
  else if (overflow_count >= 5490 && overflow_count < 7320 ) {

    pwm = 767.0 + 150.0;

  }
  else if (overflow_count >= 7320 && overflow_count < (10980) ) {
    pwm = floor(((-767.0 / 3570.0) * overflow_count) + 2340.0) + 150.0;
  }
  else {
    USART_putstring("0,0,0,0\n");
    chave = false;
    overflow_count = 0;


  }
}

int main(void) {
  USART_init();
  DDRB = 0b00001110; // Configurando B1 e B2 e B3 como saida(PWM)
  DDRD = 0b10001000; // Configurando D3(PWM) e D7(Digital)
  TCCR2A = 0b10100011; // Configurando registrador do PWM B1 e B2
  TCCR2B = 0b00000001; // Configurando registrador do PWM B1 e B2
  TCCR1A = 0b10100011; // Configurando registrador do PWM B3 e D3
  TCCR1B = 0b00000001; // Configurando registrador do PWM B3 e D3
  TCCR0A = 0b00000000; // configurando o contador para overflow
  TCCR0B = 0b00000101; // configurando o contador para overflow
  ADMUX = 0b01000000; // valor de referência
  ADCSRA = 0b10000111; // habilita o leitor do analogico e o escalonamento
  TIMSK0 = 0b00000001; // Habilita a configuração de Overflow
  sei();
  
  USART_putstring("0,0,0,0\n");

  while (1) {

    if ((PIND & 0b00000100) == 4) {
      chave = true;
      PORTD = (1 << PD7);
      overflow_count = 0;
    }


    while (chave) {

      ADMUX = 0b10000000; //escolher porta a ser usada no caso A5
      ADCSRA |= 0b01000000; // inicializar a leitura analogico
      while (!(ADCSRA & 0b00010000)); //espera por interupção que indica final de leitura
      SL = ADC;

      ADMUX = 0b10000001; //escolher porta a ser usada no caso A4
      ADCSRA |= 0b01000000; // inicializar a leitura analogico
      while (!(ADCSRA & 0b00010000)); //espera por interupção que indica final de leitura
      ST = ADC;

      OCR1A = pwm - (SL / 10.0) - (ST / 10.0); //150 É O MINIMO PRA VENCER A INERCIA DA FAN,
      OCR1B = ST; //PWM PRA LED1(DIZER A TENSÃO DO SL) atribuir valor do LD1 ao sinal do sensor de luz
      OCR2A = SL; //B3 PWM PRA LED2(DIZER A TENSÃO DO ST) atribuir valor do LD2 ao sinal do sensr de temperatura
      OCR2B = pwm - (SL / 10.0) - (ST / 10.0); //D3 PWM PRA LED3(DIZER A TENSÃO DA FAN) atribuir valor do LD3 ao sinal de PWM da FAN

      char aux[] = "";
      itoa(pwm, aux, 10);
      USART_putstring(aux);
      USART_putstring(",");

      char aux2[] = "";
      itoa(SL, aux2, 10);
      USART_putstring(aux2);
      USART_putstring(",");

      char aux3[] = "";
      itoa(ST, aux3, 10);
      USART_putstring(aux3);
      USART_putstring(",");

      USART_putstring("1");
      USART_putstring("\n");


    }

    overflow_count = 0;

    PORTD = (0 << PD7);
    chave = false;
    OCR1A = 0;
    OCR1B = 0;
    OCR2A = 0;
    OCR2B = 0;

  }
}

void USART_init(void) {

  UBRR0H = (uint8_t)(BAUD_PRESCALLER >> 8);
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (3 << UCSZ00);
}

unsigned char USART_receive(void) {

  while (!(UCSR0A & (1 << RXC0)));
  return UDR0;

}

void USART_send( unsigned char data) {

  while (!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;

}

void USART_putstring(char* StringPtr) {

  while (*StringPtr != 0x00) {
    USART_send(*StringPtr);
    StringPtr++;
  }

}

