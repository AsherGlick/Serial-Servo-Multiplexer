/**\
| This program was written by Asher Glick, you are free to use and miodify it as
| you wish
\**/
/****************** CHIP SETTINGS ******************\
| This program was designed to run on an ATMEGA328  |
| chip running with an external clock at 8MHz       |
\***************************************************/

/********** FUSE SETTINGS **********\
|   Low Fuse 0xE2                   |
|  High Fuse 0xD9                   |       +- AVRDUDE COMMANDS -+
| Extra Fuse 0x07                   |       | -U lfuse:w:0xe0:m  |
|                                   |       | -U hfuse:w:0xd9:m  |
| These fuse calculations are       |       | -U efuse:w:0xff:m  |
| based off of the usbtiny AVR      |       +--------------------+
| programmer. Other programmers     |
| may have a different fuse number  |
\***********************************/

/************************** AVRDUDE command for 8MHz **************************\ 
| sudo avrdude -p m328p -c usbtiny -U flash:w:myproject.hex                    |
|                       -U lfuse:w:0xE2:m -U hfuse:w:0xD9:m -U efuse:w:0x07:m  |
|                                                                              |
| NOTE: when messing with fuses, do this at your own risk. These fuses for the |
|        ATMEGA328P (ATMEGA328) worked for me, however if they do not work for |
|        you, it is not my fault                                               |
| NOTE: '-c usbtiny' is incorrect if you are using a different programmer      |
\******************************************************************************/


#include <avr/io.h>
#include <avr/interrupt.h>



volatile char bport[255];
volatile char cport[255];
volatile char dport[255];

//servo define values
volatile char servo_add [18];
volatile char servo_sub [18]; 
volatile char * servo_port [18];
volatile int servo_pos [18];
 


/************************* TIMER1 COMPARE A INTERRUPT *************************\
| 
\******************************************************************************/
ISR(TIMER1_COMPA_vect) {
  // run through all cycles
  int i ;
  PORTB = 0xFF;
  PORTC = 0xFF;
  PORTD = 0xFF;
  for (i = 0; i<256; i++) {
    PORTB &= bport[i];
    PORTC &= cport[i];
    PORTD &= dport[i];
    // calculate
    long newtime = (500) + i*(500/256);    
    // wait here untill the next cycle should start
    while (TCNT1 < newtime);
  }
}

/************************************ WAIT ************************************\
| A generic wait function                                                      |
\******************************************************************************/
void wait (unsigned long time) {
  long i;
  for (i = 0; i < time; i++) {
    asm volatile ("nop");
  }
}

void USART_Init(void);
void USART_Transmit( int input );

/******************************* MAIN FUNCTION *******************************\
| The main function in this program mainly only handdles the serial input     |
| However because the interrupt only runs for 10% of the program anything     |
| that can be calculatied at a 90% effency at 8Mhz (effectively 7.2Mhz) then  |
| it can be run on-chip, allowing the user to cut down on expences of other   |
| devices to communicate with the chip.                                       |
\*****************************************************************************/
int main (void) {
  int i = 0;
  
  // PIN INITILIZATION (TO FIX, all out should be able to be done)
  DDRD = 0xFF;// 00000000 configure output on port D
  DDRB = 0xFF;// 00011100 configure output on port B
  DDRC = 0xFF;//                    output on port C
  
  // SERVO TIMER INITILIZATION
  TCCR1B |= (1<<WGM12);
  OCR1A = 10000;
  
  //Initlize Off Port Bits
  for (i = 0 ; i < 256; i++) {
    bport[i] = 0xFF;
    cport[i] = 0xFF;
    dport[i] = 0xFF;
  }
  bport[0] = 0x00;
  cport[0] = 0x00;
  dport[0] = 0x00;
  
  // Initlize Servos
  
  servo_add [0]  = !(1<<0);
  servo_add [1]  = !(1<<1);
  servo_add [2]  = !(1<<2);
  servo_add [3]  = !(1<<3);
  servo_add [4]  = !(1<<4);
  servo_add [5]  = !(1<<5);
  servo_add [6]  = !(1<<6);
  servo_add [7]  = !(1<<7);
  servo_add [8]  = !(1<<0);
  servo_add [9]  = !(1<<1);
  servo_add [10] = !(1<<2);
  servo_add [11] = !(1<<3);
  servo_add [12] = !(1<<4);
  servo_add [13] = !(1<<5);
  servo_add [14] = !(1<<6);
  servo_add [15] = !(1<<7);
  servo_add [16] = !(1<<0);
  servo_add [17] = !(1<<1);
  
  servo_port [0]  = bport;
  servo_port [1]  = bport;
  servo_port [2]  = bport;
  servo_port [3]  = bport;
  servo_port [4]  = bport;
  servo_port [5]  = bport;
  servo_port [6]  = bport;
  servo_port [7]  = bport;
  servo_port [8]  = cport;
  servo_port [9]  = cport;
  servo_port [10] = cport;
  servo_port [11] = cport;
  servo_port [12] = cport;
  servo_port [13] = cport;
  servo_port [14] = cport;
  servo_port [15] = cport;
  servo_port [16] = dport;
  servo_port [17] = dport;  
  
  // set the initial positions
  for (i = 0; i < 256; i ++) {
    servo_pos [i]  = 0;
  }
  
  //set the subtract bytes
  for (i = 0; i < 256; i ++) {
    servo_sub[0] = !servo_add[0];
  }
  // USART INITILIZATION
  USART_Init();
  
  //INTERRUPT INITILAIZATION
  sei ();       // enable global interrupts
  TIMSK1 |= (1<<OCIE1A);
  
  //begin timer
  TCCR1B |= (1<<CS11);
  
  //MAIN LOOP
  while (1) {
    char servnum = 0xFF; // bit one is the servo number byte (leading 0)
    char servpos; // byte two is the new position (leading 1)
    while (servnum&0x80) {
      while ( !(UCSR0A & (1<<RXC0))); // Wait for input
      servnum = UDR0;
    }
    while ( !(UCSR0A & (1<<RXC0))); // Wait for input for position
    servpos = UDR0;
    // Parse input
    servpos = (servpos & ((servnum & 0x40) << 1));
    servnum = servnum & 0x1F;
    servnum = servnum - 1; // the first servo is servo 
    // Analize input
    
    servo_port[(int)servnum][(int)servo_pos[(int)servnum]] |= servo_sub[(int)servnum];
    servo_port[(int)servnum][(int)servpos] &= servo_add[(int)servnum]; 
  }
}

/******************************** USART CONFIG ********************************\
| USART_Init(void) initilizes the USART feature, this function needs to be run |
| before any USART functions are used, this function configures the BAUD rate  |
| for the USART and enables the format for transmission                        |
\******************************************************************************/
#define FOSC 8000000 // Clock Speed of the procesor
#define BAUD 19200    // Baud rate (to change the BAUD rate change this variable
#define MYUBRR FOSC/16/BAUD-1 // calculate the number the processor needs
void USART_Init(void) {
  unsigned int ubrr = MYUBRR;
  /*Set baud rate */
  UBRR0H = (unsigned char)(ubrr>>8);
  UBRR0L = (unsigned char)ubrr;
  /*Enable receiver and transmitter */
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
  /* Set frame format: 8data, 2stop bit */
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

/******************************* USART_Transmit *******************************\
| The USART_Transmit(int) function allows you to send numbers to USART serial  |
| This function only handles numbers up to two digits. If there is one digit   |
| the message contains a space, then the digit converted to ascii. If there    |
| are two digits then the message is the first digit followed by the seccond   |
| If the input is negative then the function will output a newline character   |
\******************************************************************************/
void USART_Transmit( int input )
{
  unsigned char data;
  if (input == -1) {
    while ( !( UCSR0A & (1<<UDRE0)) );
    // Put '\n' into the bufffer to send
    UDR0 = '\r';
    //dont continue running the function to prevent outputing E
    // Wait for empty transmit buffer
    while ( !( UCSR0A & (1<<UDRE0)) );
    // Put '\n' into the bufffer to send
    UDR0 = '\n';
    //dont continue running the function to prevent outputing E
    return;
  }
  else if (input < 10 && input >= 0) {     
    while ( !( UCSR0A & (1<<UDRE0)) );
    data = '0' + input;
    UDR0 = data;
  }
  else {
    // Output E if the number cannot be outputed
    UDR0 = 'E';
  }
}

