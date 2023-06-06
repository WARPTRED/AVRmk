#include <tiny2313.h>
#include <math.h>
#include <io.h>

#define  F_CPU (8000000)
#define  VFG_TIMER_MAX (65535)
#define  VFG_DDR DDRB
#define  VFG_PORT PORTB

#define CS    PORTD3     // Chip select
#define DO    PORTB5     // MISO or Data Out
#define USCK  PORTB7     // Clock

typedef union 
{
  unsigned long int w   ;     // w as WORD
  unsigned int h[2];          // h as HALF-WORD
  unsigned char  b[4];        // b as BYTE
} Union32;
Union32 dFi;

unsigned int fG;
unsigned char nG;
unsigned int N[]={1,8,64,256,1024};
unsigned char flag_RT = 0;
unsigned char ch_num = 0;

volatile char reqID = 0;  // This is for the first byte we receive, which is intended to be the request identifier
volatile unsigned char index = 0;  // this is to send back the right element in the array

//***********************************************USI************************************************

void SpiSlaveInit() {
    #asm("cli")
    USICR = ((1<<USIWM0)|(1<<USICS1));  // Activate 3- Wire Mode and use of external clock but NOT the interrupt at the Counter overflow (USIOIE)
    PORTD |= 1<<CS;                     // Activate Pull-Up resistor on PD3
    PCMSK|=1<<CS;                       // Active Interrupt on PD3
    GIMSK|=1<<PCIE;                     // General Interrupt Mask Register / PCIE bit activates external interrupts
    #asm("sei")
}

// External Interrupt 0 service routine
interrupt [EXT_INT1] void ext_int1_isr(void) {    
    
    if((PIND & (1<<CS))== 0){

// If edge is falling, the command and index variables shall be initialized
// and the 4-bit overflow counter of the USI communication shall be activated:
    reqID = 0;
    index = 0;
	flag_RT = 0;
    USICR |= (1<<USIOIE);
    USISR = 1<<USIOIF;      // Clear Overflow bit
    }  
    else{
// If edge is rising, turn the 4-bit overflow interrupt off:     
    USICR &= ~(1<<USIOIE);
    }
}

interrupt [USI_OVERFLOW] void usi_ovf_isr(void) {
switch(reqID) {
	case 0:
		ch_num = USIDR;
		USISR = 1<<USIOIF;  // Clear Overflow bit
		reqID++;
	break;
	case 1:
		dFi.b[0] = USIDR;
		USISR = 1<<USIOIF;  // Clear Overflow bit
		reqID++;
	break;
	case 2:
		dFi.b[1] = USIDR;
		USISR = 1<<USIOIF;  // Clear Overflow bit
		reqID++;
	break;
	case 3:
		dFi.b[2] = USIDR;
		USISR = 1<<USIOIF;  // Clear Overflow bit
		reqID++;
		flag_RT = 1;
	break;
    }      
}

//***********************************************timer1************************************************

void Tim1Init(void)
{
    #asm("cli")
    TCCR1A = (1<<COM1A0); //toggle on compare
    TCCR1B = (1<<WGM12)|(1<<CS12)|(1<<CS10); // set timer CTC mode, prescaler 1024
    TIMSK = (1<<OCIE1A);
    #asm("sei")
}

void SetUpTim1A(unsigned int Foc)    //calculate value OCR1A register
{
 unsigned int TimDiv;
 unsigned char ClockSelect=0;
 unsigned char i;
 for(i=0;i<=4;i++) {
    TimDiv = (F_CPU/(2*16*Foc*N[i])-1);
    if(TimDiv >= 0 && TimDiv<VFG_TIMER_MAX){
     ClockSelect=i+1;
     break;
    }
 }
    #asm("cli")
    OCR1A = TimDiv;
    TCCR1B = (1<<WGM12) | (ClockSelect<<CS10);
    #asm("sei")
}

void UpdateTim1A(unsigned int freq) //old value storage
{
	static unsigned int fG_old = 0;

	if (fG_old != freq)
	{
		SetUpTim1A(freq);
		fG_old = freq;
	}
}

void set_out_pin (unsigned char num){ //output pin selection
 nG=1<<num;
 VFG_DDR = nG;
}

interrupt [TIM1_COMPA] void timer1_compa_isr(void)
{
 VFG_PORT = (VFG_PORT^nG)&(nG);
}

void main(void)
{
 static unsigned int fG_old = 0;
 SpiSlaveInit();
 Tim1Init();
 UpdateTim1A(fG);
  
#asm("sei")
for(;;) {
 if (fG_old != fG) {   //old value detction
    SetUpTim1A(fG);
    fG_old = fG;
    }

 nG= ch_num;                         //generator number
 fG= dFi.b[0]+dFi.b[1]+dFi.b[2];     //generator frequency
 if (flag_RT = 1) {
 set_out_pin (nG);
 SetUpTim1A(fG);
 }
 }
}
