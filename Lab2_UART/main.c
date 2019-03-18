// main.c
// Runs on LM4F120/TM4C123
// Test main for Lab 11
// December 29, 2014

// this connection occurs in the USB debugging cable
// U0Rx (PA0) connected to serial port on PC
// U0Tx (PA1) connected to serial port on PC
// Ground connected ground in the USB cable

#include "tm4c123gh6pm.h"

#define LED (*((volatile unsigned long *)0x40025038))

void UART_INIT(void);
void PORTF_INIT(void);
void UART_OutChar(unsigned char);
unsigned char UART_InChar(void);

unsigned char ch;

void EnableInterrupts(void);  // Enable interrupts
// do not edit this main
// your job is to implement the UART_OutUDec UART_OutDistance functions 
int main(void){
  PORTF_INIT();
	LED = 0x0C;
	UART_INIT();              // initialize UART
	LED = 0x04;
  while(1){
		ch = UART_InChar();
    UART_OutChar(ch);
		LED ^= 0xC;
  }
}

unsigned char UART_InChar(){
	while((UART0_FR_R&UART_FR_RXFE) != 0);
	return ((unsigned char) (UART0_DR_R&0xFF));
}

void UART_OutChar(unsigned char data){
	while((UART0_FR_R&UART_FR_TXFF) != 0);
	UART0_DR_R = data;
}

void UART_INIT(){unsigned volatile long delay;
	SYSCTL_RCGC1_R |= 0x01; // Activate UART0
	SYSCTL_RCGC2_R |= 0x01; // Activate Port A
	delay = SYSCTL_RCGC2_R;
	//while((SYSCTL_PRGPIO_R&0x02) == 0){};
	UART0_CTL_R &= ~0x01; // Disable UART
	UART0_IBRD_R = 8; // 115200 baud,  104 for 9600 baud does not work
	UART0_FBRD_R = 43; // 10;
	UART0_LCRH_R = 0x70; //8bit no parity, one stop, fifos
	UART0_CTL_R |= 0x01; // enable uart
	GPIO_PORTA_AFSEL_R |= 0x03;
	GPIO_PORTA_PCTL_R &= (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011; // configure PA0 as U0Rx, PA1 as U0Tx
	GPIO_PORTA_AMSEL_R &= ~0x03;
	GPIO_PORTA_DEN_R |= 0x03;
}

void PORTF_INIT(){
	unsigned volatile long delay;
	SYSCTL_RCGC2_R|= 0x20;
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_AMSEL_R &= ~0x0E;
	GPIO_PORTF_AFSEL_R &= ~0x0E;
	GPIO_PORTF_PCTL_R &= ~0x0000FFF0;
	GPIO_PORTF_DEN_R |= 0x0E;
	GPIO_PORTF_CR_R |= 0x0E;
	GPIO_PORTF_DIR_R |= 0x0E;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
}




