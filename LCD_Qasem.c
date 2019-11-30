// Control Codign for L.I.M Project (COMP 6900): Runs on TM4C123/ ARM Cortex M4

#include "tm4c123gh6pm.h"
#include <stdint.h>

void LCD_Init(void);
void LCD_Clear(void);
void LCD_OutString(char *pt);
void LCD_OutChar(unsigned char letter);
void LCD_OutUDec(uint32_t n);

void SysTick_Init(void);
void SysTick_Wait(uint32_t delay);
void SysTick_Wait10ms(uint32_t delay);

void ADC0_Init(void);
uint32_t Light_Sensor(void);

int main(void){
	SysTick_Init();
  LCD_Init();              // initialize LCD
  ADC0_Init();
	uint32_t n;

  LCD_Clear();
  LCD_OutString("Start LCD");
  SysTick_Wait10ms(1000);
	
while(1){
    LCD_Clear();
		LCD_OutString("L.I.% = ");
    n=Light_Sensor();
		LCD_OutUDec(n);
	  SysTick_Wait10ms(50);
}}

// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   											// disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  											// maximum reload value
  NVIC_ST_CURRENT_R = 0;                											// any write to current clears it                                    
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;  // enable SysTick with core clock
}

// Time delay using busy wait: delay parameter in units of 20 nsec for 50 MHz clock.
void SysTick_Wait(uint32_t delay){
  volatile uint32_t elapsedTime;
  uint32_t startTime = NVIC_ST_CURRENT_R;
  do{ elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;}
  while(elapsedTime <= delay);
}
// Time delay using busy wait: This assumes 50 MHz system clock.
void SysTick_Wait10ms(uint32_t delay){
  uint32_t i;
  for(i=0; i<delay; i++){
  SysTick_Wait(500000);  				// wait 10ms (assumes 50 MHz clock)
  }}

// LCD Module: Simple device driver for the LCD Runs on TM4C123 and provide several functions for LCD
#define E  0x80 // on PA7
#define RS 0x40 // on PA6
#define LCDDATA (*((volatile uint32_t *)0x400053FC))   // PORTB
#define LCDCMD (*((volatile uint32_t *)0x40004300))    // PA7-PA6
#define BusFreq 50            // assuming a 50 MHz bus clock
#define T6us 6*BusFreq        // 6us
#define T40us 40*BusFreq      // 40us
#define T160us 160*BusFreq    // 160us
#define T1600us 1600*BusFreq  // 1.60ms
#define T5ms 5000*BusFreq     // 5ms
#define T15ms 15000*BusFreq   // 15ms

// Control Commands  of LCD 
void OutCmd(unsigned char command){
  LCDDATA = command;
  LCDCMD = 0;           // E=0, R/W=0, RS=0
  SysTick_Wait(T6us);   // wait 6us
  LCDCMD = E;           // E=1, R/W=0, RS=0
  SysTick_Wait(T6us);   // wait 6us
  LCDCMD = 0;           // E=0, R/W=0, RS=0
  SysTick_Wait(T40us);  // wait 40us
}

// Initialize LCD (no inputs, no outputs)
void LCD_Init(void){ volatile long delay;
  SYSCTL_RCGC2_R |= 0x00000003;  // 1) activate clock for Ports A and B
  delay = SYSCTL_RCGC2_R;        // 2) no need to unlock
  GPIO_PORTB_AMSEL_R &= ~0xFF;   // 3) disable analog function on PB7-0
  GPIO_PORTA_AMSEL_R &= ~0xC0;   //    disable analog function on PA7-6              
  GPIO_PORTB_PCTL_R = 0x00000000;   // 4) configure PB7-0 as GPIO   
  GPIO_PORTA_PCTL_R &= ~0xFF000000; //    configure PA7-6 as GPIO
  GPIO_PORTB_DIR_R = 0xFF;       // 5) set direction register
  GPIO_PORTA_DIR_R |= 0xC0;
  GPIO_PORTB_AFSEL_R = 0x00;     // 6) regular port function
  GPIO_PORTA_AFSEL_R &= ~0xC0;
  GPIO_PORTB_DEN_R = 0xFF;       // 7) enable digital port
  GPIO_PORTA_DEN_R |= 0xC0;
  GPIO_PORTB_DR8R_R = 0xFF;      // enable 8 mA drive
  GPIO_PORTA_DR8R_R |= 0xC0;
  SysTick_Init();       // Volume 1 Program 4.7, Volume 2 Program 2.10
  LCDCMD = 0;           // E=0, R/W=0, RS=0
  SysTick_Wait(T15ms);  // Wait >15 ms after power is applied
  OutCmd(0x30);         // command 0x30 = Wake up
  SysTick_Wait(T5ms);   // must wait 5ms, busy flag not available
  OutCmd(0x30);         // command 0x30 = Wake up #2
  SysTick_Wait(T160us); // must wait 160us, busy flag not available
  OutCmd(0x30);         // command 0x30 = Wake up #3
  SysTick_Wait(T160us); // must wait 160us, busy flag not available
  OutCmd(0x38);         // Function set: 8-bit/2-line
  OutCmd(0x10);         // Set cursor
  OutCmd(0x0C);         // Display ON; Cursor ON
  OutCmd(0x06);         // Entry mode set
}

// Output a character to LCD (Inputs==>ASCII character [0 to 0x7F])
void LCD_OutChar(unsigned char letter){
  LCDDATA = letter;
  LCDCMD = RS;          // E=0, R/W=0, RS=1
  SysTick_Wait(T6us);   // wait 6us
  LCDCMD = E+RS;        // E=1, R/W=0, RS=1
  SysTick_Wait(T6us);   // wait 6us
  LCDCMD = RS;          // E=0, R/W=0, RS=1
  SysTick_Wait(T40us);  // wait 40us
}

// Clear the LCD (no inputs, no outputs)
void LCD_Clear(void){
  OutCmd(0x01);          // Clear Display
  SysTick_Wait(T1600us); // wait 1.6ms
  OutCmd(0x02);          // Cursor to home
  SysTick_Wait(T1600us); // wait 1.6ms
}

// Output a String to LCD (Inputs==> Pointer to a NULL-terminated string to be transferred)
void LCD_OutString(char *pt){
  while(*pt){
    LCD_OutChar(*pt);
    pt++;
  } }

// Output a 32-bit number in unsigned decimal format (Input==> 32-bit number to be transferred)
void LCD_OutUDec(uint32_t n){ // Recursion to convert decimal number to ASCII string
  if(n >= 10){
    LCD_OutUDec(n/10);
    n = n%10;
  }
  LCD_OutChar(n+'0'); /* n is between 0 and 9 */
}

// Intializating Analog to Digital Convertor (ADC0) (Needs 13 steps)
void ADC0_Init(void){ 
	   /*initialize PE3 for AIN0 input  */
    SYSCTL_RCGCGPIO_R  |=  0x10;     // 1)  enable clock to GPIOE (AIN0 is on PE3) 
 		GPIO_PORTE_DIR_R   &= ~0x08;   	 // 2)  Configure PE3 as Input
  	GPIO_PORTE_AFSEL_R |= 0x08;      // 3)  enable alternate function
    GPIO_PORTE_DEN_R 		= ~0x08;     // 4)  disable digital function
    GPIO_PORTE_AMSEL_R |= 0x08;      // 5)  enable analog function 
  
    /* initialize ADC0 */
		SYSCTL_RCGC0_R |= 0x00010000;   // 6)  activate clock for ADC0 (set bit# 16 to 1)
	  SYSCTL_RCGC0_R &= ~0x00000300;  // 7)  configure speed of conversion for 125K (set bits 8 and 9 to 00)
		ADC0_SSPRI_R = 0x0123;          // 8)  Assign Sequencer 3 the highest priority (Set bits 12-13 to 00)
		ADC0_ACTSS_R   = ~0x08;         // 9)  disable SS3 during configuration (set bit#3 to 1)
    ADC0_EMUX_R    = ~0xF000;     	// 10) software trigger conversion for SS3 (Set bits 15-12 to 0000)
    ADC0_SSMUX3_R  = 0x00;        	// 11) get input from channel 0 (since PE3 is hardwired with AIN0)
    ADC0_SSCTL3_R |= 0x0006;       	// 12) take one sample at a time, set flag at 1st sample (set flag on sample campture) 
    ADC0_ACTSS_R  |= 0x0008;        // 13) enable ADC0 sequencer 3 after configuration (claer set bit#3 to 0)
}

uint32_t Light_Sensor(void){ //Sampling capture (4 steps)
	uint32_t result;								 // Variable to store periodic samples
	ADC0_PSSI_R = 0x0008;            // 1)  initiate SS3 (set bit#3 to 1)
  while((ADC0_RIS_R&0x08)==0){};   // 2)  wait for conversion done (keep check sampling falg until set to 1: sample ready)
  result = ADC0_SSFIFO3_R&0xFFF;   // 3)  read result (once Sample flag is set, then read the sample)
  ADC0_ISC_R = 0x0008;             // 4)  acknowledge completion (Claer the sampling Flag by setting bit#3 in ISC 
	result = result*100/4095;				 // Normalization of senor reads (convert from 12-bit reading to %)
	SysTick_Wait10ms(50);						 // Wait for the next sensor reading to be sampled 
	return result; 
}
