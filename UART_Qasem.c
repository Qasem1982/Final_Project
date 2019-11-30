#include "tm4c123gh6pm.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Funtions Declaration
void UART_INIT(void);
void UART_OutChar(unsigned char data);
void UART_OutUDec(uint32_t n);
void printString (char * string);

void ADC0_Init(void);
void Light_Sensor(void);

void PortE_Init(void); 
uint32_t PortE_Input(void);
void delay (int tm);

uint32_t read_light;


int main(void){
PortE_Init(); // Intilization of Port E, Called Once
UART_INIT()	; // Intilization of UART Called Once
ADC0_Init();  // Intilization of ADC_0, Called Once
int data = 0; 

printString ("\n \r");
printString ("Please the pushbutton to start LDR Sensor Calibration: \n \r");
delay(10000);
	
while(1){
data =PortE_Input(); 
if(data==0x04){	
		Light_Sensor();
		delay(10000);
} }
return 0; 
}

//Transmit characters (byte-by-byte) to PC
void printString (char * string){
while (*string) // until the string is NULL
{
UART_OutChar(*(string++));
// Read characters from sequentially // memory addresses one by one // and push them out to PC via UART
}}

// UART Intilization Function
void UART_INIT(void){ 
SYSCTL_RCGCUART_R |= 0x0001;   		// activate UART0
SYSCTL_RCGCGPIO_R |= 0x0001; 			// activate port A
UART0_CTL_R &= ~0x0001;				 		// disable UART
UART0_IBRD_R = 104; 							// IBRD=int(50000000/(16*115,200)) = int(27.1267)
UART0_FBRD_R = 11; 						 		// FBRD = round(0.1267 * 64) = 8
UART0_LCRH_R = 0x0070; 						// 8-bit word length, enable FIFO
UART0_CTL_R = 0x0301; 				 		// enable RXE, TXE and UART
GPIO_PORTA_PCTL_R =(GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011; 
GPIO_PORTA_AMSEL_R &= ~0x03; 	 		// disable analog function on PA1-0
GPIO_PORTA_AFSEL_R |= 0x03; 	 		// enable alt funct on PA1-0
GPIO_PORTA_DEN_R |= 0x03; 		 		// enable digital I/O on PA1-0
}

// PORT_E Intilization Function
void PortE_Init(void){
	SYSCTL_RCGCGPIO_R  |=  0x10;    // activate clock for part B	
	GPIO_PORTE_DIR_R   &= ~0x04;    // One external SW at PE2: Input
	GPIO_PORTE_DIR_R   |=  0x12;    // Two external LEDs at PE1 and PE4: Outputs
	GPIO_PORTE_DEN_R   |=  0x16;    // enable digital function at PE1, PE2, PE4
	GPIO_PORTE_AMSEL_R &= ~0x16;    // Disable analog function on PE1, PE2, PE4
	GPIO_PORTE_AFSEL_R &= ~0x16;    // Disable alternative function on PE1, PE2, PE4
}

// Reading Input Swich via PORT_B
uint32_t PortE_Input(void){       //Pass along PE2 as the input data triggered by SW(positive logic)
	uint32_t data = GPIO_PORTE_DATA_R & 0x04;
	return(data); 
}

// Output one charatcter into UART
void UART_OutChar(unsigned char data) {
while((UART0_FR_R&0x0020) != 0);
UART0_DR_R = data;
}

// Simple Delay function 
void delay (int tm){															
  int cnt =0;
  while (cnt<tm)
		++cnt;	}					 // Just loop and increment

// Output Decimal into UART
void UART_OutUDec(uint32_t n){
  if(n >= 10){
    UART_OutUDec(n/10); // Use recursion to convert decimal number to an ASCII string
    n = n%10;
  }
  UART_OutChar(n+'0'); // n is between 0 and 9 //
}

// Intializating ADC0 (Needs 13 steps)
void ADC0_Init(void){ 
	   /*initialize PE3 for AIN0 input  */
    SYSCTL_RCGCGPIO_R  |=  0x10;     // enable clock to GPIOE (AIN0 is on PE3) 
 		GPIO_PORTE_DIR_R   &= ~0x08;   	 // Configure PE3 as Input
  	GPIO_PORTE_AFSEL_R |= 0x08;      // enable alternate function
    GPIO_PORTE_DEN_R 		= ~0x08;     // disable digital function
    GPIO_PORTE_AMSEL_R |= 0x08;      // enable analog function 
  
    /* initialize ADC0 */
		SYSCTL_RCGC0_R |= 0x00010000;   // activate clock for ADC0 (set bit# 16 to 1)
	  SYSCTL_RCGC0_R &= ~0x00000300;  // configure speed of conversion for 125K (set bits 8 and 9 to 00)
		ADC0_SSPRI_R = 0x0123;          // Assign Sequencer 3 the highest priority (Set bits 12-13 to 00)
		ADC0_ACTSS_R   = ~0x08;         // disable SS3 during configuration (set bit#3 to 1)
    ADC0_EMUX_R    = ~0xF000;     	// software trigger conversion for SS3 (Set bits 15-12 to 0000)
    ADC0_SSMUX3_R  = 0x00;        	// get input from channel 0 (since PE3 is hardwired with AIN0)
    ADC0_SSCTL3_R |= 0x0006;       	// take one sample at a time, set flag at 1st sample (set flag on sample campture) 
    ADC0_ACTSS_R  |= 0x0008;        // enable ADC0 sequencer 3 after configuration (claer set bit#3 to 0)
}

void Light_Sensor(void){ //Sampling capture (4 steps)
	uint32_t result;								 // Variable to store periodic samples
	while(1){												 // Continous calibrating
	ADC0_PSSI_R = 0x0008;            // 1) initiate SS3 (set bit#3 to 1)
  while((ADC0_RIS_R&0x08)==0){};   // 2) wait for conversion done (keep check sampling falg until set to 1: sample ready)
  result = ADC0_SSFIFO3_R&0xFFF;   // 3) read result (once Sample flag is set, then read the sample)
  ADC0_ISC_R = 0x0008;             /* 4) acknowledge completion (Claer the sampling Flag by setting bit#3 in ISC 
																				 which will clear bit#3 in RIS) which means that i am ready for next sample*/
	result = result*100/4095;				 // Normalization of senor reads (convert from 12-bit reading to %)
	printString ("Measured Light Intensity (in %) = ");
	UART_OutUDec(result);						// Out the result to UART
	if (result>=70) {
			GPIO_PORTE_DATA_R |=  0x10;	// Turn the RED LED On if Intensity >=70% (thresold)
	}
	else GPIO_PORTE_DATA_R &=  0xEF;// Turn the RED LED OFF if Intensity < 70% (thresold)
	
	GPIO_PORTE_DATA_R |=  0x02;	   // Flash Green LED while circuit is ON
	delay(2000000);
	GPIO_PORTE_DATA_R &=  0xFD;
	printString ("\n \r");
			}}
