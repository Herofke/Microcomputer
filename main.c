//Oemer Faruk Kahraman

#include "STM32F4xx.h"
#include "_mcpr_stm32f407.h"
#include "fonts.h"
#include "string.h"
#include "stdio.h" 

#define LCD_WriteCommand (*((unsigned short *) (0x60000000)))
#define LCD_WriteData (*((unsigned short *) (0x60100000)))
	
void LCD_WriteReg(unsigned short command, unsigned short data);
void LCD_Init(void);
void LCD_SetCursor (uint32_t x, uint32_t y);
void LCD_DrawPixel (uint16_t color);
void LCD_ClearDisplay(uint16_t color);
void LCD_WriteLetter (uint32_t x, uint32_t y, uint16_t textcolor, uint16_t backgroundcolor, char code);
void LCD_WriteString(char text[], uint32_t x, uint32_t y, uint16_t textcolor, uint16_t backgroundcolor);


void InitTimer7(void);
void TIM7_IRQHandler (void);
void toggleBackground (short t);

char pointer[17];
uint16_t key=0x0000;
uint16_t keyold=0x0000;
uint16_t currentlypressed=0x0000;
uint16_t currentlyreleased=0x0000;

char Time[80]; 
char Temp[200];	
signed short delta_T = 0;

char PTS_display[30];
int potentiometer_1, potentiometer_2=0;
	
void busyWait_us(unsigned int usec)
{
	static int i=0;
	while( i<=usec*8+usec/2){
		i++;
   }
	i=0;
}

void setupUserButton()
{
	
	RCC->AHB1ENR   |= (0x1);    	// GPIO A enable
	GPIOA ->MODER  &= 0xFFFFFFFC;			// set PA0 as input
	GPIOA -> PUPDR &= 0xFFFFFFFC;			// set PA0 as floating pin
	
}

int getUserButtonState()
{
	int pinState = (GPIOA -> IDR & 0x01);
	return pinState;
}

void setupGreenLed()
{
		RCC->AHB1ENR |= (0x1 << 3);    		// GPIO D enable
		GPIOD->MODER |= 0x01000000;
}

void setGreenLed(int val)
{
	if(val == 1)
	{
		GPIOD->ODR  |= (0x1 << 12);
	}
	else
	{
		GPIOD->ODR &= ~(0x1 << 12);
	}
}
void setBoardLed(int ledIndex)
{
	*(unsigned short volatile *) 0x60120000 =  (unsigned int)(1 << ledIndex);
}

//Task 6
void set_keypad_matrix()
{
	GPIOB->MODER &= ~0xFF00;
	GPIOB->MODER |= 0x5500;
	GPIOB->OTYPER |= 0xF0;
	GPIOA->PUPDR  |= 0x1540;	
}



uint16_t read_key(void)
{
	uint16_t key=0x0000;
	for (int i=4; i<8; i++)
	{
	//IDR->We only need the 4 bits which are important for us respectively 3..6 and these 4 bits will be shifted to the left-9(0xF000)
					//GPIOB->ODR |= 0x000000F0;//The needed for Bits are set as "High"
						GPIOB->ODR &= ~(1<<i); 
						busyWait_us(45); //Wait 
					  key=(key>>4) |((~(GPIOA->IDR<<9)) & 0xF000);
	} 
	return key;
}



void Short2BitString(char *pointer, uint16_t key)
{
	for(unsigned int i=0; i<=15; i++){
		if(key & (1<<i)){
			pointer[15-i]='1';
		}
		else{
			pointer[15-i]='0';
		}
	}
	pointer[16]='\0';
}

//Task_7
void temperatureReadCircuitInit(){
	  RCC->AHB1ENR |= 0x04;   //Clock for port C
	
		GPIOB->MODER |=0x40000000; //PB15 OUTPUT for loading C
		RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;	 //Clock for Timer 8
		GPIOC->AFR[1] |=0x33; //We have only PC8&PC9 which will run over TIM8 at this situation. Alternative function number: 3
		GPIOC->MODER |= 0xA0000; //Alternative function for PC8 und PC9
		
		TIM8->PSC = 167; 					// -> 1MHz 
		TIM8->ARR = 0xFFFF; 			// full scale auto reload (CNT counts to ARR, next Tick resets CNT), 
															// 1us timer tick -> overflow period is 65.536ms  
	
		TIM8->CNT = 0x0000; 			// reset counter reg. of Timer 8, only upcouting 
		TIM8->SMCR = 0x0000; 			// It can directly read because there is no external clock mode.
		TIM8->CR1 = 0x0005; 			// update only on counter overflow (Bit 2), Run (Bit 0)
		TIM8->CCMR2 = 0x0101; 		
		TIM8->CCER = 0x3300; 			// Capture activated, falling edge, set after CCMR2
		TIM8->CCR3 = 0x0000; 			// Capture/Compare Register 3 
		TIM8->CCR4 = 0x0000; 			// Capture/Compare Register 4 
}

signed short readTemperature()
{
    GPIOB->ODR |= 0x8000; //PB15 VOLTAGE is on, the Capacitor will be charge.
    busyWait_us(100000);
    GPIOB->ODR &= ~(0x8000);	//PB15 VOLTAGE OFF
    signed short temperature = -256;
    if(TIM8->SR& 0x8){//Status register CC4IF 
        delta_T = (TIM8->CCR4)-(TIM8->CCR3);
        
        uint16_t R_KTY = 0;
        //alpha=1,143 (Look at the exercise 4-G for the mathematical calculation) 
        //The floating numbers must be ignored -> "alpha"/1000
			
			  //Tau in us and Capacity in 1uF -> Works directly
        R_KTY = ((delta_T*1000)/1143)/1;
        temperature = ((R_KTY-800)*100)/79; //
        }
				//Reset
        TIM8->SR &= 0xffe1; 
    return temperature;
}


//-------------------------------------------Task_8------------------------------------------
//Measuring the status of potentiometers and then display them indivudually as a percentage.
//STM32F4 can reacth the resolution of 6,8,10,12
//Voltage Value of a bit = (Max Voltage)/(2^n)
//n is the resolution
//ADC of STM32F4 has 3 canals and it can reach max speed of 36 MHz.
//The speed of the ADC will be decrease when the resolution increasees.(Resolution 12 = 12 Cycle)
//Reference Voltage = It is the supply voltage of Micro Processor.
//Both of the potentiometers will be operated in two different canals.

void AnalogDigitalConvertorInit()
{
	//Setting the PC0 and PC1 as Analog
	GPIOC->MODER |= 0x0000000F; 
	
//Potentiometer_1
	//ADC on-off control
	//The ADC is powered on by setting the ADON bit in the ADC_CR2 register. When the ADON
	//bit is set for the first time, it wakes up the ADC from the Power-down mode.
	//Conversion starts when either the SWSTART or the JSWSTART bit is set.
	//You can stop conversion and put the ADC in power down mode by clearing the ADON bit. In
	//this mode the ADC consumes almost no power (only a few microA).
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //Enabling the clock for ADC1(Takt) 
  ADC1->CR2    |= 0x00000001;  //Setting of the ADON (Page 416 Reference Manual)  
  ADC1->SQR3   |= 0x0000000A;  //Setting the canal number 10 
	ADC1->SQR1   &= 0xFF0FFFFF;	 //Anazahl der Kan?le=1, S. 420, optinal, da bereits 0 ist
	ADC1->CR2    |= 0x40000002;  //Setting the CONT and SWSTART 

//Potentiometer_2
	RCC->APB2ENR |=RCC_APB2ENR_ADC2EN;
	ADC2->CR2    |= 0x00000001;    //Enabling the clock for ADC2(Takt) 
  ADC2->SQR3   |= 0x0000000B;    //The 11. Canal is switched on  
	ADC2->SQR1   &= 0xFF0FFFFF;    //
	ADC2->CR2    |= 0x40000002;    //Setting the CONT(Continuos Mode) and SWSTART 
}


//The ADC writes the  result of conversion into a regular data register(ADCx_DR). 
//Here x stands for the index of the ADC used (1,2or3). 
//It should be noted that analog signals do not mix with digital signals well. 
//It is compulsory to turn off the digital circuitry for the pin designated(belirlenmis) to be used for analog signals, or analog signals will be hampered(prevent). 
//This can be done by setting bits in register MODE for ports. 
//The ADC block includes hardware to support up to 16 consecutive conversion sat pins stated in advance; 
//when one intends to implement consecutive conversion mode then the number of consecutive conversions and the desired multiplexer
//settings must be stated in registers ADCx_SQR1 and ADCx_SQR2. However, there is only one register to hold result of a conversion,
//so a user programor DMA(directmemoryaccess) hardware must be set in order to move consecutive results from the data register before next conversion is finished.

void AnalogDigitalConvertorValue()
{
potentiometer_1 = ADC1->DR;
potentiometer_2 = ADC2->DR;
}


//---------------------------------------------------------PWM----------------------------------------------------------------------
//Power-hungry consumers, such as motors, are normally controlled by pulse width modulation, as this allows the power to be adjusted and only low switching losses occur. 
//The pulse width modulation is generated by the microcontroller and passed on to an H-bridge device containing the power transistors and some protection circuits. 
//The IC5 module responsible for this, an H-bridge module MC34931 from freescale is located directly under the connector for motor current K6 and has a small heat sink for a good reason.

//-------------------------------------------------------H-Bridge-------------------------------------------------------------------
//An H-bridge is an electronic circuit that switches the polarity of a voltage applied to a load. These circuits are often used in robotics and other applications to allow DC motors to run forwards or backwards.
//Most DC-to-AC converters (power inverters), most AC/AC converters, the DC-to-DC push–pull converter, most motor controllers, and many other kinds of power electronics use H bridges. 
//In particular, a bipolar stepper motor is almost invariably driven by a motor controller containing two H bridges. 
//The H-bridge arrangement is generally used to reverse the polarity/direction of the motor, but can also be used to 'brake' the motor, 
//where the motor comes to a sudden stop, as the motor's terminals are shorted, or to let the motor 'free run' to a stop, as the motor is effectively disconnected from the circuit. The following table summarises operation, with S1-S4 corresponding to the diagram above. 


//----------------------------------------------------Circuit on the Board---------------------------------------------------------
//The microcontroller can control the throttle valve motor with the outputs PB0 and PB1 via the power bridge IC5. 
//According to the control (e.g. via a PWM signal) the motor generates a torque which more or less opens the throttle valve against a return spring.
//An enable line (EN to PC11) is used to enable the bridge (H-active). The FB line outputs an analog signal to PA2 which is proportional to the current through the bridge.
//A status flag connected to PD6 indicates undervoltage, overtemperature and short circuit by an L-level. 
//This is an open drain output. Therefore an internal pull-up must be activated at pin PD6 if the status is to be used.

void ThrottleValveInit()
{
	RCC->APB1ENR |= 0x00000002;
	GPIOB->MODER &= ~0xF;
	GPIOB->MODER |= 0xA; //Setting PB0 and PB1 as alternate function
	GPIOB->AFR[0]|= 0x22; //We need alternate Function 2

	TIM3->PSC   = 83; 						//Prescaler is 1MHz = 1uSec
	TIM3->ARR   = 999; 						//Interrupt in every 1ms
	TIM3->CR1   = 1; 							//Setting CEN(Counter Enable Bit)
	TIM3->CCMR2 = 0x6060; 			  //At CCMR2(Capture/Compare) Canal 3 and 4 -> Setting OCyM  110. Select the output compare mode by writing OCyM bits in CCMRy register (PWM1 or PWM2 mode)
	TIM3->CCER  = 0x1100; 				//Switching on Canal 3 and 4: CCyE=1 to unlock the  Compare Mode

	//TIM3->EGR   = 1;						//Event Generator Register
	//TIM3->DIER  = 1;				  	//DMA1 Interrupt Register
	GPIOC->MODER |= 0x00400000;   //PC11 will be set as output
	GPIOC->ODR   |= 0x0800;       //Switching on the current on PC11
}

int greenledState = 1;
int boardLedIndex = 0;
/*------------------------------------
MAIN function
*-----------------------------------*/
int main (void) {
 	mcpr_SetSystemCoreClock(); //has to be first line in main!
	setupUserButton();
	setupGreenLed();
	LCD_Init();
	LCD_ClearDisplay(0xFFFF);
	set_keypad_matrix();
	InitTimer7();
	GPIOD->MODER |= (1<<26);
	
	temperatureReadCircuitInit();
	Emulate_RKTYC(25);
	
	AnalogDigitalConvertorInit();
	
	ThrottleValveInit();
	
	
	while (1) { //loop forever
		Emulate_LEDsLCD (20, 60);
		LCD_WriteString("HiWorld", 0x00, 0x00, 0x001F, 0xFFFF);
		
		LCD_SetCursor(5,16);
		LCD_DrawPixel(0xFF80);
		
		LCD_SetCursor(7,16);
		LCD_DrawPixel(0xFF80);
		if(getUserButtonState()) {
			setGreenLed(greenledState);
			greenledState = 1 - greenledState;
			
			setBoardLed(boardLedIndex);
			boardLedIndex = (boardLedIndex == 15) ? 0 : boardLedIndex + 1;
			
			busyWait_us(500000);
		}
		else
		{
			*(unsigned short volatile *) 0x60120000 =  0;
			setGreenLed(0);
		}
		
		keyold=key;
		key=read_key();
		currentlypressed= key & (~keyold);
		currentlyreleased=(~key) & keyold;
		
		 Short2BitString(pointer, key);
		 LCD_WriteString(pointer, 10, 10, 0xF800, 0x0000);
		 
		 //Toggle the green LEDs
		 if ((currentlypressed & 0x00FF)!=0) {
			 if ((GPIOD->ODR & (1<<12))==0) {
					GPIOD->ODR |= (1<<12);
				}
				else {
						GPIOD->ODR &= ~(1<<12);
				}
			}
		 if ((currentlyreleased & 0xFF00)!=0) {
			 if ((GPIOD->ODR & (1<<12))==0) {
					GPIOD->ODR |= (1<<12);
				}
				else {
						GPIOD->ODR &= ~(1<<12);
				}
			}
		sprintf(Temp, "T: %d C", readTemperature());
		LCD_WriteString(Temp, 52, 0, 0x001F, 0xF800);
		sprintf(Time, "t: %dus", delta_T);
		LCD_WriteString(Time, 160, 0, 0x001F, 0xF800);
	
			
			
	  AnalogDigitalConvertorValue();
		
		//2^12=4096 >> The ADC values will be between 0 and 4095
		sprintf(PTS_display, "%3d%%", potentiometer_1*100/4095);
		LCD_WriteString(PTS_display, 0, 16, 0x001F, 0xF800);
		sprintf(PTS_display, "%3d%%", potentiometer_2*100/4095);
		LCD_WriteString(PTS_display, 50, 16, 0x001F, 0xF800);
			
			
		ThrottleValveInit();
		TIM3->CCR3 = potentiometer_1*1000/4095; //0.244200 ~ 255
		TIM3->CCR4 = potentiometer_2*1000/4095;
	}
}



void LCD_WriteReg(unsigned short command, unsigned short data){
	LCD_WriteCommand=command;
	LCD_WriteData=data;
}

void LCD_Init(void){
	
	GPIOD->MODER |= (1<<26);			//Output PD13
	GPIOD->MODER |= (1<<6);				//Output PD3
	GPIOD->ODR |= (1<<13);	      //Backlight on
	
	GPIOD->ODR |= (1<<3);
	GPIOD->ODR &= ~(1<<3);											//0 at PD3
	busyWait_us(15);
	GPIOD->ODR |= (1<<3);
	
LCD_WriteReg(0x0010, 0x0001); /* Enter sleep mode */
LCD_WriteReg(0x001E, 0x00B2); /* Set initial power parameters. */
LCD_WriteReg(0x0028, 0x0006); /* Set initial power parameters. */
LCD_WriteReg(0x0000, 0x0001); /* Start the oscillator.*/
LCD_WriteReg(0x0001, 0x72EF); /* Set pixel format and basic display orientation */
LCD_WriteReg(0x0002, 0x0600);
LCD_WriteReg(0x0010, 0x0000); /* Exit sleep mode.*/
busyWait_us(30000); 					//30ms warten // weniger geht meist auch
LCD_WriteReg(0x0011, 0x6870); /* Configure pixel color format and MCU interface parameters.*/
LCD_WriteReg(0x0012, 0x0999); /* Set analog parameters */
LCD_WriteReg(0x0026, 0x3800);
LCD_WriteReg(0x0007, 0x0033); /* Enable the display */
LCD_WriteReg(0x000C, 0x0005); /* Set VCIX2 voltage to 6.1V.*/
LCD_WriteReg(0x000D, 0x000A); /* Configure Vlcd63 and VCOMl */
LCD_WriteReg(0x000E, 0x2E00);
LCD_WriteReg(0x0044, (240-1) << 8); /* Set the display size and ensure that the GRAM window
																				is set to allow access to the full display buffer.*/
LCD_WriteReg(0x0045, 0x0000);
LCD_WriteReg(0x0046, 320-1);
LCD_WriteReg(0x004E, 0x0000); /*Set cursor to 0,0 */
LCD_WriteReg(0x004F, 0x0000);
}

void LCD_SetCursor (uint32_t x, uint32_t y) {
	 LCD_WriteReg(0x004E, x);
	 LCD_WriteReg(0x004F, y);
}

void LCD_DrawPixel (uint16_t color) {
	LCD_WriteReg(0x0022, color);
}

void LCD_ClearDisplay(uint16_t color){
	LCD_SetCursor(0x0000, 0x0000);
	unsigned int i=0;
	while(i<=320*240){
		LCD_DrawPixel(color);
		i++;
	}
	i=0;
}

void LCD_WriteLetter (uint32_t x, uint32_t y, uint16_t textcolor, uint16_t backgroundcolor, char code) {
	unsigned int offset=code*32;
	unsigned int reihe;
	LCD_SetCursor(x, y);
	for (int i=0; i<16; i++) {
		reihe=(console_font_12x16  [offset+2*i]<<4) |(console_font_12x16[offset+2*i+1]>>4);
		for (int j=11; j>=0; j--) {
			if (reihe&(1<<j)) {
				LCD_DrawPixel(textcolor);
			}
			else {
				LCD_DrawPixel(backgroundcolor);
			}
		}
		LCD_SetCursor(x, y+i);
	}
}

void LCD_WriteString(char text[], uint32_t x, uint32_t y, uint16_t textcolor, uint16_t backgroundcolor) {
	int i=0;
	while (text[i]!='\0') {
		LCD_WriteLetter (x, y, textcolor, backgroundcolor, text[i]);
		x=x+12;
		if (x>308) {
			x=0;
			y=y+16;
		}
		i++;
	}
}


void InitTimer7(void){
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM7_STOP;

	RCC->APB1ENR |=(1<<5);

	TIM7->PSC = 83;					//Prescaler divides timer 7 CLK by PSC+1
	TIM7->ARR = 999;				//ARR + 1 Ticks until the timer overflows -> 1 MHz
	
	TIM7->CR1|=5;
	TIM7->DIER|=1;
	TIM7->EGR|=1;
	
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, 4);	
}

uint32_t timer_counter = 0;
short toggle_flag = 0;
#define BACKGROUND_TOGGLE_MSEC 1000
#define LED_SHIFT_MSEC (10000UL / 16UL)
void TIM7_IRQHandler (void){
	TIM7->SR=0x0000;
	if(getUserButtonState()){
		
		if(timer_counter > 0) {
			if((timer_counter % BACKGROUND_TOGGLE_MSEC) == 0) {
			// screen toggle
				toggleBackground (1);
				//toggle_flag = 1 - toggle_flag;
			}
			if((timer_counter % LED_SHIFT_MSEC) == 0) {
				setGreenLed(greenledState);
				greenledState = 1 - greenledState;
			
				setBoardLed(boardLedIndex);
				boardLedIndex = (boardLedIndex == 15) ? 0 : boardLedIndex + 1;
			}
		}
		timer_counter++;
	}
	else {
		*(unsigned short volatile *) 0x60120000 =  0;
		setGreenLed(0);
		toggleBackground (0);
		timer_counter = 0;
	}
}

void toggleBackground (short t) {
	if (t==1) {
		if ((GPIOD->ODR & (1<<13))!=0) {		//Toggle when t=1
			GPIOD->ODR &= ~(1<<13);
		}
		else {
			GPIOD->ODR |= (1<<13);
		}
	}
	else {																//Always switch on background
		GPIOD->ODR |= (1<<13);
	}
}
