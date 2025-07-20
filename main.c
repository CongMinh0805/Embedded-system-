//Exercise 3: Battleship game
#include <stdio.h>
#include "NUC100Series.h"
#include "SYS_init.h"
#include "LCD.h"

//Function declaration
void System_Config(void); //setup system fuction
void SPI3_Config(void); //setup SPI3 fuction
void UART0_Config(void); //setup UART fuction
void GPIO_Config(void); //setup GPIO fuction

void EINT1_IRQHandler(void); // ISR for GPIOB15
void TMR0_IRQHandler(void); // ISR for TIMER0
void UART02_IRQHandler(void); // ISR for UART
void Buzzer_beep(int beep_time); 

void LCD_start(void);
void LCD_command(unsigned char temp);
void LCD_data(unsigned char temp);
void LCD_clear(void);
void LCD_SetAddress(uint8_t PageAddr, uint8_t ColumnAddr);

void KeyPadEnable(void);
uint8_t KeyPadScanningX(void);
uint8_t KeyPadScanningY(void);

//Main function for battleship game
void welcomeScreen(void);
void startScreen(void);
void setCoordinate(void);
void shoot(void);
void WinStatus(void);	
void LoseStatus(void);
void remain_shot_realTime(int i);
void displayX(void);
void displayY(void);
char ReceiveByte(void);

//Variable declaration
char map[8][8]; // Map to store the loaded map
char pos_map[8][8]; //position map

volatile int num_of_ship_sunk = 0; // Num of ship sunk
volatile int shots = 16; // Num of shots
volatile int NewData = 0; // NewData = False: replace boolean value true/ false for data loaded
volatile int isLoadMap = 0; //isLoadMap = FALSE: replace boolean value true/ false for loading map
volatile int coordinate_set = 0; //coordinate_set = FALSE: replace boolean value true/ false for changing x-y cordinator				
volatile int x = 0; // X-coordinate value
volatile int y = 0; // Y-coordinate value
volatile char data; // Data (1 byte) received from UART
volatile int button_pressed = 0;
volatile int Segment_Display = 1; // 7 Segment selection

enum state
{
	Welcome,
	Start,
	ChoosingXY,
	Shoot,
	GameWin,
	GameLose
};
enum state state;

static int pattern[] = {
 // gedbaf_dot_c
	0b10000010,  // Number 0          // ---a----
  0b11101110,  // Number 1          // |      |
  0b00000111,  // Number 2          // f      b
  0b01000110,  // Number 3          // |      |
  0b01101010,  // Number 4          // ---g----
  0b01010010,  // Number 5          // |      |
  0b00010010,  // Number 6          // e      c
  0b11100110,  // Number 7          // |      |
	0b00000010,  // Number 8          // ---d----
  0b01000010,  // Number 9
  0b11111111   // Blank LED 
};  


int main (void)
{
	System_Config();
	SPI3_Config();
	GPIO_Config();
	UART0_Config();
	
	LCD_start();
	LCD_clear();
	
	state = Welcome; // Default state
	
	while(1)
	{
		switch (state)
		{
			case Welcome:
				welcomeScreen();
				break;	
			case Start:
				startScreen();
				break;		
			case ChoosingXY:
				setCoordinate();
				break;			
			case Shoot:
				shoot();
				break;			
			case GameWin:
				WinStatus();
				break;			
			case GameLose:
				LoseStatus();
				break;			
			default:
				break;
		}
	}
}

void EINT1_IRQHandler(void) // ISR for GPIOB15
{
	button_pressed++;
	CLK_SysTickDelay(50000); // Button debounce 
	PB->ISRC |= (1<<15); // Clear flag
}

void TMR0_IRQHandler(void) // ISR for TIMER0
{
	int dozen = shots / 10; //get the second (dozen) of  number of shot
	int unit = shots % 10; //to get last decimal (unit) of number of shot
	
	if (Segment_Display == 0) //display coordinate
	{
		PC->DOUT |= (1<<7); // Turn on 7segement U11
		PC->DOUT &= ~(1<<6);	
		PC->DOUT &= ~(1<<5);		
		PC->DOUT &= ~(1<<4);
		
		if (coordinate_set == 0)  //display x-coordinate
		{
			displayX();
		}
		else if (coordinate_set == 1)  //display y-coordinate
		{
			displayY();
		}
		Segment_Display = 1; // Transition to the next 7segment display
	}
	
	else if (Segment_Display == 1)  //display number of remaining shot
	{
		PC->DOUT &= ~(1<<7); 
		PC->DOUT &= ~(1<<6);		
		PC->DOUT |= (1<<5); // Turn on 7segement U13
		PC->DOUT &= ~(1<<4);	
		
		remain_shot_realTime(dozen); // Display the remaining shot available
		Segment_Display = 2; // Transition to the next 7segment display
	}
	
	else if (Segment_Display == 2) // Condition to display number of remaining shot
	{
		PC->DOUT &= ~(1<<7); 
		PC->DOUT &= ~(1<<6);		
		PC->DOUT &= ~(1<<5);		
		PC->DOUT |= (1<<4);	// Turn on 7segement U14
		
		remain_shot_realTime(unit); // Display the remaining shot available
		Segment_Display = 0; // Transition to the next 7segment display
	}
	TIMER0->TISR |= (1<<0);	// Clear flag
}

void UART02_IRQHandler(void) // ISR for UART
{
	data = ReceiveByte(); // Receive new data
	if (data == '1' || data == '0') // Condition to store new data into the array
	{
		NewData = 1; // New data = TRUE
	}
}

char ReceiveByte(void)
{
	return (char) UART0->DATA; // Receive new data in char type
}

void welcomeScreen(void) // Welcome screen state
{
	int i = 0; // Column in array map[] 
	int j = 0; // Row in array map[]
	int size = 0; // Maximum size of the map is 64 (8x8)
	printS_5x7(40, 15, "Welcome to");
	printS_5x7(25, 26, "Battle Ship Game!");
	printS_5x7(30, 45, "Load your map!");
	
	while (1)
	{
		if (NewData && size != 64) // Condition to store new data
		{
			map[i][j] = data; // Store battle map data into the array map[]
			pos_map[i][j] = 0; //inial value of pos_map is full 0
			i++; // Go to new column
			if (i == 8) // Condition to move to the next row
			{
				i = 0; // Reset back to the first collumn if we reached the last one
				j++; // Move to next row
			}
			NewData = 0; //New data = FALSE
			size++;
		}
		else if (size == 64) // Condition to stop receiving new data 
		{
			isLoadMap = 1; // Confirm map is loaded
		}	
		if (isLoadMap == 1) // Map loaded successfully (TRUE)
		{
			state = Start; // change to the next state
			LCD_clear();
			break;
		}
		if (button_pressed > 0)
		{
			button_pressed = 0;
		}
	}
}
// Display map load sucessfully 
void startScreen(void) {
	printS_5x7(0, 25, "Map loaded successfully!");
	printS_5x7(0, 35, "Press SW_INT1 to start");
	
	while (1)
	{
		if (button_pressed > 0) //start game
		{
			state = ChoosingXY; // change to the Set Coordinate state
			button_pressed = 0;
			LCD_clear();
			break;
		}
	}
}
// Set X-Y coordinator 
void setCoordinate(void) {
	int16_t x = 0; // Move to the next column in array pos_map[]
	int16_t y = 0; // Move to the next row in array pos_map[]
	int i = 0; // column in array pos_map[]
	int j = 0; // row in array pos_map[]
	int size = 0;
	int count = 0; //for switching x-y
	TIMER0->TCSR |= (1<<30); // Start TIMER0 counting
	while (1)
	{	
		while (size < 64) // Print the field on LCD
		{
			if (pos_map[i][j] == 0) //if all of the value of position map are FALSE
			{
				printC_5x7(45+x, y, '-'); // Print "-" for the field
				printS_5x7 (0,20, "Ship:");					
			}
			if (pos_map[i][j] == 1) // if the position already shot (the value is TRUE)
			{
				printC_5x7(45+x, y, 'X');// Print "X" for the field
				printS_5x7 (0,20, "Ship:");
				printC_5x7(10,25, num_of_ship_sunk + '0');				
			}
			i++; // go to new column in the array
			x = x + 8; // move to the next column of the field (on LCD)
			if (i == 8)
			{
				y = y + 8; // move to the next row of the field (on LCD)
				j++; // move to next row in the array
				x = 0;
				i = 0;
			}
			size++;
		}
		
		PA0 = 0; PA1 = 1; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
		if (PA5 == 0) // Check if button 9 is pressed
		{
			CLK_SysTickDelay(50000); // Button debounce
			count++;
		}
		
		if (count % 2 == 0) // switch to X-coordinate from Y-coordinate
		{
			KeyPadScanningX(); // Scan for X-coordinate button press
			coordinate_set = 0; // X-coordinate
			PC->DOUT |= 1<<15; 
			PC->DOUT &= ~(1<<14); // Turn on LED7
		}
		
		if (count %2 != 0)// switch to Y-coordinate from X-coordinate
		{
			KeyPadScanningY(); // Scan for Y-coordinate button press
			coordinate_set = 1; // Y-coordinate
			PC->DOUT |= 1<<14; 
			PC->DOUT &= ~(1<<15); // Turn on LED8
		}
		
		if (button_pressed > 0) //if button is press
		{
			state = Shoot; // change to the Shot phase
			button_pressed = 0;
			shots--; // Reduce number of shot
			LCD_clear();
			break;
		}
	}
}
// Shoot function
void shoot(void) {
	PC->DOUT |= (1<<15);
	PC->DOUT |= (1<<14);
	while (1)
	{
		if (map[x][y] == '1' && shots > 0) // Hit condition
		{ 
			pos_map[x][y] = 1; // Replace 0 with 1 in position_map array
			
			for (int i = 0; i < 6; i++) // LED5 toggle 3 times
			{
				PC->DOUT ^= 1<<12;
				CLK_SysTickDelay(20000);
			}
			//check 4 surrounding point
			if (pos_map[x-1][y] == 1 || pos_map[x+1][y] == 1) //left and right 
			{
				num_of_ship_sunk++; // ship sunk++
			}
			if (pos_map[x][y-1] == 1 || pos_map[x][y+1] == 1) //above and below
			{
				num_of_ship_sunk++; // ship sunk++
			} 
			x = 0; // Reset X-coordinate
			y = 0;  // Reset Y-coordinate
			
			if (num_of_ship_sunk == 5) // Win condition (hit all 5 ships)
			{
				state = GameWin; // change to the win state
				LCD_clear();
				break;
			}		
			state = ChoosingXY; // Back to set coordinate state
			LCD_clear();
			break;
		}		
		if (shots > 0) // Miss condition
		{
			x = 0;  // Reset X-coordinate
			y = 0;  // Reset Y-coordinate
			state = ChoosingXY; // Back to set coordinate state
			clear_LCD();
			break;
		}	
		if (shots == 0 && num_of_ship_sunk < 5) // Lost condition (do not sink all the ships and run out of shot)
		{
			state = GameLose; // change to the game over state
			LCD_clear();
			break;
		}		
	}
}
// Game win status
void WinStatus (void) { 
	TIMER0->TCSR &= ~(1<<30); // Stop TIMER0 counting
	//Turn off 4 7-segment leds
	PC->DOUT &= ~(1<<7); 
	PC->DOUT &= ~(1<<6);		
	PC->DOUT &= ~(1<<5);		
	PC->DOUT &= ~(1<<4);		
	Buzzer_beep(5); //beep 5 times
	printS_5x7(40, 20, "You win!"); // Display game win message
	
	while (1)
	{	
		if (button_pressed > 0) //if button is pressed
		{
			state = Welcome; // change to the Welcome state
			button_pressed = 0;
			shots = 16; // reset the number of shot
			num_of_ship_sunk = 0; // reset the number of ship sunk
			isLoadMap = 0; //isLoadMap = False
			x = 0;  // reset y
			y = 0;  // reset x
			
			LCD_clear();
			break;
		}
	}
}

// Game lose status
void LoseStatus(void) {
	TIMER0->TCSR &= ~(1<<30); // Stop TIMER0 counting
	//Turn off 4 7-segment leds
	PC->DOUT &= ~(1<<7);
	PC->DOUT &= ~(1<<6);		
	PC->DOUT &= ~(1<<5);		
	PC->DOUT &= ~(1<<4);	
	Buzzer_beep(5); //beep 5 times
	printS_5x7(40, 20, "GAME OVER!"); // Display game over message
	while(1)
		{
			if (button_pressed > 0) //if button is press
				{
					state = Welcome; // change to the Welcome state
					button_pressed = 0;
					shots = 16; // reset the number of shot
					num_of_ship_sunk = 0; // reset the number of ship sunk
					isLoadMap = 0; //isLoadMap = False
					x = 0;  // reset x
					y = 0;  // reset y
					
					LCD_clear();
					break;
				}
		}
}
// Display num of shots
void remain_shot_realTime(int i) {
	if (i == 0){
		PE->DOUT = pattern[0];
	}else if (i == 1){
		PE->DOUT = pattern[1];
	}else if (i == 2){
		PE->DOUT = pattern[2];
	}else if (i == 3){
		PE->DOUT = pattern[3];
	}else if (i == 4){
		PE->DOUT = pattern[4];
	}else if (i == 5){
		PE->DOUT = pattern[5];
	}else if (i == 6){
		PE->DOUT = pattern[6];
	}else if (i == 7){
		PE->DOUT = pattern[7];
	}else if (i == 8){
		PE->DOUT = pattern[8];
	}else if (i == 9){
		PE->DOUT = pattern[9];
	}	
}
// display X
void displayX(void) {
	if (x == 0){
		PE->DOUT = pattern[1];
	}else if (x == 1){
		PE->DOUT = pattern[2];
	}else if (x == 2){
		PE->DOUT = pattern[3];
	}else if (x == 3){
		PE->DOUT = pattern[4];
	}else if (x == 4){
		PE->DOUT = pattern[5];
	}else if (x == 5){
		PE->DOUT = pattern[6];
	}else if (x == 6){
		PE->DOUT = pattern[7];
	}else if (x == 7){
		PE->DOUT = pattern[8];
	}	
}
// display Y
void displayY(void) {
	if (y == 0){
		PE->DOUT = pattern[1];
	}else if (y == 1){
		PE->DOUT = pattern[2];
	}else if (y == 2){
		PE->DOUT = pattern[3];
	}else if (y == 3){
		PE->DOUT = pattern[4];
	}else if (y == 4){
		PE->DOUT = pattern[5];
	}else if (y == 5){
		PE->DOUT = pattern[6];
	}else if (y == 6){
		PE->DOUT = pattern[7];
	}else if (y == 7){
		PE->DOUT = pattern[8];
	}	
}
void System_Config(void) // Clock and Timer configuration
{
	SYS_UnlockReg(); // Unlock protected registers
  
	CLK->PWRCON |= (1<<0); // 12 Mhz CLK
	while(!(CLK->CLKSTATUS & 1<<0)); // wait the CLK to be stable
    
  CLK->CLKSEL0 &= ~(0b111<<0); // Clear all bits - select bit for 12 Mhz clk
	CLK->PWRCON &= ~(1<<7); // Normal mode
  CLK->CLKDIV &= ~(0x0F<<0); // Clock divider is 1
	
	// UART0 enable
	CLK->CLKSEL1 &= ~(0b11<<24); // UART0 clock source is 22.1184 MHz
	CLK->CLKSEL1 |= (0b11<<24); // UART0 clock source is 22.1184 MHz
	CLK->CLKDIV &= ~(0xF<<8); // Clock divider is 1
	CLK->APBCLK |= (1<<16); // Enable UART0 clock
  
	// SPI3 enable
  CLK->APBCLK |= (1<<15); // SPI3 enable
	
	// TIMER0 configuration
	CLK->CLKSEL1 &= ~(0b111<<8); // Select bit for 12Mhz CLK in TIMER0
	CLK->APBCLK |= (1<<2); // Enable TIMER0
	
	TIMER0->TCSR &= ~(0xFF<<0); // Clear all bits
	TIMER0->TCSR |= (1<<26); // Reset TIMER0
	
	TIMER0->TCSR &= ~(0b11<<27); // Clear bits
	TIMER0->TCSR |= (0b01<<27); // Periodic mode
	TIMER0->TCSR &= ~(1<<24); // Timer mode
	
	TIMER0->TCSR |= (1<<29); // Enable interrupt bit
	
	TIMER0->TCMPR = 5999; // (0.0005 / (1 / 12000000)), frequency: 2000Hz
	
	NVIC->ISER[0] |= (1<<8); // Enable control register
	NVIC->IP[2] |= (3<<6); // Set priority
	
	SYS_LockReg();  // Lock protected registers    
}

void SPI3_Config(void) // SPI3 configuration
{
	SYS->GPD_MFP |= (1<<11); // 1: PD11 is configured for alternative function
  SYS->GPD_MFP |= (1<<9); // 1: PD9 is configured for alternative function
  SYS->GPD_MFP |= (1<<8); // 1: PD8 is configured for alternative function
 
  SPI3->CNTRL &= ~(1<<23); // 0: disable variable clock feature
  SPI3->CNTRL &= ~(1<<22); // 0: disable two bits transfer mode
  SPI3->CNTRL &= ~(1<<18); // 0: select Master mode
  SPI3->CNTRL &= ~(1<<17); // 0: disable SPI interrupt    
  SPI3->CNTRL |= (1<<11); // 1: SPI clock idle high 
  SPI3->CNTRL &= ~(1<<10); // 0: MSB is sent first   
  SPI3->CNTRL &= ~(3<<8); // 00: one transmit/receive word will be executed in one data transfer
   
  SPI3->CNTRL &= ~(31<<3); // Transmit/Receive bit length
	SPI3->CNTRL |= (9<<3); // 9: 9 bits transmitted/received per data transfer
    
  SPI3->CNTRL |= (1<<2);  // 1: Transmit at negative edge of SPI CLK       
  SPI3->DIVIDER = 0; // SPI clock divider. SPI clock = HCLK / ((DIVIDER+1)*2). HCLK = 50 MHz
}

void UART0_Config(void) // UART0 configuration
{
	// UART0 pin configuration. PB.1 pin is for UART0 TX
	PB->PMD &= ~(0b11 << 2);
	PB->PMD |= (0b01 << 2); // PB.1 is output pin
	SYS->GPB_MFP |= (1 << 1); // GPB_MFP[1] = 1 -> PB.1 is UART0 TX pin
	SYS->GPB_MFP |= (1 << 0); // GPB_MFP[0] = 1 -> PB.0 is UART0 RX pin	
	PB->PMD &= ~(0b11 << 0);	// Set Pin Mode for GPB.0(RX - Input)

	// UART0 operation configuration
	UART0->LCR |= (0b11 << 0); // 8 data bit
	UART0->LCR &= ~(1 << 2); // one stop bit	
	UART0->LCR &= ~(1 << 3); // no parity bit
	UART0->FCR |= (1 << 1); // clear RX FIFO
	UART0->FCR |= (1 << 2); // clear TX FIFO
	UART0->FCR &= ~(0xF << 16); // FIFO Trigger Level is 1 byte]
	
	//UART0 interrupt configuration
	UART0->IER |= (1 << 0); //Recieve data available interrupt enabled
	NVIC->ISER[0] |= (1 << 12);
	NVIC->IP[3] &= (~(0b11 << 6));
	
	//Baud rate config: BRD/A = 1, DIV_X_EN=0
	//--> Mode 0, Baud rate = UART_CLK/[16*(A+2)] = 22.1184 MHz/[16*(1+2)]= 9600 bps
	UART0->BAUD &= ~(0b11 << 28); // mode 0	
	UART0->BAUD &= ~(0xFFFF << 0);
	UART0->BAUD |= 142;
}

void GPIO_Config(void) // GPIO configuration
{
	PB->PMD &= ~(0b11<<22); // Clear all bits of PB11
	PB->PMD |= (0b01<<22); // Output push-pull for PB11
	
	PC->PMD &= ~(0b11<<24); // Clear LED5
	PC->PMD |= (0b01<<24); // Output Push-pull LED5
	
	PC->PMD &= ~(0b11<<28); // Clear LED7
	PC->PMD |= (0b01<<28); // Output Push-pull LED7
	
	PC->PMD &= ~(0b11<<30); // Clear LED8
	PC->PMD |= (0b01<<30); // Output Push-pull LED8
	
	PC->PMD &= ~(0xFF<<8); // Clear 4-7 segments
	PC->PMD |= (0b01010101<<8); // Output Push-pull 7 segments
	
	PE->PMD &= ~(0xFFFF<<0); // Clear all digits in 7 seg
	PE->PMD |= (0b0101010101010101<<0); // Output Push-pull for all digits in 7 segments
	
	PB->PMD &= ~(0b11<<30); // Input-PB15
	PB->IMD &= ~(1<<15);
	PB->IEN |= (1<<15); // Enable interrupt
	NVIC->ISER[0] |= (1<<3); // Enable control register
	NVIC->IP[0] &= ~(3<<30); // Set priority
}

void LCD_start(void)
{
	LCD_command(0xE2); // Set system reset
	LCD_command(0xA1); // Set Frame rate 100 fps  
  LCD_command(0xEB); // Set LCD bias ratio E8~EB for 6~9 (min~max)  
  LCD_command(0x81); // Set V BIAS potentiometer
	LCD_command(0xA0); // Set V BIAS potentiometer: A0 ()        	
  LCD_command(0xC0);  
	LCD_command(0xAF); // Set Display Enable
}

void LCD_command(unsigned char temp)
{
	SPI3->SSR |= (1<<0);  
	SPI3->TX[0] = temp;
	SPI3->CNTRL |= (1<<0);
	while(SPI3->CNTRL & (1<<0));
	SPI3->SSR &= ~(1<<0);
}

void LCD_data(unsigned char temp)
{
	SPI3->SSR |= (1<<0);  
	SPI3->TX[0] = 0x0100 + temp;
	SPI3->CNTRL |= (1<<0);
	while(SPI3->CNTRL & (1<<0));
	SPI3->SSR &= ~(1<<0);
}

void LCD_clear(void)
{	
	int16_t i;
	LCD_SetAddress(0x0, 0x0);			  								  
	for (i = 0; i < 132 *8; i++)
	{
		LCD_data(0x00);
	}
}

void LCD_SetAddress(uint8_t PageAddr, uint8_t ColumnAddr)
{
	LCD_command(0xB0 | PageAddr);
  LCD_command(0x10 | (ColumnAddr>>4) & 0xF); 
  LCD_command(0x00 | (ColumnAddr & 0xF));
}

void KeyPadEnable(void)
{
	GPIO_SetMode(PA, BIT0, GPIO_MODE_QUASI);
  GPIO_SetMode(PA, BIT1, GPIO_MODE_QUASI);
  GPIO_SetMode(PA, BIT2, GPIO_MODE_QUASI); 
  GPIO_SetMode(PA, BIT3, GPIO_MODE_QUASI);
  GPIO_SetMode(PA, BIT4, GPIO_MODE_QUASI);
  GPIO_SetMode(PA, BIT5, GPIO_MODE_QUASI);
}

uint8_t KeyPadScanningX(void) // Scan for x-coordinate
{
	PA0 = 1; PA1 = 1; PA2 = 0; PA3 = 1; PA4 = 1; PA5 = 1;
  if (PA3 == 0){
		x = 0;
	}
	if (PA4 == 0){
		x = 3;
	}
  if (PA5 == 0) {
		x = 6;
	}
	PA0 = 1; PA1 = 0; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
  if (PA3 == 0){
		x = 1;
	}
  if (PA4 == 0){
		x = 4;
	}
  if (PA5 == 0){
		x = 7;
	}
	PA0 = 0; PA1 = 1; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
  if (PA3 == 0) {
		x = 2;
	}
  if (PA4 == 0){
		x = 5;
	}
  return 0;
}

uint8_t KeyPadScanningY(void) // Scan for y-coordinate
{
	PA0 = 1; PA1 = 1; PA2 = 0; PA3 = 1; PA4 = 1; PA5 = 1;
  if (PA3 == 0){
		y = 0;
	}
  if (PA4 == 0){
		y = 3;
	}
  if (PA5 == 0) {
		y = 6;
	}
  
	PA0 = 1; PA1 = 0; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
  if (PA3 == 0){
		y = 1;
	}
  if (PA4 == 0){
		y = 4;
	}
  if (PA5 == 0){
		y = 7;
	}
	PA0 = 0; PA1 = 1; PA2 = 1; PA3 = 1; PA4 = 1; PA5 = 1;
  if (PA3 == 0) {
		y = 2;
	}
  if (PA4 == 0){
		y = 5;
	}
  return 0;
}

void Buzzer_beep(int beep_time) {
	int i;
	for (i = 0; i < (beep_time * 2); i++) {
		PB->DOUT ^= (1 << 11);
		PC->DOUT ^= 1 << 12;
		CLK_SysTickDelay(40000);
	}
}
