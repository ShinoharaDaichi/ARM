#include "LPC17xx.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "Board_LED.h"
#include "Board_Buttons.h"
#include "Board_Joystick.h"
#include "Board_ADC.h"
#include "Board_GLCD.h"
#include "GLCD_Config.h"

extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;

/* Joystick position definitions */
#define JOY_X        (9*16)
#define JOY_Y        (5*24 + 6)

/* Buttons bit masks */
#define BTN_INT0     (1 << 0)

/* GLCD string buffer */
#define STRINGBUF_LEN 21

char            StringBuf[STRINGBUF_LEN];

/* Extern variables */
extern uint8_t  Arrows_16bpp_red[];
extern uint8_t  Button_16bpp[];
extern uint8_t  Bulb_16bpp[];

/* Static variables */
static uint32_t LEDOn, LEDOff;
static int32_t  LEDDir = 1;

char stringAdc[32];
char stringLed[10];

double map(double input, double input_start, double input_end, double output_start, double output_end)
{
    return (output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start));
}

static uint32_t triangle[32];

static void T0_Init(void)
{
	/* select MAT0.0 at P1.28 */
    LPC_PINCON->PINSEL3 |= (3<<24); 

	/* 100 usec @ 25 Mhz */
    LPC_TIM0->MR0 = 2500;

	/* interrupt on MR0, reset timer on match 0 */
    LPC_TIM0->MCR = 0x0003;
	
	/* toggle MAT0.0 pin on match */
    LPC_TIM0->EMR = 0x0031;

	/* Reset Timer */
    LPC_TIM0->TCR  = 2;
}

void ADC_Init(void)
{
	/* Turn on the ADC */
	LPC_SC->PCONP 	|= (1<<12);	
	LPC_ADC->ADCR	=	(1<<2) | (0x4<<8) | (1<<21);
	LPC_SC->PCLKSEL0 &= ~(0x3<<24);	// PCLK = clk/4	 100MHz / 4 = 25MHz

	/* Select P0.25 as AD0.2 for input */
	LPC_PINCON->PINSEL1	 = (LPC_PINCON->PINSEL1  & ~(0x3<<18)) | (1<<18);

	/* Disable Pullup and Pulldown resistors */
	LPC_PINCON->PINMODE1 = (LPC_PINCON->PINMODE1 & ~(0x3<<18)) | (0x2<<18);

	return;
}

void Start_Converstion(void){
	LPC_ADC->ADCR	|=	(1<<24);
	return;
}

uint32_t Get_ADC_value(void){
	/* Wait for conversion to complete */
	while(!(LPC_ADC->ADSTAT & (1<<2)));
	/* Return the conversion value */
	return ((LPC_ADC->ADDR2 >> 4) & (0xFFF));
}

void Beep()
{
        LPC_GPDMACH0->DMACCSrcAddr  = (uint32_t) &triangle[0];
        LPC_GPDMACH0->DMACCDestAddr = (uint32_t) &(LPC_DAC->DACR);
        LPC_GPDMACH0->DMACCLLI      = 0;           // linked lists for ch0
        LPC_GPDMACH0->DMACCControl  = 32           // transfer size (0 - 11) = 32
                                | (0 << 12)        // source burst size (12 - 14) = 1
                                | (0 << 15)        // destination burst size (15 - 17) = 1
                                | (2 << 18)        // source width (18 - 20) = 32 bit
                                | (2 << 21)        // destination width (21 - 23) = 32 bit
                                | (0 << 24)        // source AHB select (24) = AHB 0
                                | (0 << 25)        // destination AHB select (25) = AHB 0
                                | (1 << 26)        // source increment (26) = increment
                                | (0 << 27)        // destination increment (27) = no increment
                                | (0 << 28)        // mode select (28) = access in user mode
                                | (0 << 29)        // (29) = access not bufferable
                                | (0 << 30)        // (30) = access not cacheable
                                | (0 << 31);       // terminal count interrupt disabled

        LPC_GPDMACH0->DMACCConfig   =  1           // channel enabled (0)
                                | (0 << 1)         // source peripheral (1 - 5) = none
                                | (8 << 6)         // destination request peripheral (6 - 10) = MAT0.0
                                | (1 << 11)        // flow control (11 - 13) = mem to per
                                | (0 << 14)        // (14) = mask out error interrupt
                                | (0 << 15)        // (15) = mask out terminal count interrupt
                                | (0 << 16)        // (16) = no locked transfers
                                | (0 << 18);       // (27) = no HALT
        
		LPC_TIM0->TCR = 1;                         // enable timer
        while (LPC_GPDMACH0->DMACCConfig & 1) ;    // wait for the DMA to finish
        LPC_TIM0->TCR = 0;                         // disable timer
				osDelay(500);
}

void initDMA()
{
		uint32_t i;
    for(i=0; i!=16; i++) 
		{
			triangle[i] = (((i+1)<<6) - 1)<<6;			
		}
		
    for(i=0; i!=16; i++) 
		{
			triangle[16+i] = (((16-i)<<6) - 64)<<6;
		}

	// enable AOUT (P0.26) pin
    LPC_PINCON->PINSEL1 |= (2<<20); 

	// Select MAT0.0 instead of UART0 Tx
    LPC_SC->DMAREQSEL = 0x01;

	// Enable the GPDMA controller
    LPC_GPDMA->DMACConfig = 1;

	// Enable synchro logic request 8, MAT0.0
    LPC_GPDMA->DMACSync   = 1 << 8;

}

int main (void)                                    // Main Program
{
  uint32_t i, t;
	
	int32_t BackgroundColor = GLCD_COLOR_BLACK;
	int32_t ForegroundColor = GLCD_COLOR_WHITE;
		
  SystemInit();

  T0_Init();
	//ADC_Init();
	ADC_Initialize          ();           //Initialize A/D Converter
		
	GLCD_Initialize();
	
	GLCD_SetBackgroundColor (BackgroundColor);
  GLCD_SetForegroundColor (ForegroundColor);
  GLCD_SetFont            (&GLCD_Font_16x24);
	GLCD_ClearScreen();	

	// Select P0.16 as output 
	LPC_GPIO0->FIODIR |= 0x00010000;

    for(i=0; i!=16; i++) 
		{
			triangle[i] = (((i+1)<<6) - 1)<<6;			
		}
		
    for(i=0; i!=16; i++) 
		{
			triangle[16+i] = (((16-i)<<6) - 64)<<6;
		}


	// enable AOUT (P0.26) pin
    LPC_PINCON->PINSEL1 |= (2<<20); 

	// Select MAT0.0 instead of UART0 Tx
    LPC_SC->DMAREQSEL = 0x01;

	// Enable the GPDMA controller
    LPC_GPDMA->DMACConfig = 1;

	// Enable synchro logic request 8, MAT0.0
    LPC_GPDMA->DMACSync   = 1 << 8;

		initDMA();
		
	while(1)                                    // Loop forever
    {
			
				Start_Converstion();
				sprintf (StringBuf, "%4d", ADC_GetValue());		
				StringBuf[sizeof(StringBuf)-1] = '\0'; //indicate end of string
				GLCD_DrawString(0, 0, StringBuf);
			
				/*
        LPC_GPIO0->FIOPIN ^= 1 << 16;                  // Toggle P0.16
				
        LPC_GPDMACH0->DMACCSrcAddr  = (uint32_t) &triangle[0];
        LPC_GPDMACH0->DMACCDestAddr = (uint32_t) &(LPC_DAC->DACR);
        LPC_GPDMACH0->DMACCLLI      = 0;           // linked lists for ch0
        LPC_GPDMACH0->DMACCControl  = 32           // transfer size (0 - 11) = 32
                                | (0 << 12)        // source burst size (12 - 14) = 1
                                | (0 << 15)        // destination burst size (15 - 17) = 1
                                | (2 << 18)        // source width (18 - 20) = 32 bit
                                | (2 << 21)        // destination width (21 - 23) = 32 bit
                                | (0 << 24)        // source AHB select (24) = AHB 0
                                | (0 << 25)        // destination AHB select (25) = AHB 0
                                | (1 << 26)        // source increment (26) = increment
                                | (0 << 27)        // destination increment (27) = no increment
                                | (0 << 28)        // mode select (28) = access in user mode
                                | (0 << 29)        // (29) = access not bufferable
                                | (0 << 30)        // (30) = access not cacheable
                                | (0 << 31);       // terminal count interrupt disabled

        LPC_GPDMACH0->DMACCConfig   =  1           // channel enabled (0)
                                | (0 << 1)         // source peripheral (1 - 5) = none
                                | (8 << 6)         // destination request peripheral (6 - 10) = MAT0.0
                                | (1 << 11)        // flow control (11 - 13) = mem to per
                                | (0 << 14)        // (14) = mask out error interrupt
                                | (0 << 15)        // (15) = mask out terminal count interrupt
                                | (0 << 16)        // (16) = no locked transfers
                                | (0 << 18);       // (27) = no HALT
        
		LPC_TIM0->TCR = 1;                         // enable timer
        while (LPC_GPDMACH0->DMACCConfig & 1) ;    // wait for the DMA to finish
        LPC_TIM0->TCR = 0;                         // disable timer
		
			*/
			
		t =  Get_ADC_value();
		
		if(t < 1000)
		{
			Beep();
			osDelay(500);
		}
		//for (i=0; i<(10000 * t); i++);// Delay
    }
}








/*-----------------------------------------------------------------------------
  Main function
 *----------------------------------------------------------------------------*/
		int old_paddle0_X;
		int old_paddle0_Y;
		int old_paddle1_X;
		int old_paddle1_Y;
		int old_ball_X;
		int old_ball_Y;
				
		int new_paddle0_X;
		int new_paddle0_Y;
		int new_paddle1_X;
		int new_paddle1_Y;
		int new_ball_X;
		int new_ball_Y;
		
		int ballVelocityX = 10;  // Ball left/right velocity
    int ballVelocityY = 5;  // Ball up/down velocity
    int paddle0Velocity = -1;  // Paddle 0 velocity
    int paddle1Velocity = 1;  // Paddle 1 velocity

		unsigned int ball_rad = 5;  // Ball radius
    unsigned int paddleW = 3;  // Paddle width
    unsigned int paddleH = 60;  // Paddle height
		
		unsigned int score0 = 0;
		unsigned int score1 = 0;
		
		char str_score0[10]; //string representation of score0
		char str_score1[10]; //string representation of score1
		
		unsigned int borderWidth = 1;
		bool isPaused = false;
		
/*

int main (void){
    uint32_t    ofs;
    uint32_t    btnMsk, joyMsk;
    int32_t     adcVal;
    int32_t     joy  = -1;
    int32_t     adc  = -1;
    int32_t     btn  = -1;
		int32_t BackgroundColor = GLCD_COLOR_BLACK;
	  int32_t ForegroundColor = GLCD_COLOR_WHITE;
		int32_t BallForegroundColor = GLCD_COLOR_RED;
	  int32_t LeftPaddleForegroundColor = GLCD_COLOR_BLUE;
		int32_t RightPaddleForegroundColor = GLCD_COLOR_GREEN;
	
    ADC_Initialize          ();           // Initialize A/D Converter           
    GLCD_Initialize         ();           // Initialize Graphical LCD           
    Joystick_Initialize     ();           // Initialize joystick                
    LED_Initialize          ();           // Initialize LED                     
		initDMA();
    // Prepare display for ADC, Buttons, Joystick 
    GLCD_SetBackgroundColor (BackgroundColor);
    GLCD_SetForegroundColor (ForegroundColor);
    GLCD_SetFont            (&GLCD_Font_16x24);
		GLCD_ClearScreen();		
		
		// Select P0.16 as output 
		LPC_GPIO0->FIODIR |= 0x00010000;
	
    while (1)        {
			if (!btn ^ btnMsk)
			{
				 btn = btnMsk;
         if (btnMsk & BTN_INT0) 
				 {
					 isPaused = ~isPaused;
				 }
			}
			while(!isPaused)
			{			
				GLCD_ClearScreen();	
						
				//GAME START	
						
				// Paddle 0 (left) position coordinates
				new_paddle0_Y = (GLCD_HEIGHT / 2) - (paddleH / 2);
				new_paddle0_X = borderWidth + paddleW;
				// Paddle 1 (right) position coordinates
				new_paddle1_Y = (GLCD_HEIGHT / 2) - (paddleH / 2);
				new_paddle1_X = GLCD_WIDTH - (borderWidth + (2*paddleW));

				// Ball position coordinates
				new_ball_X = GLCD_WIDTH/2;
				
				//int ball_Y = random(1 + ball_rad, GLCD_HEIGHT - ball_rad);//paddle0_Y + ball_rad;
				new_ball_Y = new_paddle0_Y + ball_rad;
				
				//srand(rand());
				//int r = rand();
				
				// We don't want to have the ball running towards the left whenever a new game start !
				ballVelocityX = abs(ballVelocityX);
				
				while ((new_ball_X > 1) && (new_ball_X + ball_rad < (GLCD_WIDTH - (2*borderWidth))))
				{
					if (btn ^ btnMsk)
						{
							 btn = btnMsk;
							 if (btnMsk & BTN_INT0) 
							 {
								 isPaused = ~isPaused;
							 }
						}
			
					// Increment ball's position
					new_ball_X += ballVelocityX;
					new_ball_Y += ballVelocityY;
					// Check if the ball is colliding with the left paddle
					if (new_ball_X < (new_paddle0_X + paddleW) )
					{
						// Check if ball is within paddle's height
						if ((new_ball_Y > new_paddle0_Y) && (new_ball_Y < new_paddle0_Y + paddleH))
						{
							new_ball_X += ball_rad;  // Move ball over one to the right
							ballVelocityX = -ballVelocityX; // Change velocity
							Beep();
						}
					}
					// Check if the ball hit the right paddle
					if (new_ball_X + ball_rad > new_paddle1_X)
					{
						// Check if ball is within paddle's height
						if ((new_ball_Y > new_paddle1_Y) && (new_ball_Y < new_paddle1_Y + paddleH))
						{
							new_ball_X -= ball_rad;  // Move ball over one to the left
							ballVelocityX = -ballVelocityX; // change velocity
							Beep();
						}
					}
					// Check if the ball hit the top or bottom
					if ((new_ball_Y <= borderWidth) || (new_ball_Y >= (GLCD_HEIGHT - ball_rad - 1)))
					{
						// Change up/down velocity direction
						ballVelocityY = -ballVelocityY;
						Beep();
					}

#define AUTOMATIC_CONTROL 1
//define as 0 to control the left paddle manually using the potentiometer
//define as 1 to have the left paddle be controlled automatically

					if(AUTOMATIC_CONTROL)
					{
						//Automatic Control
						new_paddle0_Y = new_ball_Y-paddleH/2; // Invincible paddle0
					
					}
					else
					{
						// Controlled by potentiometer
						ADC_StartConversion();
						new_paddle0_Y = map(ADC_GetValue(),0,4096,0,GLCD_HEIGHT);
					}
					
					// Automatic control
					new_paddle1_Y = new_ball_Y-paddleH/2; // Invincible paddle1
		

					// Check if paddle0 is not out of the frame
					if(new_paddle0_Y  < borderWidth) { // Top Frame
						new_paddle0_Y = borderWidth;
					}
					if(new_paddle0_Y + paddleH > GLCD_HEIGHT - borderWidth) { // Bottom Frame
						new_paddle0_Y = GLCD_HEIGHT - borderWidth - paddleH;
					}
					
					// Check if paddle1 is not out of the frame
					if(new_paddle1_Y  < borderWidth) { // Top Frame
						new_paddle1_Y = borderWidth;
					}
					if(new_paddle1_Y + paddleH > GLCD_HEIGHT - borderWidth) { // Bottom Frame
						new_paddle1_Y = GLCD_HEIGHT - borderWidth - paddleH;
					}

					joyMsk = Joystick_GetState();   // Show joystick arrows               
						if (joy ^ joyMsk)                
							{
								joy = joyMsk;

								if(joy & JOYSTICK_LEFT)
								{
									if(ballVelocityX > 0)
									{
										ballVelocityX--;
									}
									else
									{
										ballVelocityX++;
									}
									
									if(ballVelocityY > 0)
									{
										ballVelocityY--;
									}
									else
									{
										ballVelocityY++;
									}
								}
								if(joy & JOYSTICK_RIGHT)
								{
									if(ballVelocityX > 0)
									{
										ballVelocityX++;
									}
									else
									{
										ballVelocityX--;
									}
									
									if(ballVelocityY > 0)
									{
										ballVelocityY++;
									}
									else
									{
										ballVelocityY--;
									}
								}
								if(joy & JOYSTICK_CENTER)
								{
									
								}
								if(joy & JOYSTICK_UP)
								{						
									GLCD_SetForegroundColor(BackgroundColor);
									GLCD_DrawRectangle(old_ball_X, old_ball_Y, ball_rad, ball_rad);
									GLCD_SetForegroundColor(BallForegroundColor);
									ball_rad++;
								}
								if(joy & JOYSTICK_DOWN)
								{
									GLCD_SetForegroundColor(BackgroundColor);
									GLCD_DrawRectangle(old_ball_X, old_ball_Y, ball_rad, ball_rad);
									GLCD_SetForegroundColor(BallForegroundColor);
									ball_rad--;
								}	
						}
						
					// Draw the Pong Field					
					
					//STATIC OBJECTS
					// Draw an outline of the screen:
					GLCD_SetForegroundColor(ForegroundColor);
					GLCD_DrawRectangle(0, 0, GLCD_WIDTH - 1, GLCD_HEIGHT - 1);
					// Draw the center line
					GLCD_DrawRectangle(GLCD_WIDTH/2 - 1, 0, borderWidth, GLCD_HEIGHT);	
					
					//MOVING OBJECTS		
					GLCD_SetForegroundColor(BackgroundColor);
					// Erase the Paddles:
					GLCD_DrawRectangle(old_paddle0_X, old_paddle0_Y, paddleW, paddleH);
					GLCD_DrawRectangle(old_paddle1_X, old_paddle1_Y, paddleW, paddleH);
					// Erase the ball:
					GLCD_DrawRectangle(old_ball_X, old_ball_Y, ball_rad, ball_rad);
					
					old_paddle0_X = new_paddle0_X;
					old_paddle0_Y = new_paddle0_Y;
					old_paddle1_X = new_paddle1_X;
					old_paddle1_Y = new_paddle1_Y;
					old_ball_X = new_ball_X;
					old_ball_Y = new_ball_Y;
					
					// Draw the Paddles:
					GLCD_SetForegroundColor(LeftPaddleForegroundColor);
					GLCD_DrawRectangle(new_paddle0_X, new_paddle0_Y, paddleW, paddleH);
					
					GLCD_SetForegroundColor(RightPaddleForegroundColor);					
					GLCD_DrawRectangle(new_paddle1_X, new_paddle1_Y, paddleW, paddleH);
					// Draw the ball:
					GLCD_SetForegroundColor(BallForegroundColor);	
					GLCD_DrawRectangle(new_ball_X, new_ball_Y, ball_rad, ball_rad);

					if((new_ball_X + ball_rad) >= (GLCD_WIDTH - (2*borderWidth)))
					{
						score0++;
					}		
					
					if(new_ball_X <= 1)
					{
						score1++;
					}
					
					sprintf (str_score0, "%2u", score0);
					sprintf (str_score1, "%2u", score1);
					str_score0[sizeof(str_score0)-1] = '\0'; //indicate end of string
					str_score1[sizeof(str_score1)-1] = '\0'; //indicate end of string
					GLCD_SetForegroundColor(LeftPaddleForegroundColor);
					GLCD_DrawString(0, 0, str_score0);
					GLCD_SetForegroundColor(RightPaddleForegroundColor);
					GLCD_DrawString(GLCD_WIDTH/2, 0,  str_score1);
				}			
  }
			
 }//end while(1)
}//end main



 */









