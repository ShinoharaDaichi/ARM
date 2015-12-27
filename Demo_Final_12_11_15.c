
#include "LPC17xx.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include "Board_LED.h"
#include "Board_Buttons.h"
#include "Board_Joystick.h"
#include "Board_ADC.h"
#include "Board_GLCD.h"
#include "GLCD_Config.h"
#include "GLCD_Config.h"

#include "PIN_LPC17xx.h"
#include "GPIO_LPC17xx.h"
#include <stdbool.h>
#include <stdlib.h>

extern GLCD_FONT GLCD_Font_6x8;
extern GLCD_FONT GLCD_Font_16x24;

/* Joystick position definitions */
#define JOY_X        (9*16)
#define JOY_Y        (5*24 + 6)

/* Buttons bit masks */
#define BTN_INT0     (1 << 0)

/* GLCD string buffer */
#define STRINGBUF_LEN 21

#define numberOfApps 2 //must be modified if the number of apps is changed

char StringBuf[STRINGBUF_LEN];

int selection = 0;
int32_t 	joy  = -1;
uint32_t 	joyMsk;
uint32_t  btnMsk;

int32_t BackgroundColor;
int32_t ForegroundColor;

//+++++++++++++++++++++Variables for AppMenu+++++++++++++++++++ 
int offset_y = (1*24)+(3*8); //Leave enough space to write the Menu title
int select_y_start,select_y_end; //Current selection y position
typedef enum {InitMenu, Menu, InitPong, Pong, InitShooter, Shooter} MenuChoice;
MenuChoice menuChoice;
//++++++++++++++++++++++END Variables for AppMenu+++++++++++++++

//+++++++++++++++++++++Variables for AppPong+++++++++++++++++++
#define AUTOMATIC_CONTROL 0
//define as 0 to control the left paddle manually using the potentiometer
//define as 1 to have the left paddle be controlled automatically
#define SCORE_MAX_PONG 5
void Thread_Beep (void const *arg);                           // function prototype for Thread_Beep
osThreadDef (Thread_Beep, osPriorityLow, 1, 0);            // define Thread_Beep
osThreadId idThreadBeep;

//Variables of AppPong
int32_t     adcVal, adc, btn;
int32_t 		BallForegroundColor, LeftPaddleForegroundColor, RightPaddleForegroundColor;

typedef enum {Normal, Lose, GameOver} AppPongState;
AppPongState appPongState;

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

int ballVelocityX = 10;  		// Ball left/right velocity
int ballVelocityY = 5;  		// Ball up/down velocity
int paddle0Velocity = -1;  	// Paddle 0 velocity
int paddle1Velocity = 1;  	// Paddle 1 velocity

unsigned int ball_rad = 5;  // Ball radius
unsigned int paddleW = 3;  	// Paddle width
unsigned int paddleH = 60;  // Paddle height

unsigned int score0 = 0;
unsigned int score1 = 0;

char str_score0[10]; //string representation of score0
char str_score1[10]; //string representation of score1

unsigned int borderWidth = 1;
//++++++++++++++++++++END Variables for AppPong+++++++++++++++++

//++++++++++++++++++++Variables for AppShooter++++++++++++++++++
#define maxNumberBalls 10
#define maxNumberObjects 2
#define SCORE_MAX_SHOOTER 2

/* Static variables */
static uint32_t		timerTick_01;												// Timer counter for Main
static uint32_t		timerTick_02;												// Timer counter for drawing the Objects
static uint16_t 	plane_x;														// Plane X position
static uint16_t 	plane_y;														// Plane Y position
static uint16_t 	ballArray[maxNumberBalls][2];				// ballArray[actualBallNumberer-1][0:x,1:y]
static uint8_t 		actualBallNumber;														
static bool				newBall;														// Tells to the routine S_MoveBall to create a new ball
static uint16_t 	objectArray[maxNumberObjects][2];		// objectArray[objectNumber-1][0:x,1:y]
static uint8_t 		objectNumb;
static uint16_t		objectColor;
static int16_t		objSenseArray[maxNumberObjects][2];	// Sense (+/-) Speed (0-v)
static bool				stopThreads;
static uint8_t 		playerScore;
static uint8_t 		machineScore;
static uint8_t		randSeed = 0;
static int8_t			ballTimer;
static bool 			stopShooter;
static bool				S_firstTime = true;

osThreadId idThreadPlane;

/* Functions Definitions */
void S_DrawLineOverRide (uint16_t x, uint16_t y, uint16_t data);
void S_DrawPlane(uint16_t x, uint16_t y, bool del);
void NewBall (void);
void S_MoveBall(bool newBall);
void S_DrawStar(uint16_t x, uint16_t y);
void S_MoveObjects (void);
void S_DrawExplosion(uint16_t x, uint16_t y);
void S_IncPlayerScore(void);
void S_IncMachineScore(void);

/* Semaphore for using the display */
osSemaphoreDef(glcd_semaph);			// Semaphore definition
osSemaphoreId glcd_semaph_id;			// Semaphore ID

/* Mutex for using Plane Variables */
osMutexDef (planeVar_mutex);    	// Declare mutex
osMutexId  (planeVar_mutex_id); 	// Mutex ID

/* Mutex for using Objects Variables */
osMutexDef (objectVar_mutex);    	// Declare mutex
osMutexId  (objectVar_mutex_id); 	// Mutex ID

/*-----------------------------------------------------------------------------
  Periodic timer callback
 *----------------------------------------------------------------------------*/
/* Periodic timer definition */
void S_Timer_Callback (void const *arg);
osTimerDef (PeriodicTimer, S_Timer_Callback);
osTimerId TimerId;

void S_Timer_Callback (void const *arg) {
	timerTick_01++;
	timerTick_02++;
}
//+++++++++++++++++++++ENDVariables for AppShooter+++++++++++++++++++



//*******************************PROTOTYPES_START*************************************
void DrawPongPreview(void);
void DrawShooterPreview(void);
//*******************************PROTOTYPES_END***************************************

static double map(double input, double input_start, double input_end, double output_start, double output_end)
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


uint32_t Get_ADC_value(void){
	/* Wait for conversion to complete */
	while(!(LPC_ADC->ADSTAT & (1<<2)));
	/* Return the conversion value */
	return ((LPC_ADC->ADDR2 >> 4) & (0xFFF));
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

		// Select MAT0.0 instead of UART0 Tx (p.609)
    LPC_SC->DMAREQSEL = 0x01;
		
		// Enable the GPDMA controller
    LPC_GPDMA->DMACConfig = 1;

		// Enable synchro logic request 8, MAT0.0
    LPC_GPDMA->DMACSync   = 1 << 8;  

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
     
		 idThreadBeep = osThreadCreate (osThread (Thread_Beep), NULL);         // create the thread
		 T0_Init();	
}

void Thread_Beep(void const *arg)
{
	while(1)
	{
		osSignalWait (0x0001, osWaitForever); // wait forever for the signal 0x0001 which is the beep signal

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
	}
		
}

void Beep()
{
	osSignalSet (idThreadBeep, 0x0001);    // set the beep signal 0x0001 for thread tid_thread1
	LPC_GPIO0->FIOPIN ^= 1 << 16;                  // Toggle P0.16
	osDelay(1); // wait a little for the Thread_Beep to be able to process the signal since the thread priority is set to low to have a higher framerate
}


void InitializeAppMenu()
{
	selection = 0;
	BackgroundColor = GLCD_COLOR_WHITE;
	ForegroundColor = GLCD_COLOR_BLACK;
	GLCD_SetBackgroundColor (BackgroundColor);
  GLCD_SetForegroundColor (ForegroundColor);
	GLCD_ClearScreen();
	
}

void AppMenu()
{
	offset_y = (1*24)+(3*8);
	BackgroundColor = GLCD_COLOR_WHITE;
	ForegroundColor = GLCD_COLOR_BLACK;
	
	joyMsk = Joystick_GetState();         
	if (joy ^ joyMsk)                		
	{
		joy = joyMsk;
	}
	
	if(joy & JOYSTICK_RIGHT)
	{
		switch(selection)
		{
			case 0:
				menuChoice = InitPong;
				break;
			case 1:
				menuChoice = InitShooter;
				break;
		}
	}

	if(joy & JOYSTICK_UP)
	{				
		if(selection > 0)
		{
			selection--;
		}		
	}
	if(joy & JOYSTICK_DOWN)
	{
		if(selection < (numberOfApps-1))
		{
			selection++;
		}		
	}

	GLCD_SetBackgroundColor (BackgroundColor);
  GLCD_SetForegroundColor (ForegroundColor);
  
	GLCD_DrawRectangle(24, 3, 180, offset_y);
					
	GLCD_SetFont            (&GLCD_Font_16x24);
	GLCD_DrawString         (32, (0*24)+(1*8), "Menu");
	GLCD_SetFont            (&GLCD_Font_6x8);
  GLCD_DrawString         (32, (1*24)+(1*8), "UP/DOWN to select a game");
  GLCD_DrawString         (32, (1*24)+(2*8), "RIGHT to confirm your choice");
	
	GLCD_DrawRectangle(210,3,105,233); //Draw border for preview
	//-------------------------------------------
	GLCD_SetFont            (&GLCD_Font_16x24);
	GLCD_SetForegroundColor (ForegroundColor);
	GLCD_DrawString         (32, (((float) (GLCD_HEIGHT - offset_y)/(float)(numberOfApps+1))*1) + offset_y - 12, "PONG");
		
	if(selection == 0)
	{
		GLCD_SetForegroundColor (ForegroundColor); //Draw if selected		
	}
	else
	{
		GLCD_SetForegroundColor (BackgroundColor); //Don't draw if not selected (will be drawn but in the same color as the background, hence invisible)
	}
	GLCD_DrawRectangle(24, (((float) (GLCD_HEIGHT - offset_y)/(float)(numberOfApps+1))*1) + offset_y - 24, 180, (((float) (GLCD_HEIGHT - offset_y)/(float)(numberOfApps+1))*1)-20);
	DrawPongPreview();	
	
	//-------------------------------------------
	
	//-------------------------------------------
	GLCD_SetForegroundColor (ForegroundColor);
	GLCD_DrawString         (32, (((float) (GLCD_HEIGHT - offset_y)/(float)(numberOfApps+1))*2) + offset_y - 12, "SHOOTER");
		
	if(selection == 1)
	{
		GLCD_SetForegroundColor (ForegroundColor);		
	}
	else
	{
		GLCD_SetForegroundColor (BackgroundColor);	
	}
	GLCD_DrawRectangle(24, (((float) (GLCD_HEIGHT - offset_y)/(float)(numberOfApps+1))*2) + offset_y - 24, 180, (((float) (GLCD_HEIGHT - offset_y)/(float)(numberOfApps+1))*1)-20);
	DrawShooterPreview();
	 
	 //-------------------------------------------

	
	select_y_start = (((float) (GLCD_HEIGHT - offset_y)/(float)(numberOfApps+1))*1) + offset_y - 12;
	select_y_end = select_y_start + 32;	

}

// ***************************** AppPong ********************************

void DrawPongPreview()
{
	GLCD_DrawRectangle(211,GLCD_HEIGHT/2,103,2); //Draw middle line
	GLCD_DrawRectangle(250,160,5,5); //Draw ball	

	GLCD_DrawRectangle(215,8,50,4); //Draw Up paddle
	GLCD_DrawString(215, (GLCD_HEIGHT/2)-25,  "3"); //Write Upper player's score	

	GLCD_DrawRectangle(260,227,50,4); //Draw bottom paddle
	GLCD_DrawString(215, (GLCD_HEIGHT/2)+10,  "5"); //Write bottmom player's score
}

void InitializeAppPong()
{			
    joy  = -1;
    adc  = -1;
    btn  = -1;
	
		BackgroundColor = GLCD_COLOR_BLACK;
	  ForegroundColor = GLCD_COLOR_WHITE;
		BallForegroundColor = GLCD_COLOR_RED;
	  LeftPaddleForegroundColor = GLCD_COLOR_BLUE;
		RightPaddleForegroundColor = GLCD_COLOR_GREEN;
	
    GLCD_SetBackgroundColor (BackgroundColor);
    GLCD_SetForegroundColor (ForegroundColor);
    GLCD_SetFont            (&GLCD_Font_16x24);
		GLCD_ClearScreen();
	
		// Paddle 0 (left) position coordinates
		new_paddle0_Y = (GLCD_HEIGHT / 2) - (paddleH / 2);
		new_paddle0_X = borderWidth + paddleW;
		// Paddle 1 (right) position coordinates
		new_paddle1_Y = (GLCD_HEIGHT / 2) - (paddleH / 2);
		new_paddle1_X = GLCD_WIDTH - (borderWidth + (2*paddleW));

		// Ball position coordinates
		new_ball_X = GLCD_WIDTH/2;
				
		new_ball_Y = new_paddle0_Y + ball_rad;
				
				
		// We don't want to have the ball running towards the left whenever a new game start !
		ballVelocityX = abs(ballVelocityX);
		
		appPongState = Normal;		
}

void AppPong()
{					
	switch(appPongState)
	{
		case Normal:			

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

			if(AUTOMATIC_CONTROL)
			{
				//Automatic Control
				new_paddle0_Y = new_ball_Y-paddleH/2; // Invincible paddle0					
			}
			else
			{
				//Controlled by potentiometer
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
				
			if(new_ball_X > GLCD_WIDTH)
			{
				score0++;
				appPongState = Lose;
			}		
			
			if(new_ball_X < 0)
			{
				score1++;
				appPongState = Lose;
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
			
			sprintf (str_score0, "%1u", score0); //takes the numeric value and convert it to a char with the %1u format (1 unit unsigned)
			sprintf (str_score1, "%1u", score1);
			str_score0[sizeof(str_score0)-1] = '\0'; //indicate end of string
			str_score1[sizeof(str_score1)-1] = '\0';
			GLCD_SetForegroundColor(LeftPaddleForegroundColor);
			GLCD_DrawString(5, 5, str_score0);
			GLCD_SetForegroundColor(RightPaddleForegroundColor);
			GLCD_DrawString((GLCD_WIDTH/2)+5, 5,  str_score1);
		
			break;
			
		case Lose:
			if(score0 >= SCORE_MAX_PONG || score1 >= SCORE_MAX_PONG)
			{
				appPongState = GameOver;
				break;
			}

			InitializeAppPong();
			break;
			
		case GameOver:
			GLCD_SetFont            (&GLCD_Font_16x24);
			if(score0 > score1)
			{
				GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
				GLCD_DrawString((((GLCD_WIDTH/2)-(16*6))/2), GLCD_HEIGHT/2, "WIN !");
				GLCD_SetForegroundColor(GLCD_COLOR_GREEN);
				GLCD_DrawString((((GLCD_WIDTH/2)-(16*6))/2) + (GLCD_WIDTH/2), GLCD_HEIGHT/2, "LOSE !");
			}
			else
			{
				GLCD_SetForegroundColor(GLCD_COLOR_GREEN);
				GLCD_DrawString((((GLCD_WIDTH/2)-(16*6))/2) + (GLCD_WIDTH/2), GLCD_HEIGHT/2, "WIN !");
				GLCD_SetForegroundColor(GLCD_COLOR_BLUE);
				GLCD_DrawString((((GLCD_WIDTH/2)-(16*6))/2), GLCD_HEIGHT/2,  "LOSE !");
			}			
			
			score0 = 0;
			score1 = 0;
			osDelay(2000);
			menuChoice = InitMenu;

			osThreadTerminate (idThreadBeep);
			break;
	}	
			
}

// ***************************** AppShooter ********************************

/*-----------------------------------------------------------------------------
  Routine to Move the Balls
 *----------------------------------------------------------------------------*/
void S_MoveBall (bool newBall){
	uint8_t i;
	uint8_t j;
	
	if (newBall & (actualBallNumber < maxNumberBalls)){
		ballArray[actualBallNumber][0] = plane_x+9;
		ballArray[actualBallNumber][1] = plane_y+5;
		actualBallNumber++;
	}

	for (i = 0; i < actualBallNumber; i++){
		osSemaphoreWait (glcd_semaph_id, osWaitForever);
			GLCD_SetForegroundColor (GLCD_COLOR_BLACK);
			GLCD_DrawPixel	(ballArray[i][0], ballArray[i][1]);
		osSemaphoreRelease(glcd_semaph_id);
		ballArray[i][0]+=2;
		// If the new position exceed the limits, it will be erased from the array and it wont be drawn
		if(ballArray[i][0] > GLCD_WIDTH ){
			actualBallNumber--;
			ballArray[i][0] = ballArray[actualBallNumber][0];
			ballArray[i][1] = ballArray[actualBallNumber][1];
		} else {
			osSemaphoreWait (glcd_semaph_id, osWaitForever);
				GLCD_SetForegroundColor (GLCD_COLOR_RED);
				GLCD_DrawPixel	(ballArray[i][0]+2, ballArray[i][1]);
			osSemaphoreRelease(glcd_semaph_id);
		}
		// Object is deleted if hit by a ball
		osMutexWait(objectVar_mutex_id, osWaitForever);
		for (j = 0; j < objectNumb; j++){
			if ( (ballArray[i][0] > objectArray[j][0]) & (ballArray[i][0] < objectArray[j][0]+9) & (ballArray[i][1] > objectArray[j][1]) & (ballArray[i][1] < objectArray[j][1]+9) ){
			osSemaphoreWait (glcd_semaph_id, osWaitForever);
				GLCD_SetForegroundColor (GLCD_COLOR_BLACK);
				S_DrawStar (objectArray[j][0], objectArray[j][1]);
				S_IncPlayerScore();
			osSemaphoreRelease(glcd_semaph_id);
				objectNumb--;
				objectArray[j][0] = objectArray[objectNumb][0];
				objectArray[j][1] = objectArray[objectNumb][1];
			}
		}
		osMutexRelease(objectVar_mutex_id);
	}

}
/*-----------------------------------------------------------------------------
  Thread to Move the Plane
 *----------------------------------------------------------------------------*/
void S_Thread_MovePlane (void const *arg);
osThreadDef (S_Thread_MovePlane, osPriorityRealtime, 1, 0);

void S_Thread_MovePlane (void const *arg){  
	uint16_t 	plane_x_last = 1;
	uint16_t 	plane_y_last = GLCD_HEIGHT/2;
	uint16_t 	plane_x_i;
	uint16_t 	plane_y_i;
	uint8_t		i;
	
	while(!stopThreads){
		osMutexWait(planeVar_mutex_id, osWaitForever);
		plane_x_i = plane_x;
		plane_y_i = plane_y;
		osMutexRelease(planeVar_mutex_id);
		
		if( (plane_x_last != plane_x_i) || (plane_y_last != plane_y_i) ){
		
			osSemaphoreWait (glcd_semaph_id, osWaitForever);
				S_DrawPlane(plane_x_last, plane_y_last, true);	// Delete last position plane
				S_DrawPlane(plane_x_i, plane_y_i, false);				// Draw new position plane
			osSemaphoreRelease(glcd_semaph_id);
			
			plane_x_last = plane_x_i;
			plane_y_last = plane_y_i;
		}
		
		osMutexWait(objectVar_mutex_id, osWaitForever);
		for (i = 0; i < objectNumb; i++){
			// If the square around the plane intersects with the square around the object: the plane is destroyed
			if ( ( (objectArray[i][0]>(plane_x_i-9)) & (objectArray[i][0]<(plane_x_i+9)) ) & 
				( (objectArray[i][1]>(plane_y_i-9)) & (objectArray[i][1]<(plane_y_i+9)) ) ){
				osSemaphoreWait (glcd_semaph_id, osWaitForever);
					GLCD_SetForegroundColor (GLCD_COLOR_YELLOW);
					S_DrawExplosion(plane_x_i, plane_y_i);
					S_IncMachineScore();
				osSemaphoreRelease(glcd_semaph_id);
			}
		}
		osMutexRelease(objectVar_mutex_id);
		
		osDelay(50);
	}
}
/*-----------------------------------------------------------------------------
  Draw Plane
 *----------------------------------------------------------------------------*/
void S_DrawPlane(uint16_t x, uint16_t y, bool del){
	
	if (del){
		GLCD_SetForegroundColor (GLCD_COLOR_BLACK);
	} else {
		GLCD_SetForegroundColor (GLCD_COLOR_BLUE);
	}
	S_DrawLineOverRide (x, y, 	0x0F);
	S_DrawLineOverRide (x, y+1, 0x07);
	S_DrawLineOverRide (x, y+2, 0x0F);
	S_DrawLineOverRide (x, y+3, 0x3E);
	S_DrawLineOverRide (x, y+4, 0xFE);
	S_DrawLineOverRide (x, y+5, 0x3E);
	S_DrawLineOverRide (x, y+6, 0x0F);
	S_DrawLineOverRide (x, y+7, 0x07);
	S_DrawLineOverRide (x, y+8, 0x0F);

	if (del){
		GLCD_SetForegroundColor (GLCD_COLOR_BLACK);
	} else {
		GLCD_SetForegroundColor (GLCD_COLOR_YELLOW);
	}
	GLCD_DrawPixel (x, y+3);
	GLCD_DrawPixel (x, y+4);
	GLCD_DrawPixel (x, y+5);
}
/*-----------------------------------------------------------------------------
  Routine to Move the Objects
 *----------------------------------------------------------------------------*/
void S_MoveObjects (void){
	int8_t i = 0;
	uint32_t 	randNum;
	
	osMutexWait(objectVar_mutex_id, osWaitForever);
	
		srand(randSeed);
		randSeed++;
		randNum = rand();
	
	if (objectNumb < maxNumberObjects){				

		if (randNum % 17 == 0) {
				objectArray[objectNumb][0] = 100;
				objectArray[objectNumb][1] = 80;
		} else if (randNum % 11 == 0) {
				objectArray[objectNumb][0] = 40;
				objectArray[objectNumb][1] = 90;
		} else if (randNum % 7 == 0) {
				objectArray[objectNumb][0] = 200;
				objectArray[objectNumb][1] = 220;
		} else if (randNum % 5 == 0) {
				objectArray[objectNumb][0] = 60;
				objectArray[objectNumb][1] = 180;
		} else if (randNum % 3 == 0) {
				objectArray[objectNumb][0] = 150;
				objectArray[objectNumb][1] = 120;
		} else if (randNum % 2 == 0) {
				objectArray[objectNumb][0] = 80;
				objectArray[objectNumb][1] = 130;
		} else {
				objectArray[objectNumb][0] = 90;
				objectArray[objectNumb][1] = 80;
		}

		objSenseArray[objectNumb][0] = -10;
		objSenseArray[objectNumb][1] = 10;
		
		objectNumb++;
	}

	osMutexRelease(objectVar_mutex_id);
	

	for (i = 0; i < objectNumb; i++){
		// Delete the object at its last position
		osSemaphoreWait (glcd_semaph_id, osWaitForever);
			GLCD_SetForegroundColor (GLCD_COLOR_BLACK);
			S_DrawStar (objectArray[i][0], objectArray[i][1]);
		osSemaphoreRelease(glcd_semaph_id);

		osMutexWait(objectVar_mutex_id, osWaitForever);
		
		if(!i)
		{
			if (randNum % 5 == 0)
			{
				objSenseArray[i][0] = 20;
				objSenseArray[i][1] = 20;
			}
			else
			{
				objSenseArray[i][0] = 3;
				objSenseArray[i][1] = 3;
			}
				
			//Homing behaviour for ball 0
			if( objectArray[i][0] < plane_x ){
				objSenseArray[i][0] = abs(objSenseArray[i][0]);
			}
			else
			{
				objSenseArray[i][0] = -abs(objSenseArray[i][0]);
			}
		
			if( objectArray[i][1] < plane_y ){
				objSenseArray[i][1] = abs(objSenseArray[i][1]);
			}
			else
			{
				objSenseArray[i][1] = -abs(objSenseArray[i][1]);
			}
		}		
			
		// Reverse X sense if the object reach the limits
			if( objectArray[i][0] < 1 | objectArray[i][0] > GLCD_WIDTH - 11){
				objSenseArray[i][0] = -objSenseArray[i][0];
			}
		// Reverse Y sense if the object reach the limits
			if( objectArray[i][1] < 29 | objectArray[i][1] > GLCD_HEIGHT - 11){
				objSenseArray[i][1] = -objSenseArray[i][1];
			}
		// Move position of object
			objectArray[i][0]+=objSenseArray[i][0];
			objectArray[i][1]+=objSenseArray[i][1];

		// Draw the object at its new position
		osSemaphoreWait (glcd_semaph_id, osWaitForever);
			
			switch(i)
			{
				case 0:
					if(abs(objSenseArray[i][0]) == 20)						
					{
						objectColor = GLCD_COLOR_RED;
					}
					else
					{
						objectColor = GLCD_COLOR_BLUE;
					}
					break;
				case 1:
					objectColor = GLCD_COLOR_YELLOW;
					break;
				default:
					objectColor = GLCD_COLOR_WHITE;
				break;
			}	
			
			GLCD_SetForegroundColor (objectColor);
			S_DrawStar (objectArray[i][0],objectArray[i][1]);
		osSemaphoreRelease(glcd_semaph_id); 
		
			osMutexRelease(objectVar_mutex_id);
	}
}

/*-----------------------------------------------------------------------------
  Draw Objects
 *----------------------------------------------------------------------------*/
void S_DrawStar(uint16_t x, uint16_t y)
{
			S_DrawLineOverRide (x, y, 	0x38);
			S_DrawLineOverRide (x, y+1, 0x7C);
			S_DrawLineOverRide (x, y+2, 0xFE);
			S_DrawLineOverRide (x, y+3, 0x1FF);
			S_DrawLineOverRide (x, y+4, 0x1FF);
			S_DrawLineOverRide (x, y+5, 0x1FF);
			S_DrawLineOverRide (x, y+6, 0xFE);
			S_DrawLineOverRide (x, y+7, 0x7C);
			S_DrawLineOverRide (x, y+8, 0x38);
}
/*-----------------------------------------------------------------------------
  Draw Objects
 *----------------------------------------------------------------------------*/
void S_DrawExplosion(uint16_t x, uint16_t y)
{
			S_DrawLineOverRide (x, y, 	0x10);
			S_DrawLineOverRide (x, y+1, 0x3A);
			S_DrawLineOverRide (x, y+2, 0xCC);
			S_DrawLineOverRide (x, y+3, 0x30);
			S_DrawLineOverRide (x, y+4, 0xB4);
			S_DrawLineOverRide (x, y+5, 0x08);
			S_DrawLineOverRide (x, y+6, 0x12A);
			S_DrawLineOverRide (x, y+7, 0x40);
			S_DrawLineOverRide (x, y+8, 0x88);
}
/*-----------------------------------------------------------------------------
  Draw Horizontal "Line" of 9 pixels
 *----------------------------------------------------------------------------*/
void S_DrawLineOverRide (uint16_t x, uint16_t y, uint16_t data){
	uint16_t i = 1;
	while (i <= 0x100)
	{
		if ((data & i) != 0){
			GLCD_DrawPixel (x, y);
		}
		x++;
		i = (i<<1);
	}
}
/*-----------------------------------------------------------------------------
  Increase the Score of the Playerf and refresh score in GLCD
 *----------------------------------------------------------------------------*/

void S_IncPlayerScore(void){
	char 		scoreU = '0';
	char 		scoreD = '0';
	uint8_t	scoreTmp;
	
	scoreTmp = playerScore;
	playerScore++;

	GLCD_SetForegroundColor (GLCD_COLOR_GREEN);
	GLCD_SetFont 			(&GLCD_Font_16x24);
	
	while (scoreTmp > 9){
		scoreTmp -= 10;
		scoreD += 1;
	}
	scoreU += scoreTmp;
	
	GLCD_DrawChar     (5, 0*24, scoreD);
	GLCD_DrawChar     (20, 0*24, scoreU);
	
	if (playerScore > SCORE_MAX_SHOOTER) { stopShooter = true; }
}
/*-----------------------------------------------------------------------------
  Increase the Score of the Machine and refresh score in GLCD
 *----------------------------------------------------------------------------*/
void S_IncMachineScore(void){
	char 		scoreU = '0';
	char 		scoreD = '0';
	uint8_t	scoreTmp;
	
	scoreTmp = machineScore;
	machineScore++;

	GLCD_SetForegroundColor (GLCD_COLOR_RED);
	GLCD_SetFont 			(&GLCD_Font_16x24);
	
	while (scoreTmp > 9){
		scoreTmp -= 10;
		scoreD += 1;
	}
	scoreU += scoreTmp;
	
	GLCD_DrawChar     (165, 0*24, scoreD);
	GLCD_DrawChar     (180, 0*24, scoreU);
	
	if (machineScore > SCORE_MAX_SHOOTER) { stopShooter = true; }
}

void DrawShooterPreview(void)
{
	S_DrawStar(230, 80);
	S_DrawStar(305, 150);
	S_DrawStar(290, 30);

	GLCD_DrawPixel (243, 130);
	GLCD_DrawPixel (243, 131);

	GLCD_DrawPixel (253, 140);
	GLCD_DrawPixel (253, 141);

	GLCD_DrawPixel (263, 150);
	GLCD_DrawPixel (263, 151);

	GLCD_DrawPixel (273, 170);
	GLCD_DrawPixel (273, 171);
	
	GLCD_DrawPixel (273, 180);
	GLCD_DrawPixel (273, 181);

	S_DrawLineOverRide (270, 199, 0x10);
	S_DrawLineOverRide (270, 200, 0x10);
	S_DrawLineOverRide (270, 201, 0x38);
	S_DrawLineOverRide (270, 202, 0x38);
	S_DrawLineOverRide (270, 203, 0x17D);
	S_DrawLineOverRide (270, 204, 0x1FF);
	S_DrawLineOverRide (270, 205, 0x1FF);
	S_DrawLineOverRide (270, 206, 0x1C7);

	S_DrawExplosion (268, 160);

}
void InitializeAppShooter()
{
  TimerId = osTimerCreate (osTimer(PeriodicTimer), osTimerPeriodic, NULL);
  if (TimerId) {
    osTimerStart (TimerId, 8);
  }
	
  /* Prepare display */
	GLCD_SetBackgroundColor (GLCD_COLOR_BLACK);
  GLCD_SetForegroundColor (GLCD_COLOR_WHITE);
  GLCD_ClearScreen();
	
	// Variables initialization
	timerTick_01 = 0;										// Timer counter for Main
	timerTick_02 = 0;										// Timer counter for drawing the Objects
	plane_x = 0;												// Plane X position
	plane_y = 110;											// Plane Y position
	actualBallNumber = 0;
	newBall = false;										// Tells to the routine S_MoveBall to create a new ball
	objectNumb = 0;
	objectColor = 1;
	stopThreads = false;
	ballTimer = 0;
	stopShooter = false;
	playerScore = 0;
	machineScore = 0;
	
	S_IncPlayerScore();
	S_IncMachineScore();
	
	if(S_firstTime){
		glcd_semaph_id = osSemaphoreCreate(osSemaphore(glcd_semaph), 1); // Semaphore binaire, only one thread at the same time
		planeVar_mutex_id = osMutexCreate(osMutex(planeVar_mutex));
		objectVar_mutex_id = osMutexCreate(osMutex(objectVar_mutex));
		S_firstTime = false;
	}
	
	idThreadPlane = osThreadCreate (osThread (S_Thread_MovePlane), NULL);   // create the thread

}

void AppShooter()
{
	  while (!stopShooter) {

		joyMsk = Joystick_GetState();
		btnMsk = Buttons_GetState();

		if (timerTick_01 > 2)
		{
			if (ballTimer != 0) { ballTimer--; }
			if ( ((btnMsk & 1) != 0) && (ballTimer == 0) )
			{
				ballTimer = 8;
				newBall = true;
			}
			
			if (joyMsk != 0)
			{
				osMutexWait(planeVar_mutex_id, osWaitForever);
				// RIGHT
				if ((plane_x < GLCD_WIDTH - 10) && ((joyMsk & 2) != 0))
				{
					plane_x += 10;
				}
				// LEFT
				if ((plane_x > 0) && ((joyMsk & 1) != 0))
				{
					plane_x -= 10;
				}
				// UP
				if ((plane_y > 20) && ((joyMsk & 8) != 0))
				{
					plane_y -= 10;
				}
				// DOWN
				if ((plane_y < GLCD_HEIGHT - 10) && ((joyMsk & 16) != 0))
				{
					plane_y += 10;
				}
				osMutexRelease(planeVar_mutex_id);
			}
			timerTick_01 = 0;
			S_MoveBall(newBall);
			newBall = false;
		}	
		
		if ( timerTick_02 > 12 )
		{
			S_MoveObjects ();
			timerTick_02 = 0;
			
		}
  }

	osThreadTerminate (idThreadPlane);
	osDelay(500);
	
	if (playerScore > machineScore) {
		GLCD_SetForegroundColor (GLCD_COLOR_GREEN);
		GLCD_DrawString ((GLCD_WIDTH-16*9)/2, GLCD_HEIGHT/2-16, "YOU WIN !");
	} else {
		GLCD_SetForegroundColor (GLCD_COLOR_RED);
		GLCD_DrawString ((GLCD_WIDTH-16*11)/2, GLCD_HEIGHT/2-16, "YOU LOSE !");
	}
	osDelay(2000);
	
}

// ************************** MAIN *****************************

int main (void){
			
		ADC_Initialize          ();           // Initialize A/D Converter      
    GLCD_Initialize         ();           // Initialize Graphical LCD           
    Joystick_Initialize     ();           // Initialize joystick                

    GLCD_SetBackgroundColor (BackgroundColor);
    GLCD_SetForegroundColor (ForegroundColor);
    GLCD_SetFont            (&GLCD_Font_16x24);
		GLCD_ClearScreen();		
		
		// Select P0.16 as output 
		LPC_GPIO0->FIODIR |= 0x00010000;
			
		menuChoice = InitMenu;
		while (1)
			{
					switch(menuChoice)
					{
						case InitMenu:
							InitializeAppMenu();
							menuChoice = Menu;
							break;
						case Menu:
							AppMenu();
							break;
						case InitPong:
							initDMA();
							InitializeAppPong();
							menuChoice = Pong;
							break;
						case Pong:
							AppPong();
							break;
						case InitShooter:
							InitializeAppShooter();
							menuChoice = Shooter;
							break;
						case Shooter:
							AppShooter();
							menuChoice = InitMenu;
							break;
					}	
			

			}			
}



