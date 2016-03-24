#include "stm32f10x.h"
#include "usart1.h"
#include <stdio.h>
#include <stdbool.h>


#define TEXT_SIZE 255





//void TIM2_Configuration(void);
volatile int32_t debug;
void TIM4_IRQHandler(void);
/* User defined function prototypes */
void GPIOA_Init(void);

void led_toggle(void);

int8_t ATcmd(char* cmd, char* rep, unsigned int timeout);

bool find(char *a);


static char text[TEXT_SIZE];
//char buf[RBUF_SIZE];
//int dem=0;
int i;//using clear buf[]
//bool flag_timeout = 1;
long tick ;

void setup(void);
//================

int main(void)
{
		setup();
	
	/*
		tick = millis();
	
		put1("POWER_ON\n");
		sim800c_pwrOn();
	*/
 
    while(1)
    {
			
			/*
			put1(buffer());
			
			// some test at command
			i = sim800c_cmd("AT+hehehehe","OK",2000);

			if(i==1)
			{
				put1("\nhave reply");
			}
			else put1("\nTimeout, next session");
			// end_some test at command

			put1(buffer());// Print data reiceived
			if (find("OK"))
			{
				put1("\n-->OK");
				empty_buff();
			}
			*/
			/*
			if(flag_timeout)
			{
				for(i=0;i<RBUF_SIZE;i++)
				{	buf[i]='\0'; dem=0;	}
			}
			
			if(millis() - tick > 200)
			{
				led_toggle();
				tick = millis();
			}
			*/
			//put2("TICK\n");
			put1("TICK\n");
			USART2->DR = 45;
			delay(500);
    }
}   
 
/***********************************************
 * Initialize GPIOA PIN8 as push-pull output
 ***********************************************/
void GPIOB_Init(void)
{
    /* Bit configuration structure for GPIOA PIN8 */
    GPIO_InitTypeDef gpioa_init_struct = { GPIO_Pin_6, GPIO_Speed_50MHz, 
                                           GPIO_Mode_Out_PP };
                                             
    /* Enable PORT A clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Initialize GPIOA: 50MHz, PIN8, Push-pull Output */
    GPIO_Init(GPIOB, &gpioa_init_struct);   
     
    /* Turn off LED to start with */
    GPIO_SetBits(GPIOB, GPIO_Pin_6);
}

void led_toggle(void)
{
    /* Read LED output (GPIOA PIN8) status */
    uint8_t led_bit = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_6);
     
    /* If LED output set, clear it */
    if(led_bit == (uint8_t)Bit_SET)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_6);
    }
    /* If LED output clear, set it */
    else
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_6);
    }
}

/*

int8_t ATcmd(char* cmd, char* rep, unsigned int timeout)
{
	unsigned long last;
	int8_t have_rep=0;
	put(cmd);
	
	last = millis();
	
	do{
		if(find("OK")) have_rep=1;
		else have_rep=2;
	}
	while((millis()-last) < timeout);
	
	return have_rep;
}
*/


void setup()
{
	systick();// config for millis(), delay();
	
	/* Initialize GPIOA PIN8 */
	GPIOB_Init();
	
	/* Initialize USART1 */
	//usart1_init(115200);
	
	usart2_init(19200);
	usart1_init(19200);
	
	/* Init Timer*/
	TIMbase_Configuration();
}


