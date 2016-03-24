#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "usart1.h"
#include "stdio.h"

volatile uint32_t SysTickCnt=0;
static uint32_t time=0;
RCC_ClocksTypeDef   Clocks;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
void TIMbase_Configuration(void);

int dem=0;//using by buff[RBUF_SIZE]
int i_buff = 0; //using empty buff

int dem2=0;//using by buff[RBUF_SIZE]
int i_buff2 = 0; //using empty buff

char buff[RBUF_SIZE];
char buff2[RBUF_SIZE];
bool flag_timeout = 0;
bool flag_timeout2 = 0;

void usart1_init(int baud)
{
    /* USART configuration structure for USART1 */
    USART_InitTypeDef usart1_init_struct;
    /* Bit configuration structure for GPIOA PIN9 and PIN10 */
    GPIO_InitTypeDef gpioa_init_struct;
     
    /* Enalbe clock for USART1, AFIO and GPIOA */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO | 
                           RCC_APB2Periph_GPIOA, ENABLE);
                            
    /* GPIOA PIN9 alternative function Tx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioa_init_struct);
    /* GPIOA PIN10 alternative function Rx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioa_init_struct);
 
    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);  
    /* Baud rate 9600, 8-bit data, One stop bit
     * No parity, Do both Rx and Tx, No HW flow control
     */
    usart1_init_struct.USART_BaudRate = baud;   //9600
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;  
    usart1_init_struct.USART_StopBits = USART_StopBits_1;   
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* Configure USART1 */
    USART_Init(USART1, &usart1_init_struct);
    /* Enable RXNE interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /* Enable USART1 global interrupt */
    NVIC_EnableIRQ(USART1_IRQn);
}

void usart2_init(int baud)
{
    /* USART configuration structure for USART1 */
    USART_InitTypeDef usart2_init_struct;
    /* Bit configuration structure for GPIOA PIN9 and PIN10 */
    GPIO_InitTypeDef gpioa_init_struct;
     
    /* Enalbe clock for USART2, AFIO and GPIOD */
    RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2 | 
                           RCC_APB2Periph_GPIOD, ENABLE);
                            
    /* GPIOA PIN2 alternative function Tx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_2;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioa_init_struct);
    /* GPIOA PIN3 alternative function Rx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_3;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioa_init_struct);
 
    /* Enable USART2 */
    USART_Cmd(USART2, ENABLE);  
    /* Baud rate 9600, 8-bit data, One stop bit
     * No parity, Do both Rx and Tx, No HW flow control
     */
    usart2_init_struct.USART_BaudRate = baud;
    usart2_init_struct.USART_WordLength = USART_WordLength_8b;  
    usart2_init_struct.USART_StopBits = USART_StopBits_1;   
    usart2_init_struct.USART_Parity = USART_Parity_No ;
    usart2_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart2_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* Configure USART2 */
    USART_Init(USART2, &usart2_init_struct);
    /* Enable RXNE interrupt */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    /* Enable USART1 global interrupt */
    NVIC_EnableIRQ(USART2_IRQn);
}
 
void put1(char *c)
{
	while (*c)
	{
		while( !(USART1->SR & USART_FLAG_TXE) )
		{}
		USART1->DR = *c++;
	}
}

void put2(char *c)
{
	while (*c)
	{
		while( !(USART2->SR & USART_FLAG_TXE) )
		{}
		USART2->DR = *c++;
	}
}

void SysTick_Handler(void)  // IRQ!
{
	SysTickCnt++;
}

uint32_t millis(void)
{
	return (SysTickCnt);
}

void systick()
{
	RCC_GetClocksFreq(&Clocks);	
	SysTick_Config( Clocks.HCLK_Frequency/1000 - 1 );  // 1000 Hz ( T=1ms)
}

void delay(__IO uint32_t count)
{
	uint32_t SysTickCntHold;
	SysTickCntHold = SysTickCnt;
	while((SysTickCnt - SysTickCntHold ) <= count )
	{} // Nothing, really.;
}

char* buffer()
{
	return &buff[0];
}

void USART1_IRQHandler(void)
{
	volatile unsigned int IIR;
	IIR = USART1->SR;
		if(IIR & USART_FLAG_RXNE)
		{
			USART1->SR &= ~USART_FLAG_RXNE;              // clear interrupt
			TIM_Cmd(TIM4, ENABLE); // Enable time out RX
			flag_timeout=0;
			buff[dem++]=(char)USART_ReceiveData(USART1);
			if(dem>RBUF_SIZE) dem = 0;
		}
}

void USART2_IRQHandler(void)
{
	volatile unsigned int IIR;
	IIR = USART2->SR;
		if(IIR & USART_FLAG_RXNE)
		{
			USART2->SR &= ~USART_FLAG_RXNE;              // clear interrupt
			TIM_Cmd(TIM4, ENABLE); // Enable time out RX
			flag_timeout2=0;
			buff2[dem2++]=(char)USART_ReceiveData(USART2);
			if(dem>RBUF_SIZE) dem = 0;
		}
}

bool find(char *a)
{
	int n=0;
	int i,j;
	int kt=0;
	while(a[n])
	{n++;}
	for (i=0;i<dem;i++)
	{
		kt=1;
		for (j=0;j<n;j++)
		if (buff[i+j]!=a[j]) kt=0;
		if (kt==1) return 1;
	}
	return 0;
}

bool find2(char *a)
{
	int n=0;
	int i,j;
	int kt=0;
	while(a[n])
	{n++;}
	for (i=0;i<dem2;i++)
	{
		kt=1;
		for (j=0;j<n;j++)
		if (buff2[i+j]!=a[j]) kt=0;
		if (kt==1) return 1;
	}
	return 0;
}

void TIMbase_Configuration(void)
{
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
   
 /* Time base configuration */
 TIM_TimeBaseStructure.TIM_Prescaler = ((SystemCoreClock)/1000000)-1;     // frequency = 1000000
 TIM_TimeBaseStructure.TIM_Period = TIME_OUT_RX;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
 TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
 TIM_Cmd(TIM4, ENABLE);
 
 NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 NVIC_Init(&NVIC_InitStructure);   
}

//=========================
//TIM2_Configuration();
/*
void TIM2_Configuration(void)
{
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
   
 // Time base configuration 
 TIM_TimeBaseStructure.TIM_Prescaler = 0;
 TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 TIM_Cmd(TIM2, ENABLE);
}
*/
//=========================

void TIM4_IRQHandler(void)
{
 if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
 {
  if(++time>1)
  {
		time = 0;
		/* Disable timer*/
		TIM_Cmd(TIM4, DISABLE);
		flag_timeout=1;
  }
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
 }
}


int8_t sim800c_cmd(char* cmd, char* rep, unsigned int timeout)
{
	unsigned long last;
	int8_t have_rep=0;
	put1(cmd);
	
	last = millis();
	
	do{
		if(find("OK")) have_rep=1;
//		else have_rep=2;
	}
	while((millis()-last) < timeout);
	
	return have_rep;
}

void empty_buff(void)
{
	for(i_buff=0;i_buff<RBUF_SIZE;i_buff++)
			{	buff[i_buff]='\0'; dem=0;	}
}

bool sim800c_pwrOn(void)
{
	/* Bit configuration structure for GPIOA PIN8 */
	GPIO_InitTypeDef gpioa_init_struct =
	{ GPIO_Pin_8, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };
                                             
	/* Enable PORT A clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	/* Initialize GPIOB: 50MHz, PIN8, Push-pull Output */
	GPIO_Init(GPIOB, &gpioa_init_struct);   
     
	/* Turn off LED to start with */
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	delay(10);
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	delay(1000);
	GPIO_SetBits(GPIOB, GPIO_Pin_8);
	delay(3000);	
}

bool sim800c_init(void)
{
	if(sim800c_cmd("AT","OK",100))
	{
		sim800c_cmd("AT+IPR=115200","OK",100);
		sim800c_cmd("ATEO","OK",100);
		sim800c_cmd("AT+CFUN=1","OK",10000);
		return true;
	}
	return false;
}

