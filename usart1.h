#ifndef __USART1_H

#include "stm32f10x.h"
#include <stdbool.h>

#define p0 GPIO_Pin_0
#define p1 GPIO_Pin_1
#define p2 GPIO_Pin_2
#define p3 GPIO_Pin_3
#define p4 GPIO_Pin_4
#define p5 GPIO_Pin_5
#define p6 GPIO_Pin_6
#define p7 GPIO_Pin_7
#define p8 GPIO_Pin_8
#define p9 GPIO_Pin_9
#define p10 GPIO_Pin_10
#define p11 GPIO_Pin_11
#define p12 GPIO_Pin_12
#define p13 GPIO_Pin_13
#define p14 GPIO_Pin_14
#define p15 GPIO_Pin_15

#define A GPIOA
#define B GPIOB
#define C GPIOC
#define D GPIOD
//============================
#define port B
#define pin 8

#define RBUF_SIZE 128
#define TIME_OUT_RX 10

typedef struct {
  float lat;
  float lon;
  uint8_t year; /* year past 2000, e.g. 15 for 2015 */
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} GSM_LOCATION;

void put1(char *s);
void put2(char *s);
void usart1_init(int baud);
void usart2_init(int baud);

//int find(char *a);
//================
void SysTick_Handler(void);
void systick();
void delay(__IO uint32_t count);
uint32_t millis(void);
//================

//================
bool		sim800c_pwrOn(void);
bool		sim800c_init(void);
int			sim800c_setup(const char* apn);
bool		sim800c_getOperatorName(void);
bool		sim800c_checkSMS(void);
int			sim800c_getSignalQuality(void);
bool		sim800c_getLocation(GSM_LOCATION* loc);
void		sim800c_httpUninit(void);
bool		sim800c_httpInit(void);
bool		sim800c_httpConnect(const char* url, const char* args);
int			sim800c_httpIsConnected(void);
void		sim800c_httpRead(void);
int			sim800c_httpIsRead(void);
int8_t	sim800c_cmd(char* cmd, char* rep, unsigned int timeout);
int			sim800c_checkbuffer(const char* arg1, const char* arg2, unsigned int timeout);

char* buffer(void);
void empty_buff(void);

void TIMbase_Configuration(void);

//================

#endif
