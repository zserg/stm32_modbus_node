#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "string.h"

#include "usart_log.h"

#define NUM_OF_MESSAGES 1

extern xMessage xUsartLogMessage;
 extern xQueueHandle xUsartLogQueue;
 
 const uint8_t hex[16]="0123456789ABCDEF";
 uint8_t hex_str[4];
 
 const uint8_t* msgArray[] = 
 {
   "Message 0: Info: ",// 0
   "Message 1: Warning: ",// 1
   "Message 2: Error: ",// 2
   "d=",// 3
   };
 
 void UsartLog(uint8_t MessageIndex, uint8_t data)
 {
 
   portBASE_TYPE xStatus;
   
   xUsartLogMessage.iMessage = msgArray[MessageIndex];
   xUsartLogMessage.iLen = strlen(msgArray[MessageIndex]);
 
   xStatus = xQueueSend(xUsartLogQueue,&xUsartLogMessage,portMAX_DELAY);
    hex_str[0] = hex[(data>>4)];
    hex_str[1] = hex[data%16];
    hex_str[2] = '\n';
    hex_str[3] = '\r';
 
   xUsartLogMessage.iMessage = hex_str;
   xUsartLogMessage.iLen = sizeof(hex_str);
 
   xStatus = xQueueSend(xUsartLogQueue,&xUsartLogMessage,portMAX_DELAY);
 }
 

