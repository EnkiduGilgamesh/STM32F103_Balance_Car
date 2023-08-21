#include "debugger.h"

/**
  * @IDE IAR for ARM
  */
/* USARRT DEBUGGER */
//int fputc(int ch, FILE *f){
//  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
//  return ch;
//}

/* SWD DEBUGGER */
int fputc(int ch, FILE *f){
  ITM_SendChar(ch);
  return ch;
}