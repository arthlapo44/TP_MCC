/**
  ******************************************************************************
  * @file           : console.h
  * @brief          Header to the console.c file
  ******************************************************************************
  */

#ifndef INC_CONSOLE_H_

/* Includes --------------------------------------------------------------*/
#define INC_CONSOLEs_H_
#include "main.h"

/* Variables ------------------------------------------------------------ */
#define UART_RX_BUFFER_SIZE 80 // size of the data reception buffer via the uart link
#define UART_TX_BUFFER_SIZE 80 // size of the data transmission buffer via the uart link

/* Function prototypes ---------------------------------------------------*/
void shell_init(void);
char uart_read();
void uart_write(uint8_t*, uint16_t);
void print_prompt(void);
char echo(void);
void shellTreatment(void);
void shell_exec(char *);
void command(int, char **);
void help(void);
void pinout_f(void);
void stop(void);
int isNumeric(char *);

void start(void);
void speed(int);

#endif
