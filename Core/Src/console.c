/**
  ******************************************************************************
  * @file           : console.c
  * @brief          Contains the different functions to instore a shell with the uart transmission
  ******************************************************************************
  */

/* Includes --------------------------------------------------------------*/
#include "console.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

/* Variables ------------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
const uint8_t starting[] = "\r\n *** Starting ***\r\n"; // message to display during startup
const uint8_t prompt[] = "<Laporte / Hanquiez>@Nucleo-G431 >>"; // prompt to display while waiting for a new instruction after each new command
int it_uart_rx_ready = 0; // variable to know if the processing of a character should take place after an interrupt
char uart_rx_buffer[UART_RX_BUFFER_SIZE]; // data reception buffer via the uart link
char uart_tx_buffer[UART_TX_BUFFER_SIZE]; // data transmission buffer via the uart link
char enter[3] = "\r\n"; // contains the value "return"
char backspace[3] = " \b"; // contains the value "backspace"
char cmd[18]; // contains the current command
int id = 0; // contains the index of the next character to be filled
const uint8_t help_s[] = "Fonctions :\n\r- pinout : affiche toutes les broches connectees et leur fonction\n\r- start : allume l'etage de puissance du moteur\n\r- stop : eteind l'etage de puissance du moteur"; // contains help message
const uint8_t pinout[] = "Liste des pin utilisés :\n\r- PA0 : Y_HALL\n\r- PA1 : R_HALL\n\r- PA2 : USART2_TX\n\r- PA3 : USART2_RX\n\r- PA7 : TIM1_CH1N\n\r- PA13 : SYS_JTMS-SWDIO\n\r- PA14 : SYS_JTCK-SWCLK\n\r- PB0 : TIM1_CH2N\n\r- PC0 : TIM1_CH1\n\r- PC1 : TIM2_CH2\n\r- PC5 : Start_Command"; // contains the list of pins used
const uint8_t power_on[] = "Power ON"; // contains engine ignition message
const uint8_t power_off[] = "Power OFF"; // contains the engine shutdown message
const uint8_t not_found[] = "Command not found"; // contains the message of the unrecognized command

/* Functions ------------------------------------------------------------ */

/**
 * @brief Displays a message during startup
 *
 */
void shell_init(void){
	HAL_UART_Transmit(&huart2, (uint8_t*)starting, sizeof(starting), HAL_MAX_DELAY);
	print_prompt();
	HAL_UART_Receive_IT(&huart2,(uint8_t*) uart_rx_buffer, 1);

}

/**
 * @brief Reading data via uart
 *
 * @return c : character given by the UART
 *
 */
char uart_read(){
	char c;
	HAL_UART_Receive_IT(&huart2, (uint8_t*)(&c), 1);
	return c;
}

/**
 * @brief Writing data via uart
 *
 * @param[in] s : uint8_t*, pointer to the text to transmit to the UART
 * @param[in] size : uint16_t, size of the text to transmit
 *
 */
void uart_write(uint8_t* s, uint16_t size){
	HAL_UART_Transmit(&huart2, (uint8_t*)s, size, HAL_MAX_DELAY);
}

/**
 * @brief Display this prompt while waiting for a new command after each new instruction
 *
 */
void print_prompt(void){
	HAL_UART_Transmit(&huart2, (uint8_t*) prompt, sizeof(prompt), HAL_MAX_DELAY);
}

/**
 * @brief Interrupt function that modifies it_uart_rx_ready, allowing the process of the character received by the HAL_UART_Receive_IT
 *
 *	@param[in] UartHandle : UART_HandleTypeDef*, pointer to the definition of the UART
 *
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	if (UartHandle == &huart2){
		it_uart_rx_ready = 1;
	}
}

/**
 * @brief Function that echoes the letters received by the UART to the transmit of the UART
 *
 * @return Returns the character received from the uart with a Receive_IT (not blocking the execution of other applications)
 * */
char echo(void){
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_rx_buffer, 1, HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart2,(uint8_t*) uart_rx_buffer, 1);
	return uart_rx_buffer[0];
}

/**
 * @brief Shell processing, stores the texte entered in the shell in a buffer.
 *
 * */
void shellTreatment(void){
	if (it_uart_rx_ready == 1){
		cmd[id] = echo();
		switch(cmd[id]){
			// Return
			case '\r':
				uart_write((uint8_t*) enter,2);
				cmd[id++] = 0;
				shell_exec(cmd);
				uart_write((uint8_t*) "\r\n",2);
				id = 0;
				uart_write((uint8_t*) prompt,sizeof(prompt));
				break;
			// Backspace
			case '\b':
				if (id > 0) {
					cmd[id]=0;
					id--;
					uart_write((uint8_t*) backspace,sizeof(backspace));
				}
				break;
			// Other characters
			default:
				//only store characters if buffer has space
				if (id < sizeof(cmd)) {
					id++;
				}
		}
		it_uart_rx_ready = 0;
	}
}

/**
 * @brief Function that separates the different arguments passed by the shellTreatment and put it in a buffer
 *
 * @param[in] buf : char*, buffer transmitted by shellTreatment containing the text entrerd in the UART transmission
 *
 * */
void shell_exec(char * buf){
    int argc; // integer that will count the element of argv
    char * argv[5]; // table containing the console character strings
    char *p;
	argc = 1;
	argv[0] = buf;
	for(p = buf ; *p != '\0' && argc < 5 ; p++){
		if(*p == ' ') {
			*p = '\0';
			argv[argc++] = p+1;
		}
	}
	command(argc,argv);
}

/**
 * @brief Compares the first argument passed by shell_exec with the denomination of the functions and lanches the corresponding one
 *
 * @param[in] argc : int, number of argument passed
 * @param[in] argv[] : char*, arguments passed
 *
 * */
void command(int argc, char *argv[]){
	if (strcmp(argv[0],"help")==0 && argc == 1){ // Displays all available commands
		help();
	}
	else if (strcmp(argv[0],"pinout")==0 && argc == 1){ // Displays all connected pins and their function
		pinout_f();
	}
	else if (strcmp(argv[0],"start")==0){ // Turns on the power stage of the motor
		start();
	}
	else if (strcmp(argv[0],"speed")==0 && strcmp(argv[1],"=")==0 && isNumeric(argv[2])){ // Controls the rotational speed of the motor
		speed(atoi(argv[2]));
	}
	else if (strcmp(argv[0],"stop")==0 && argc == 1){ // Switch off the engine power stage
		stop();
	}
	else { // send a message to the console : "Command not found"
		uart_write((uint8_t*) not_found,sizeof(not_found));
	}
}

/**
 * @brief Send help message
 *
 * */
void help(void){
	uart_write((uint8_t*) help_s,sizeof(help_s));
}

/**
 * @brief Send the list of pins used
 *
 * */
void pinout_f(void){
	uart_write((uint8_t*) pinout,sizeof(pinout));
}

/**
 * @brief Send the engine shutdown message
 *
 * */
void stop(void){
	uart_write((uint8_t*) power_off,sizeof(power_off));
}

/**
 * @brief Indicates whether an expression can be evaluated as a number or not
 *
 * @param[in] s : char*, string to verify
 * @return Returns 1 if it's a number, 0 if not.
 *
 * */
int isNumeric(char *s){
    int i = 0;
    while(s[i]!='\0'){// Compares element by element.
        if (isdigit((int) s[i])==0){
            return 0;
        }
        i++;
    }
    return 1;
}

/**
 * @brief Initialise the chopper
 *
 * @param[in] GPIO_Pin : uint16_t, GPIO pin that triggers the Callback event
 *
 * */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == USER_Button_Pin){ // Verification that it's the blue button that triggers the event
		uart_write((uint8_t*) "\n",1);
		start();
		shellTreatment();
	}
}
