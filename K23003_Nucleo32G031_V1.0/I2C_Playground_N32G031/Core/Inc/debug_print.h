/*
 * debug_print.h
 *
 *  Created on: Jan 4, 2024
 *      Author: micha
 */

#ifndef INC_DEBUG_PRINT_H_
#define INC_DEBUG_PRINT_H_


// Includes
#include <stdio.h>
#include <stdarg.h>
#include "main.h"



// Defines
#define TX_BUF_SIZE				100


void debug_print(char* format,...);


#endif /* INC_DEBUG_PRINT_H_ */
