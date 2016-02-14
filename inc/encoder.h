#ifndef __ENCODER_H_
#define __ENCODER_H_

/*==================[inclusions]=============================================*/

#include "ciaaPOSIX_stdio.h"  /* <= device handler header */

/*==================[macros]=================================================*/

#define ENCODER_COUNT			(2)
#define SPEED_TYPE_RPM			(0)
#define SPEED_TYPE_INTERRUPTS	(1)

/*==================[typedef]================================================*/

typedef void (*callBackTimeElapsedFunction_type)(void);

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void encoder_init(void);
extern void encoder_beginCount(uint16_t periodMS);
extern void encoder_setTimeElapsedCallback(callBackTimeElapsedFunction_type fcnPtr);
extern uint16_t encoder_getLastCount(uint8_t encoderID);
extern void encoder_resetCount(void);

#endif /* __ENCODER_H_ */
