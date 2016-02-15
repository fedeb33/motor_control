#ifndef __TIMER_H_
#define __TIMER_H_

/*==================[inclusions]=============================================*/

#include "ciaaPOSIX_stdlib.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void timer_init(void);
extern void timer_stop(void);
extern void timer_delay_ms(uint32_t milisecs);

#endif /* __TIMER_H_ */
