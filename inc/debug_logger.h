#ifndef _DEBUG_LOGGER_H_
#define _DEBUG_LOGGER_H_

/*==================[inclusions]=============================================*/

#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_string.h"

/*==================[macros]=================================================*/

#ifndef LOGGING_ALLOWED
    #define LOGGING_ALLOWED
#endif // LOGGING_ALLOWED

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void logger_init(void);
extern void logger_print_data(uint8_t * data, size_t size);
extern void logger_print_string(const char * str);
extern ssize_t logger_read_input(void * buf, size_t size);

#endif /* _DEBUG_LOGGER_H_ */
