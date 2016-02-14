#ifndef _PARSER_H_
#define _PARSER_H_

/*==================[inclusions]=============================================*/

#include "ciaaPOSIX_stdio.h"
#include "ciaaPOSIX_stdlib.h"

/*==================[macros]=================================================*/

#define __PARSER_RESULTS_TYPE(name)         parser_ ## name ## _results
#define __PARSER_DATA_TYPE(name)            parser_ ## name ## _data

#define PARSER_RESULTS_TYPE(name)           __PARSER_RESULTS_TYPE(name)
#define PARSER_DATA_TYPE(name)              __PARSER_DATA_TYPE(name)

#define free	ciaaPOSIX_free
#define malloc	ciaaPOSIX_malloc
#define memcpy	ciaaPOSIX_memcpy

/*==================[typedef]================================================*/

typedef enum {S0, S1, S2, S3, S4, S5} fsmStatus;

typedef enum {
    STATUS_UNINITIALIZED,
    STATUS_INITIALIZED,
    STATUS_INCOMPLETE,
    STATUS_NOT_MATCHES,
    STATUS_COMPLETE
} ParserStatus;

/* Los siguientes son los valores reservados para el tipo de parser.
   Si se agrega un nuevo parser, es conveniente agregar el tipo en este
   enumerativo.

   Si por algún razón no importa el tipo, establecerlo en un valor
   distinto a los reservados.
*/
typedef enum {
    AT_MSG_IPD = 0,
    AT_MSG_CONNECTION_OPEN,
    AT_MSG_CONNECTION_CLOSE,
	AT_MSG_CONNECTION_FAILED,
    USER_DUTYCYCLE,
    LITERAL_PARSER,
    AT_MSG_RESET,
    USER_CARACTERIZAR,
    PARSER_TYPES_COUNT
} ParserType;

typedef struct Parser_struct Parser;

typedef struct {
    void            (*init)(Parser* parserPtr);
    ParserStatus    (*tryMatch)(Parser* parserPtr, uint8_t newChar);
    void            (*deinit)(Parser* parserPtr);
} ParserFunctions;

struct Parser_struct {
    const ParserType        type;
    ParserStatus            status;
    void*                   data;
    void*                   results;
    const ParserFunctions*  functions;
};

/*==================[inclusions]=============================================*/

#include "at_cmd/ipd.h"
#include "at_cmd/connection_open.h"
#include "at_cmd/connection_close.h"
#include "at_cmd/connection_failed.h"
#include "at_cmd/literal_parser.h"
#include "user_cmd/dutycycle.h"
#include "at_cmd/reset_detection.h"
#include "user_cmd/caracterizar.h"

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void parser_initModule(void);
extern int32_t parser_init(Parser * parser);
extern ParserStatus parser_getStatus(const Parser * parser);
extern ParserStatus parser_tryMatch(Parser * parser, uint8_t newChar);
extern void* parser_getResults(const Parser * parser);
extern void parser_deinit(Parser * parser);

#endif /* _PARSER_H_ */
