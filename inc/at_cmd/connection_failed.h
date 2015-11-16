#ifndef _CONNECTION_FAILED_H_
#define _CONNECTION_FAILED_H_

/*==================[inclusions]=============================================*/

#include "../parser.h"

/*==================[macros]=================================================*/

#undef PARSER_NAME
#define PARSER_NAME              connectionFailed
#define PARSER_INTERNALDATA_T    PARSER_INTERNALDATA_TYPE(PARSER_NAME)
#define PARSER_RESULTS_T         PARSER_RESULTS_TYPE(PARSER_NAME)
#define PARSER_DATA_T            PARSER_DATA_TYPE(PARSER_NAME)
#define PARSER_DATA_CONNECTIONFAILED_T       PARSER_DATA_TYPE(connectionFailed)
#define PARSER_RESULTS_CONNECTIONFAILED_T    PARSER_RESULTS_TYPE(connectionFailed)

/*==================[typedef]================================================*/

typedef struct {
    ParserStatus    parserState;
    fsmStatus       state;
    uint8_t         readPos;
} PARSER_INTERNALDATA_T;

typedef struct {
    uint8_t connectionID;
} PARSER_RESULTS_T;


typedef struct {
    PARSER_INTERNALDATA_T  internalData;
    PARSER_RESULTS_T       results;
} PARSER_DATA_T;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void parser_connectionFailedModule_init(void);

#endif // _CONNECTION_FAILED_H_
