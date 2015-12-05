#ifndef _LITERAL_PARSER_H_
#define _LITERAL_PARSER_H_

/*==================[inclusions]=============================================*/

#include "../parser.h"

/*==================[macros]=================================================*/

#undef PARSER_NAME
#define PARSER_NAME              literalParser
#define PARSER_INTERNALDATA_T    PARSER_INTERNALDATA_TYPE(PARSER_NAME)
#define PARSER_RESULTS_T         PARSER_RESULTS_TYPE(PARSER_NAME)
#define PARSER_DATA_T            PARSER_DATA_TYPE(PARSER_NAME)
#define PARSER_DATA_LITERALPARSER_T       PARSER_DATA_TYPE(literalParser)
#define PARSER_RESULTS_LITERALPARSER_T    PARSER_RESULTS_TYPE(literalParser)

/*==================[typedef]================================================*/

typedef struct {
	char * 			string;
    ParserStatus    parserState;
    uint8_t         readPos;
} PARSER_INTERNALDATA_T;

typedef struct {
    uint8_t dummy;
} PARSER_RESULTS_T;


typedef struct {
    PARSER_INTERNALDATA_T  internalData;
    PARSER_RESULTS_T       results;
} PARSER_DATA_T;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern void parser_literalParserModule_init(void);
extern void literalParser_setStringToMatch(void* parserData, const char * str);

#endif // _LITERAL_PARSER_H_
