/*==================[inclusions]=============================================*/

#include "literal_parser.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static void init(void* parserData);
static ParserStatus getStatus(void* parserData);
static void tryMatch(void* parserData, uint8_t newChar);
static void tryMatch_internal(PARSER_INTERNALDATA_T * internalData,
                              PARSER_RESULTS_T * results,
                              uint8_t newChar);
static void* getResults(void* parserData);

/*==================[internal data definition]===============================*/

static const ParserDefinition parserDef =
{
    &init,
    &getStatus,
    &tryMatch,
    &getResults,
    LITERAL_PARSER
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void init(void* parserData){
    PARSER_DATA_T * p = parserData;
    p->internalData.readPos = 0;
    p->internalData.string = 0;
    p->internalData.parserState = STATUS_INITIALIZED;
}

static ParserStatus getStatus(void* parserData){
    return (((PARSER_DATA_T*)parserData)->internalData.parserState);
}

static void tryMatch(void* parserData, uint8_t newChar){
    PARSER_DATA_T * p = parserData;
    tryMatch_internal(&(p->internalData), &(p->results), newChar);
}


static void tryMatch_internal(PARSER_INTERNALDATA_T * internalData,
                              PARSER_RESULTS_T * results,
                              uint8_t newChar){

	if (newChar == internalData->string[internalData->readPos]){
		internalData->readPos++;
		if (internalData->string[internalData->readPos] == '\0'){
			internalData->parserState = STATUS_COMPLETE;
			internalData->readPos = 0;
		}
		else{
			internalData->parserState = STATUS_INCOMPLETE;
		}
	}
	else if (internalData->readPos != 0)
	{
		internalData->readPos = 0;
		tryMatch_internal(internalData, results, newChar);
	}
	else
	{
		internalData->parserState = STATUS_NOT_MATCHES;
	}

}

static void* getResults(void* parserData){
    return 0;
}

/*==================[external functions definition]==========================*/

extern void parser_literalParserModule_init(void){
    parser_add(&parserDef);
}

extern void literalParser_setStringToMatch(void* parserData, const char * str)
{
	PARSER_DATA_T * p = parserData;
	p->internalData.string = str;
}
