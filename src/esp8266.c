/* Copyright 2014, Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


/*==================[inclusions]=============================================*/
#include "chip.h"
#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "esp8266.h"         /* <= own header */
#include "pwm.h"
#include "parser.h"
#include "encoder.h"
#include "StringUtils.h"

#if 0
volatile uint32_t * DWT_CTRL = (uint32_t *)0xE0001000;
volatile uint32_t * DWT_CYCCNT = (uint32_t *)0xE0001004;

volatile static uint32_t execTime;

void DWTStart (void)
{
	*DWT_CTRL |= 1;
	*DWT_CYCCNT = 0;
}

uint32_t DWTStop (void)
{
	uint32_t cycles = *DWT_CYCCNT;

	cycles = ((uint64_t)1000000 * cycles) / SystemCoreClock;
	if (cycles > execTime)
		execTime = cycles;

	return (cycles);
}
#endif

/*==================[macros and definitions]=================================*/

#define IPD_BUFFER_LENGTH   		(256)
#define MOTOR_COUNT					(2)
#define MAX_MULTIPLE_CONNECTIONS	(5)
#define CONNECTION_STATUS_OPEN		(1)
#define CONNECTION_STATUS_CLOSE		(0)
#define SEND_BUFFER_SIZE			(256)

#define CONTENT_EMPTY       (0)
#define CONTENT_INTERNAL    (1)
#define CONTENT_EXTERNAL    (2)

#define MAX_QUEUED_COMMANDS     (16)	/* Debe ser potencia de 2 */
#define MAX_SENDBUFFER_SIZE     (2048)	/* Debe ser potencia de 2 */
#define COMMAND_PARSERS_SIZE	(5)

#define AT_Command_toString(cmd) AT_Command_string[(cmd)]
#define isCommandTypeValid(command, type) ((valid_types[(command)] & (uint8_t)(type)) != 0)

#define internalBufferedData_getWroteLength(bufInfo) (((InternalBufferedDataInfo)(bufInfo)).length)
#define internalBufferedData_wasDataWritten(bufInfo) (internalBufferedData_getWroteLength(bufInfo) > 0)
#define internalBuffer_deleteLastN(N) (((uint16_t)(N) < internalBuffer.count) ? (internalBuffer.count -= (uint16_t)(N)) : (internalBuffer.count = 0))
#define internalBuffer_charAt(index) (internalBuffer.buffer[(uint16_t)(index)])

/*==================[internal data declaration]==============================*/

typedef struct {
	uint16_t startIndex;
	uint16_t length;
} InternalBufferedDataInfo;

typedef struct {
	char * buffer;
	uint16_t length;
} ExternalBufferedDataInfo;

typedef struct {
	AT_Command                  command;
	AT_Type                     type;
	InternalBufferedDataInfo    paramsData;
	uint8_t                     contentInfo;
	union{
		InternalBufferedDataInfo internal;
		ExternalBufferedDataInfo external;
	} content;
	// callback?
} QueuedCommand;

typedef int32_t (*paramsToString_type)(QueuedCommand* commandData, void* parameters);

typedef struct {
	char *      buffer;
	uint16_t    front;
	uint16_t    count;
	uint16_t    size;
} CircularBuffer;

typedef int32_t (*waitFunction_type)(void);

/*==================[internal functions declaration]=========================*/

static inline const char * AT_Type_toString(const AT_Type type);

static int32_t paramsToString_cwmode(QueuedCommand* commandData, void* parameters);
static int32_t paramsToString_cipsend(QueuedCommand* commandData, void* parameters);
static int32_t paramsToString_cwsap(QueuedCommand* commandData, void* parameters);
static int32_t paramsToString_cipmux(QueuedCommand* commandData, void* parameters);
static int32_t paramsToString_cipserver(QueuedCommand* commandData, void* parameters);

static int32_t queue_cmd_push(QueuedCommand newCommand);
static QueuedCommand queue_cmd_pop(void);
static uint8_t queue_cmd_isEmpty(void);

/*==================[internal data definition]===============================*/
/** \brief File descriptor for digital input ports
 *
 * Device path /dev/dio/in/0
 */
static int32_t fd_in;

/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
static int32_t fd_out;

/** \brief File descriptor of the USB uart
 *
 * Device path /dev/serial/uart/1
 */
static int32_t fd_uart1;

/** \brief File descriptor of the RS232 uart
 *
 * Device path /dev/serial/uart/2
 */
static int32_t fd_uart2;


static uint8_t bufferIPD[IPD_BUFFER_LENGTH];
static PARSER_DATA_IPD_T parserIPD_data;
static const Parser parserIPD = {AT_MSG_IPD, &parserIPD_data};

static PARSER_DATA_DUTYCYCLE_T parserDutyCycle_data;
static const Parser parserDutyCycle = {USER_DUTYCYCLE, &parserDutyCycle_data};

static PARSER_DATA_CONNECTIONOPEN_T parserConnectionOpen_data;
static const Parser parserConnectionOpen = {AT_MSG_CONNECTION_OPEN, &parserConnectionOpen_data};

static PARSER_DATA_CONNECTIONCLOSE_T parserConnectionClose_data;
static const Parser parserConnectionClose = {AT_MSG_CONNECTION_CLOSE, &parserConnectionClose_data};

static PARSER_DATA_CONNECTIONFAILED_T parserConnectionFailed_data;
static const Parser parserConnectionFailed = {AT_MSG_CONNECTION_FAILED, &parserConnectionFailed_data};

static PARSER_DATA_LITERALPARSER_T parserLiteral_data[3];
static const Parser parserLiteral[3] =
{
		{LITERAL_PARSER, &parserLiteral_data[0]},
		{LITERAL_PARSER, &parserLiteral_data[1]},
		{LITERAL_PARSER, &parserLiteral_data[2]}
};

static uint8_t connectionStatus[MAX_MULTIPLE_CONNECTIONS];
MotorControlData  lastDutyCycle[MOTOR_COUNT];

static const char * AT_Command_string[AT_COMMAND_SIZE] = {
		"AT+RST",
		"AT+CWMODE",
		"AT+CWSAP",
		"AT+CWSAP_CUR",
		"AT+CWSAP_DEF",
		"AT+CIPMUX",
		"AT+CIPSERVER",
		"AT+CIPSEND",
		"AT+CIPSENDEX",
		"AT+CIPSENDBUF"
};

static const char * AT_Type_string[4] = {
		"=?",   /* Test */
		"?",    /* Query */
		"=",    /* Set */
		""      /* Execute */
};

static const uint8_t valid_types[AT_COMMAND_SIZE] = {
		AT_TYPE_EXECUTE,                            /* <= AT+RST */
		AT_TYPE_QUERY | AT_TYPE_SET | AT_TYPE_TEST, /* <= AT+CWMODE */
		AT_TYPE_QUERY | AT_TYPE_SET,                /* <= AT+CWSAP */
		AT_TYPE_QUERY | AT_TYPE_SET,                /* <= AT+CWSAP_CUR */
		AT_TYPE_QUERY | AT_TYPE_SET,                /* <= AT+CWSAP_DEF */
		AT_TYPE_QUERY | AT_TYPE_SET,                /* <= AT+CIPMUX */
		AT_TYPE_QUERY | AT_TYPE_SET,                /* <= AT+CIPSERVER */
		AT_TYPE_SET,                                /* <= AT+CIPSEND */
		AT_TYPE_SET,                                /* <= AT+CIPSENDEX */
		AT_TYPE_SET,                                /* <= AT+CIPSENDBUF */
};

static const paramsToString_type paramsToString[AT_COMMAND_SIZE] = {
		0,
		&paramsToString_cwmode,
		&paramsToString_cwsap,
		&paramsToString_cwsap,
		&paramsToString_cwsap,
		&paramsToString_cipmux,
		&paramsToString_cipserver,
		&paramsToString_cipsend,
		&paramsToString_cipsend,
		&paramsToString_cipsend
};

static const uint8_t maxRetryNumber[AT_COMMAND_SIZE] =
{
		5,  /* <= AT+RST */
		1,  /* <= AT+CWMODE */
		3,  /* <= AT+CWSAP */
		3,  /* <= AT+CWSAP_CUR */
		3,  /* <= AT+CWSAP_DEF */
		1,  /* <= AT+CIPMUX */
		1,  /* <= AT+CIPSERVER */
		2,  /* <= AT+CIPSEND */
		2,  /* <= AT+CIPSENDEX */
		1,  /* <= AT+CIPSENDBUF */
};

static QueuedCommand command_queue[MAX_QUEUED_COMMANDS];
static uint8_t queue_front = 0;
static uint8_t queue_count = 0;

static char internalBuffer_data[MAX_SENDBUFFER_SIZE];
static CircularBuffer internalBuffer = {internalBuffer_data, 0, 0, MAX_SENDBUFFER_SIZE};

static waitFunction_type waitFunctions[AT_COMMAND_SIZE] = {0};

static const Parser* cmd_parsers[COMMAND_PARSERS_SIZE];
static uint8_t cmd_parsers_length = 0;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static inline void delayMS(uint16_t milliseconds){
	volatile uint16_t i;
	while (milliseconds--){
		for (i = 0; i < (18360); i++);
	}
}

static inline const char * AT_Type_toString(const AT_Type type){
	switch(type){
	case AT_TYPE_TEST:
		return AT_Type_string[0];
		break;
	case AT_TYPE_QUERY:
		return AT_Type_string[1];
		break;
	case AT_TYPE_SET:
		return AT_Type_string[2];
		break;
	default:
		return AT_Type_string[3];
	}
}

static int32_t queue_cmd_push(QueuedCommand newCommand)
{
	int32_t ret = -1;

	if (queue_count < MAX_QUEUED_COMMANDS){
		command_queue[(queue_front + queue_count) & (MAX_QUEUED_COMMANDS - 1)] = newCommand;
		queue_count++;
		ret = 1;
	}

	return ret;
}

static QueuedCommand queue_cmd_pop(void){
	QueuedCommand ret;

	if (queue_count > 0){
		ret = command_queue[queue_front];
		queue_front = (uint8_t)((queue_front + 1) & (MAX_QUEUED_COMMANDS - 1));
		queue_count--;
	}

	return ret;
}

static uint8_t queue_cmd_isEmpty(void){
	if (queue_count == 0){
		return 1;
	}else{
		return 0;
	}
}

static inline void cmd_parsers_add(const Parser* parser)
{
	if (cmd_parsers_length < COMMAND_PARSERS_SIZE && parser != NULL)
	{
		cmd_parsers[cmd_parsers_length++] = parser;
	}
}

static inline void cmd_parsers_clear(void)
{
	cmd_parsers_length = 0;
}

static void esp8266_setCommandWaitFunction(AT_Command command, waitFunction_type fcn)
{
	if (command < AT_COMMAND_SIZE)
		waitFunctions[command] = fcn;
}

int32_t esp8266_queueCommand(AT_Command command, AT_Type type, void* parameters){
	int32_t ret = 0;
	QueuedCommand newCommand = {0};

	/* Verifico que el comando sea uno de los definidos, y que el tipo sea compatible con este */
	if (command >= AT_COMMAND_SIZE || !isCommandTypeValid(command, type)){
		return -1;
	}

	newCommand.command = command;
	newCommand.type = type;
	newCommand.contentInfo = CONTENT_EMPTY;

	if (type == AT_TYPE_SET){ /* Sólo los comandos de tipo SET necesitan parámetros al ser llamados */
		ret = paramsToString[command](&newCommand, parameters);
	}

	if (ret >= 0)
	{
		ret = queue_cmd_push(newCommand);
	}

	return ret;
}


static InternalBufferedDataInfo writeDataToBuffer(const char * buf, uint16_t length)
{
	InternalBufferedDataInfo ret = {0, 0};
	uint16_t i, size_1;

	/* Se comprueba que haya espacio en el buffer interno */
	if (internalBuffer.size - internalBuffer.count >= length)
	{
		/* Se busca evitar cálculos repetidos */
		size_1 = internalBuffer.size - 1;

		/* La posición inicial de los datos en el buffer coincidirá con
        la primera posición disponible para escritura */
		ret.startIndex = (internalBuffer.front + internalBuffer.count) & size_1;
		ret.length = length;

		/* Como hay espacio disponible, copio los datos */
		for (i = 0; i < length; i++)
		{
			internalBuffer.buffer[(ret.startIndex + i) & size_1] = buf[i];
		}

		internalBuffer.count += length;
	}

	return ret;
}

static InternalBufferedDataInfo writeStringToBuffer(const char * str)
{
	InternalBufferedDataInfo ret = {0, 0};
	uint16_t originalCount;
	uint16_t size_1;

	/* Compruebo que la string no esté vacía */
	if (*str != '\0')
	{
		originalCount = internalBuffer.count;
		size_1 = internalBuffer.size - 1;

		/* Mientras no llegue al final de la string y haya lugar en el buffer, guardo en él */
		while (*str != '\0' && internalBuffer.count < internalBuffer.size)
		{
			internalBuffer.buffer[(internalBuffer.front + internalBuffer.count) & size_1] = *str;
			str++;
			internalBuffer.count++;
		}

		/* Si se llega al límite de almacenamiento pero quedan caracteres sin guardar */
		if (internalBuffer.count == internalBuffer.size && *str != '\0')
		{
			/* Elimino lo guardado, retornando que no se guardó ningún carácter */
			internalBuffer.count = originalCount;
		}
		else
		{
			ret.startIndex = (internalBuffer.front + originalCount) & size_1;
			ret.length = internalBuffer.count - originalCount;
		}

	}

	return ret;
}




static int32_t paramsToString_cwmode(QueuedCommand* cmd, void* parameters){
	AT_CWMODE_MODE mode = (AT_CWMODE_MODE)parameters;
	char buf;

	if (mode >= AT_CWMODE_MODE_MIN && mode <= AT_CWMODE_MODE_MAX){
		buf = '0' + mode;
		cmd->paramsData = writeDataToBuffer(&buf, 1);
		if (internalBufferedData_wasDataWritten(cmd->paramsData)){
			return 1;
		}
	}
	return -1;
}

static int32_t paramsToString_cipmux(QueuedCommand* cmd, void* parameters){
	AT_CIPMUX_MODE mode = (AT_CIPMUX_MODE)parameters;
	char buf;

	if (mode <= AT_CIPMUX_MULTIPLE_CONNECTION){
		buf = '0' + mode;
		cmd->paramsData = writeDataToBuffer(&buf, 1);
		if (internalBufferedData_wasDataWritten(cmd->paramsData)){
			return 1;
		}
	}

	return -1;
}

static int32_t paramsToString_cipserver(QueuedCommand* cmd, void* parameters){
	AT_CIPSERVER_DATA* data = (AT_CIPSERVER_DATA*)parameters;
	char paramStr[8] = "N,XXXXX";

	if (data->mode <= AT_CIPSERVER_CREATE) /* Si el modo tiene un valor correcto */
	{
		paramStr[0] = '0' + data->mode;
		if (data->port == 0) /* Si el puerto es inválido */
		{
			/* Se usa el puerto por defecto del ESP8266 */
			paramStr[1] = '\0';
		}
		else
		{
			uintToString(data->port, 1, (unsigned char *)&paramStr[2]);
		}

		cmd->paramsData = writeStringToBuffer(paramStr);
		if (internalBufferedData_wasDataWritten(cmd->paramsData))
		{
			return internalBufferedData_getWroteLength(cmd->paramsData);
		}
	}

	return -1;
}

static int32_t paramsToString_cipsend(QueuedCommand* cmd, void* parameters){
	AT_CIPSEND_DATA* data = (AT_CIPSEND_DATA*)parameters;
	int32_t cantChars;
	char paramStr[7] = "N,XXXX";

	if (data->content == 0)
	{
		return -1;
	}

	if (data->length == 0)
	{
		data->length = ciaaPOSIX_strlen(data->content);
	}

	if (data->length > 0)
	{
		paramStr[0] = '0' + data->connectionID;
		uintToString(data->length, 1, (unsigned char *)&paramStr[2]);
		cmd->paramsData = writeStringToBuffer(paramStr);
		if (internalBufferedData_wasDataWritten(cmd->paramsData))
		{
			cantChars = internalBufferedData_getWroteLength(cmd->paramsData);
			if (data->copyContentToBuffer == AT_CIPSEND_CONTENT_COPYTOBUFFER)
			{
				cmd->content.internal = writeDataToBuffer(data->content, data->length);
				cmd->contentInfo = CONTENT_INTERNAL;
				if (!internalBufferedData_wasDataWritten(cmd->content.internal))
				{
					/* No hay lugar en el buffer */
					/* Elimino la cadena de los parámetros */
					internalBuffer_deleteLastN(internalBufferedData_getWroteLength(cmd->paramsData));
					return -1;
				}

				cantChars += internalBufferedData_getWroteLength(cmd->content.internal);
			}
			else if (data->copyContentToBuffer == AT_CIPSEND_CONTENT_DONT_COPY)
			{
				cmd->content.external.buffer = data->content;
				cmd->content.external.length = data->length;
				cmd->contentInfo = CONTENT_EXTERNAL;
			}

			return cantChars;
		}
	}

	return -1;
}


size_t strlcpy(char *dest, const char *src, size_t size)
{
	size_t i;

	/* Debo copiar a dest como mucho size-1 caracteres de src, o hasta llegar a su fin */
	size--;
	for (i = 0; i < size && src[i] != '\0'; i++)
	{
		dest[i] = src[i];
	}

	/* Siempre se finaliza la cadena con el carácter nulo */
	dest[i] = '\0';

	/* Cuento los caracteres que restan de src, sin copiarlos a dest */
	while(src[i] != '\0')
		i++;

	/* Retorno la longitud de la cadena src */
	return i;
}

static int32_t paramsToString_cwsap(QueuedCommand* cmd, void* parameters){
	AT_CWSAP_DATA* data = (AT_CWSAP_DATA*)parameters;
	char paramStr[107];
	char * ptr = paramStr;
	int cant;

	/* Formato para Set: AT+CWSAP=<ssid>,<pwd>,<chl>,<ecn>
       ssid: string, SSID del soft AP del ESP8266, 32 chars máx.
       pwd: string, contraseña del soft AP, de 8 a 64 chars.
       chl: int, channel id, de 1 a 14 para 802.11b/g/n.
       ecn: int, encriptación empleada en la comunicación, posibles valores: 0, 2, 3, 4.
       Cantidad máxima de caracteres necesaria: 32 + 64 + 2 + 1 + 4 (comillas) + 3 (comas) + '\0' = 107
	 */

	if (data->ssid == 0 || data->pwd == 0 || data->chl < 1 || data->chl > 14)
	{
		return -1;
	}

	*ptr++ = '"';
	cant = strlcpy(&paramStr[1], data->ssid, 32 + 1);
	if (cant <= 32)
	{
		ptr += cant;
		*ptr++ = '"';
		*ptr++ = ',';
		*ptr++ = '"';
		cant = strlcpy(ptr, data->pwd, 64 + 1);

		/* Si no hay encriptación no hace falta contraseña */
		if ((data->ecn == AT_SAP_ENCRYPTION_OPEN) || (cant >= 8 && cant <= 64))
		{
			ptr += cant;
			*ptr++ = '"';
			*ptr++ = ',';
			ptr = (char *)uintToString(data->chl, 1, (unsigned char *)ptr);
			*ptr++ = ',';
			ptr = (char *)uintToString(data->ecn, 1, (unsigned char *)ptr);

			cmd->paramsData = writeStringToBuffer(paramStr);
			if (internalBufferedData_wasDataWritten(cmd->paramsData))
			{
				return internalBufferedData_getWroteLength(cmd->paramsData);
			}
		}
	}

	return -1;
}

#define WAIT_RESULT_BUSY		(0)
#define WAIT_RESULT_OK			(1)
#define WAIT_RESULT_TIMEOUT		(-1)
#define WAIT_RESULT_ERROR		(-2)

static int32_t rst_wait(void)
{
	uint16_t timeoutMS = 1500;

	/* config parser */
	parser_init(&parserLiteral[0]);
	literalParser_setStringToMatch(parserLiteral[0].data, "\r\nready");

	/* activate parser */
	cmd_parsers_add(&parserLiteral[0]);

	/* wait for "\r\nready" OR timeout */
	while (parser_getStatus(&parserLiteral[0]) != STATUS_COMPLETE && timeoutMS-- > 0)
	{
		delayMS(1);
	}

	cmd_parsers_clear();

	return (timeoutMS != 0 ? WAIT_RESULT_OK : WAIT_RESULT_TIMEOUT);
}

static int32_t wait_OK_busy_error(void)
{
	uint16_t timeoutMS = 1500;
	int32_t ret = WAIT_RESULT_TIMEOUT;

	/* config parser */
	parser_init(&parserLiteral[0]);
	literalParser_setStringToMatch(parserLiteral[0].data, "\r\nOK");
	parser_init(&parserLiteral[1]);
	literalParser_setStringToMatch(parserLiteral[1].data, "busy p...");
	parser_init(&parserLiteral[2]);
	literalParser_setStringToMatch(parserLiteral[2].data, "\r\nERROR");

	/* activate parser */
	cmd_parsers_add(&parserLiteral[0]);
	cmd_parsers_add(&parserLiteral[1]);
	cmd_parsers_add(&parserLiteral[2]);

	/* wait for the three parsers OR timeout */
	while (timeoutMS-- > 0 && ret == WAIT_RESULT_TIMEOUT)
	{
		if (parser_getStatus(&parserLiteral[0]) == STATUS_COMPLETE)
		{
			ret = WAIT_RESULT_OK;
		}
		else if (parser_getStatus(&parserLiteral[1]) == STATUS_COMPLETE)
		{
			ret = WAIT_RESULT_BUSY;
		}
		else if (parser_getStatus(&parserLiteral[1]) == STATUS_COMPLETE)
		{
			ret = WAIT_RESULT_ERROR;
		}
		else
		{
			delayMS(1);
		}
	}

	cmd_parsers_clear();

	return ret;
}

static int32_t wait_cipsend(void)
{
	uint16_t timeoutMS = 1000;
	int32_t ret = WAIT_RESULT_TIMEOUT;

	/* config parser */
	parser_init(&parserLiteral[0]);
	literalParser_setStringToMatch(parserLiteral[0].data, "OK\r\n>");
	parser_init(&parserLiteral[1]);
	literalParser_setStringToMatch(parserLiteral[1].data, "busy p...");
	parser_init(&parserLiteral[2]);
	literalParser_setStringToMatch(parserLiteral[2].data, "\r\nERROR");

	/* activate parser */
	cmd_parsers_add(&parserLiteral[0]);
	cmd_parsers_add(&parserLiteral[1]);
	cmd_parsers_add(&parserLiteral[2]);

	/* wait for the three parsers OR timeout */
	while (timeoutMS-- > 0 && ret == WAIT_RESULT_TIMEOUT)
	{
		if (parser_getStatus(&parserLiteral[0]) == STATUS_COMPLETE)
		{
			ret = WAIT_RESULT_OK;
		}
		else if (parser_getStatus(&parserLiteral[1]) == STATUS_COMPLETE)
		{
			ret = WAIT_RESULT_BUSY;
		}
		else if (parser_getStatus(&parserLiteral[1]) == STATUS_COMPLETE)
		{
			ret = WAIT_RESULT_ERROR;
		}
		else
		{
			delayMS(1);
		}
	}

	cmd_parsers_clear();

	return ret;
}

static int32_t waitFor(const char * str)
{
	uint16_t timeoutMS = 1000;

	/* config parser */
	parser_init(&parserLiteral[0]);
	literalParser_setStringToMatch(parserLiteral[0].data, str);


	/* activate parser */
	cmd_parsers_add(&parserLiteral[0]);

	/* wait for str */
	while (parser_getStatus(&parserLiteral[0]) != STATUS_COMPLETE && timeoutMS-- > 0)
	{
		delayMS(1);
	}

	cmd_parsers_clear();

	return (timeoutMS != 0 ? WAIT_RESULT_OK : WAIT_RESULT_TIMEOUT);
}

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
	/* Starts the operating system in the Application Mode 1 */
	/* This example has only one Application Mode */
	StartOS(AppMode1);

	/* StartOs shall never returns, but to avoid compiler warnings or errors
	 * 0 is returned */
	return 0;
}

/** \brief Error Hook function
 *
 * This function is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
	ciaaPOSIX_printf("ErrorHook was called\n");
	ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
	ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
	uint16_t gpio_buffer;
	/* init CIAA kernel and devices */
	ciaak_start();

	/* open CIAA digital inputs */
	fd_in = ciaaPOSIX_open("/dev/dio/in/0", ciaaPOSIX_O_RDONLY);

	/* open CIAA digital outputs */
	fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);

	/* open UART connected to USB bridge (FT2232) */
	fd_uart1 = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR | ciaaPOSIX_O_NONBLOCK);

	/* open UART connected to RS232 connector */
	fd_uart2 = ciaaPOSIX_open("/dev/serial/uart/2", ciaaPOSIX_O_RDWR | ciaaPOSIX_O_NONBLOCK);

	/* change baud rate */
	ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);
	ciaaPOSIX_ioctl(fd_uart2, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

	/* change FIFO TRIGGER LEVEL for uart usb */
	ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

	/* Starts PWM configuration */
	ciaaPWM_init();

	parser_initModule();

	parser_init(&parserIPD);
	parserIPD_data.results.bufferLength = IPD_BUFFER_LENGTH;
	parserIPD_data.results.buffer = bufferIPD;

	parser_init(&parserConnectionClose);
	parser_init(&parserConnectionOpen);
	parser_init(&parserConnectionFailed);
	parser_init(&parserDutyCycle);

	encoder_init();

	/* Configuracion de los GPIO de salida. */
	//ciaaPOSIX_read(fd_out,&gpio_buffer,2);
	/* Se guardan los valores que vienen por defecto en la lectura y se setean en 0
	 * 	aquellos que luego seran cambiados en la escritura. */
	//gpio_buffer &= ~(ENABLE12|ENABLE34|ESP8266_EN|ESP8266_RST);
	/* Habilitacion de los enable, /reset y chip_enable del puente H. */
	//gpio_buffer ^= (ENABLE12|ENABLE34|ESP8266_EN|ESP8266_RST);
	gpio_buffer |= (ENABLE12|ENABLE34|ESP8266_EN|ESP8266_RST);
	ciaaPOSIX_write(fd_out,&gpio_buffer,2);

	SetRelAlarm(ActivatePeriodicTask, 10, 5);
	SetRelAlarm(ActivateSendStatusTask, 250, 1000);
	ActivateTask(SerialEchoTask);

	/* end InitTask */
	TerminateTask();
}


TASK(SerialEchoTask)
{
	int8_t buf[20];    // buffer for uart operation
	int32_t ret;       // return value variable for posix calls

	QueuedCommand cmd;
	AT_CWSAP_DATA cwsap_data = {"wifi", "12345678", 11, AT_SAP_ENCRYPTION_WPA2_PSK};
	AT_CIPSERVER_DATA cipserver_data = {AT_CIPSERVER_CREATE, 8080};
	uint8_t retry, haveToRetry;

	delayMS(900);

	esp8266_setCommandWaitFunction(AT_RST, rst_wait);
	esp8266_setCommandWaitFunction(AT_CWMODE, wait_OK_busy_error);
	esp8266_setCommandWaitFunction(AT_CWSAP_CUR, wait_OK_busy_error);
	esp8266_setCommandWaitFunction(AT_CIPMUX, wait_OK_busy_error);
	esp8266_setCommandWaitFunction(AT_CIPSERVER, wait_OK_busy_error);
	esp8266_setCommandWaitFunction(AT_CIPSENDBUF, wait_cipsend);

	esp8266_queueCommand(AT_RST, AT_TYPE_EXECUTE, 0);
	esp8266_queueCommand(AT_CWMODE, AT_TYPE_SET, (void*)AT_CWMODE_SOFTAP);
	esp8266_queueCommand(AT_CWSAP_CUR, AT_TYPE_SET, &cwsap_data);
	esp8266_queueCommand(AT_CIPMUX, AT_TYPE_SET, (void*)AT_CIPMUX_MULTIPLE_CONNECTION);
	esp8266_queueCommand(AT_CIPSERVER, AT_TYPE_SET, &cipserver_data);

	while(1)
	{
		// wait for any character ...
		ret = ciaaPOSIX_read(fd_uart1, buf, sizeof(buf));

		if(ret > 0)
		{
			// also write them to the other device
			ciaaPOSIX_write(fd_uart2, buf, ret);
		}

		while(!queue_cmd_isEmpty()){
			cmd = queue_cmd_pop();

			haveToRetry = 1;
			for (retry = 1; retry <= maxRetryNumber[cmd.command] && haveToRetry; retry++)
			{
				ciaaPOSIX_write(fd_uart2, AT_Command_toString(cmd.command), ciaaPOSIX_strlen(AT_Command_toString(cmd.command)));
				ciaaPOSIX_write(fd_uart2, AT_Type_toString(cmd.type), ciaaPOSIX_strlen(AT_Type_toString(cmd.type)));

				if (internalBufferedData_getWroteLength(cmd.paramsData) + internalBuffer.front > internalBuffer.size)
				{
					/* Hay wrapping */
					ciaaPOSIX_write(fd_uart2, &internalBuffer.buffer[internalBuffer.front], internalBuffer.size - internalBuffer.front);
					ciaaPOSIX_write(fd_uart2, &internalBuffer.buffer[0], internalBufferedData_getWroteLength(cmd.paramsData) - (internalBuffer.size - internalBuffer.front));
				}
				else
				{
					ciaaPOSIX_write(fd_uart2, &internalBuffer.buffer[internalBuffer.front], internalBufferedData_getWroteLength(cmd.paramsData));
				}

				ciaaPOSIX_write(fd_uart2, "\r\n", 2);
				if (waitFunctions[cmd.command] != NULL && waitFunctions[cmd.command]() <= WAIT_RESULT_BUSY)
				{
					/* Hubo error al esperar (timeout, etc), por lo cual reintento si es posible */
					continue;
				}

				/* En teoría el comando ha sido enviado correctamente.
	               Borro la información del buffer. */

				internalBuffer.front = (internalBuffer.front + internalBufferedData_getWroteLength(cmd.paramsData)) & (internalBuffer.size - 1);
				internalBuffer_deleteLastN(internalBufferedData_getWroteLength(cmd.paramsData));

				if (cmd.contentInfo != CONTENT_EMPTY)
				{
					if (cmd.contentInfo == CONTENT_INTERNAL)
					{
						if (internalBufferedData_getWroteLength(cmd.content.internal) + internalBuffer.front > internalBuffer.size)
						{
							/* Hay wrapping */
							ciaaPOSIX_write(fd_uart2, &internalBuffer.buffer[internalBuffer.front], internalBuffer.size - internalBuffer.front);
							ciaaPOSIX_write(fd_uart2, &internalBuffer.buffer[0], internalBufferedData_getWroteLength(cmd.content.internal) - (internalBuffer.size - internalBuffer.front));
						}
						else
						{
							ciaaPOSIX_write(fd_uart2, &internalBuffer.buffer[internalBuffer.front], internalBufferedData_getWroteLength(cmd.content.internal));
						}

						internalBuffer.front = (internalBuffer.front + internalBufferedData_getWroteLength(cmd.content.internal)) & (internalBuffer.size - 1);
						internalBuffer_deleteLastN(internalBufferedData_getWroteLength(cmd.content.internal));
					}
					else
					{
						ciaaPOSIX_write(fd_uart2, cmd.content.external.buffer, cmd.content.external.length);
					}

					ciaaPOSIX_strcpy((char *)uintToString((cmd.contentInfo == CONTENT_INTERNAL) ? internalBufferedData_getWroteLength(cmd.content.internal) : cmd.content.external.length,
											 	  	  	  1,
														  (unsigned char *)ciaaPOSIX_strcpy((char *)buf, "Recv ")),
									 " byte");
					waitFor((char*)buf);
					/* If connection cannot be established,or it’s not a TCP connection , or buffer
					   full, or some other error occurred, returns
					   ERROR
					*/
				}

				// Callback...
				/* Terminó correctamente la ejecución del comando, no reintento más */
				haveToRetry = 0;
				if (waitFunctions[cmd.command] == NULL)
				{
					/* Si el comando no tiene función de espera para confirmar la finalización de su ejecución,
					 * se asume un delay.
					 */
					delayMS(200);
				}

			}

			if (haveToRetry)
			{
				/* Se excedió la máxima cantidad de reintentos, hacer algo */

				/* Elimino el comando del buffer, el contenido (si está presente) también */
				internalBuffer.front = (internalBuffer.front + internalBufferedData_getWroteLength(cmd.paramsData)) & (internalBuffer.size - 1);
				internalBuffer_deleteLastN(internalBufferedData_getWroteLength(cmd.paramsData));

				if (cmd.contentInfo == CONTENT_INTERNAL)
				{
					internalBuffer.front = (internalBuffer.front + internalBufferedData_getWroteLength(cmd.content.internal)) & (internalBuffer.size - 1);
					internalBuffer_deleteLastN(internalBufferedData_getWroteLength(cmd.content.internal));
				}

			}

		}

	}
}


TASK(SendStatusTask){
	uint8_t encoder_data[] = "$SPEEDXTVALOR$";
	uint8_t i;
	unsigned char * ptr;
	AT_CIPSEND_DATA cipsend_data;

	cipsend_data.connectionID = MAX_MULTIPLE_CONNECTIONS;

	for (i = 0; i < MAX_MULTIPLE_CONNECTIONS; i++){
		if (connectionStatus[i] == CONNECTION_STATUS_OPEN){
			cipsend_data.connectionID = i;
			break;
		}
	}


	if (cipsend_data.connectionID < MAX_MULTIPLE_CONNECTIONS){

		cipsend_data.content = (char *)encoder_data;
		cipsend_data.length = 0; /* Dejo que el comando calcule internamente la longitud */
		cipsend_data.copyContentToBuffer = AT_CIPSEND_CONTENT_COPYTOBUFFER;

		for (i = 0; i < ENCODER_COUNT; i++){
			ptr = uintToString(i, 1, &(encoder_data[6]));
			*ptr = '0' + SPEED_TYPE_RPM;
			ptr = uintToString(encoder_getAverageRPM(i), 4, &(encoder_data[8]));
			*ptr++ = '$';
			*ptr = '\0';
			esp8266_queueCommand(AT_CIPSENDBUF, AT_TYPE_SET, &cipsend_data);
		}
	}

	TerminateTask();
}

static const char staticResponseHeaders[] =
		"HTTP/1.1 200 OK\r\n"
		"Cache-Control: no-cache\r\n"
		"Content-Type: text/html; charset=iso-8859-1\r\n"
		"Content-Length: 146\r\n"
		"Connection: close\r\n"
		"\r\n"
		"<html><head><title>ESP8266</title></head>\r\n"
		"<body>\r\n"
		"<h1>Control de motores via WiFi</h1>\r\n"
		"<h3>Primera prueba de servidor Web</h3>\r\n"
		"</body>\r\n"
		"</html>";

TASK(PeriodicTask)
{
	int8_t buf[64];
	int8_t newChar;
	int32_t ret;
	uint8_t i;
	uint16_t j;

	AT_CIPSEND_DATA cipsend_data;

	ret = ciaaPOSIX_read(fd_uart2, buf, sizeof(buf));
	if(ret > 0)
	{

		/* Echo in serial terminal */
		ciaaPOSIX_write(fd_uart1, buf, ret);

		for (i = 0; i < ret; i++){
			newChar = buf[i];

			if (parser_tryMatch(&parserIPD, newChar) == STATUS_COMPLETE){

				if (ciaaPOSIX_strncmp("GET / HTTP/1.1", bufferIPD, 14) == 0)
				{
					/* El mensaje es un HTTP request, entonces envío la respuesta */
					cipsend_data.connectionID = parserIPD_data.results.connectionID;
					cipsend_data.content = staticResponseHeaders;
					cipsend_data.length = sizeof(staticResponseHeaders);
					cipsend_data.copyContentToBuffer = AT_CIPSEND_CONTENT_DONT_COPY;

					esp8266_queueCommand(AT_CIPSENDBUF, AT_TYPE_SET, &cipsend_data);
				}

				/* Pongo el número de motor en un valor incorrecto. Esto es para evitar
				 * actualizar las salidas PWM si es que no se recibió un comando de
				 * usuario para ello.
				 */
				for (j = 0; j < MOTOR_COUNT; j++){
					lastDutyCycle[j].motorID = MOTOR_COUNT + 1;
				}

				for (j = 0; j < (parserIPD_data.results.payloadLength > parserIPD_data.results.bufferLength ? parserIPD_data.results.bufferLength : parserIPD_data.results.payloadLength); j++){
					if (parser_tryMatch(&parserDutyCycle, bufferIPD[j]) == STATUS_COMPLETE &&
							parserDutyCycle_data.results.motorID < MOTOR_COUNT){

						lastDutyCycle[parserDutyCycle_data.results.motorID] = parserDutyCycle_data.results;

					}
				}

				for (j = 0; j < MOTOR_COUNT; j++){
					ciaaPWM_updateMotor(lastDutyCycle[j]);
				}
			}

			if (!parser_ipd_isDataBeingSaved(parserIPD.data))
			{
				/* Si no se está guardando en el buffer significa que no hay match
				 * o todavía no se completó la parte que precede al contenido mismo.
				 * Por tanto, es correcto intentar matchear los demás casos.
				 * Una vez que se comienza a procesar el contenido de IPD, se ignora
				 * al resto de los parsers ya que se supone que el contenido no es
				 * interferido. */

				for (j = 0; j < cmd_parsers_length; j++)
				{
					if (parser_getStatus(cmd_parsers[j]) != STATUS_COMPLETE)
					{
						parser_tryMatch(cmd_parsers[j], newChar);
					}
				}

				if (parser_tryMatch(&parserConnectionOpen, newChar) == STATUS_COMPLETE){
					connectionStatus[parserConnectionOpen_data.results.connectionID] = CONNECTION_STATUS_OPEN;
				}

				if (parser_tryMatch(&parserConnectionClose, newChar) == STATUS_COMPLETE){
					connectionStatus[parserConnectionClose_data.results.connectionID] = CONNECTION_STATUS_CLOSE;
				}

				if (parser_tryMatch(&parserConnectionFailed, newChar) == STATUS_COMPLETE){
					connectionStatus[parserConnectionFailed_data.results.connectionID] = CONNECTION_STATUS_CLOSE;
				}
			}

		}

	}

	TerminateTask();
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

