#ifndef ESP8266_H
#define ESP8266_H

/*==================[inclusions]=============================================*/

#include "parser.h"

/*==================[macros]=================================================*/

#define MAX_MULTIPLE_CONNECTIONS	(5)

#define AT_CIPSEND_CONTENT_DONT_COPY        (0)
#define AT_CIPSEND_CONTENT_COPYTOBUFFER     (1)

#define AT_CIPSEND_ZERO_TERMINATED_CONTENT  (0)

/*==================[typedef]================================================*/

typedef enum {
	AT_RST = 0,
	AT_CWMODE,
	AT_CWSAP,
	AT_CWSAP_CUR,
	AT_CWSAP_DEF,
	AT_CIPMUX,
	AT_CIPSERVER,
	AT_CIPSEND,
	AT_CIPSENDEX,
	AT_CIPSENDBUF,
	AT_COMMAND_SIZE
} AT_Command;


typedef enum {
    AT_TYPE_TEST    = 1 << 0,
    AT_TYPE_QUERY   = 1 << 1,
    AT_TYPE_SET     = 1 << 2,
    AT_TYPE_EXECUTE = 1 << 3
} AT_Type;


typedef enum {
    AT_CWMODE_MODE_MIN        = 1,
    AT_CWMODE_STATION         = 1,
    AT_CWMODE_SOFTAP          = 2,
    AT_CWMODE_SOFTAP_STATION  = 3,
    AT_CWMODE_MODE_MAX        = 3
} AT_CWMODE_MODE;


typedef enum {
    AT_SAP_ENCRYPTION_OPEN          = 0,
    AT_SAP_ENCRYPTION_WPA_PSK       = 2,
    AT_SAP_ENCRYPTION_WPA2_PSK      = 3,
    AT_SAP_ENCRYPTION_WPA_WPA2_PSK  = 4
} AT_SAP_ENCRYPTION;


typedef struct {
    char *      content;
    uint16_t    length;
    uint8_t     copyContentToBuffer; /* 1: se copia content a buffer interno, 0: no se lo copia, usar en caso de que content se encuentre en flash */
    uint8_t     connectionID;
} AT_CIPSEND_DATA;


typedef enum {
    AT_CIPMUX_SINGLE_CONNECTION     = 0,
    AT_CIPMUX_MULTIPLE_CONNECTION   = 1
} AT_CIPMUX_MODE;


typedef enum {
    AT_CIPSERVER_DELETE = 0,
    AT_CIPSERVER_CREATE
} AT_CIPSERVER_MODE;


typedef struct {
    AT_CIPSERVER_MODE   mode;
    uint16_t            port;
} AT_CIPSERVER_DATA;


typedef struct {
    char *              ssid;
    char *              pwd;
    uint8_t             chl;
    AT_SAP_ENCRYPTION   ecn;
} AT_CWSAP_DATA;

typedef void (*callbackCommandSentFunction_type)(AT_Command cmd);

typedef PARSER_RESULTS_IPD_T ReceivedDataInfo;
typedef void (*callbackDataReceivedFunction_type)(ReceivedDataInfo info);


typedef enum
{
    CONNECTION_STATUS_CLOSE = 0,
    CONNECTION_STATUS_OPEN = 1
} ConnectionStatus;

typedef struct
{
    ConnectionStatus newStatus;
    uint8_t connectionID;
} ConnectionInfo;

typedef void (*callbackConnectionChangedFunction_type)(ConnectionInfo info);

typedef void (*callbackResetDetectedFunction_type)(void);

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

extern int32_t esp8266_queueCommand(AT_Command command, AT_Type type, void* parameters);

extern void esp8266_registerCommandSentCallback(callbackCommandSentFunction_type fcn);
extern void esp8266_registerDataReceivedCallback(callbackDataReceivedFunction_type fcn);
extern void esp8266_registerConnectionChangedCallback(callbackConnectionChangedFunction_type fcn);
extern void esp8266_registerResetDetectedCallback(callbackResetDetectedFunction_type fcn);

extern void esp8266_doWork(void);
extern void esp8266_init(void);
extern void esp8266_setReceiveBuffer(uint8_t * buf, uint16_t size);

extern ssize_t esp8266_writeRawData(void * buf, size_t size);

extern ConnectionStatus esp8266_getConnectionStatus(uint8_t connectionID);

#endif // ESP8266_H
