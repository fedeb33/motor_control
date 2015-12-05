#ifndef ESP8266_H_
#define ESP8266_H_

/*==================[inclusions]=============================================*/

/*==================[macros]=================================================*/

#define AT_CIPSEND_CONTENT_DONT_COPY        (0)
#define AT_CIPSEND_CONTENT_COPYTOBUFFER     (1)

#define ENABLE12 	0x0040		// 00000000 01000000
#define ENABLE34    0x0080		// 00000000 10000000
#define ESP8266_EN	0x0100		// 00000010	00000000
#define ESP8266_RST 0x0200		// 00000100	00000000

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

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

int32_t esp8266_queueCommand(AT_Command command, AT_Type type, void* parameters);


#endif // ESP8266_H_
