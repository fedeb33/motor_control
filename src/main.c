
/*==================[inclusions]=============================================*/
#include "chip.h"
#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "main.h"			  /* <= own header */
#include "esp8266.h"
#include "pwm.h"
#include "parser.h"
#include "encoder.h"
#include "StringUtils.h"
#include "debug_logger.h"

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

#define RECEIVE_BUFFER_LENGTH   	(2048)
#define MOTOR_COUNT					(2)

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static void SendStatus(void);
static void SendDatosCaracterizar(void);
static void ReceiveData(ReceivedDataInfo info);
static void WiFiReset(void);

static void ComenzarCaracterizar(PARSER_RESULTS_CARACTERIZAR_T * infoPtr, uint8_t connectionID);
static void FinalizarCaracterizar(void);

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


static uint8_t receiveBuffer[RECEIVE_BUFFER_LENGTH];

static Parser parserDutyCycle = INITIALIZER_DUTYCYCLE;
static Parser parserCaracterizar = INITIALIZER_CARACTERIZAR;
static Parser parserCancelarCaracterizar = INITIALIZER_LITERAL_PARSER;

static MotorControlData  lastDutyCycle[MOTOR_COUNT];

static uint8_t caracterizando = 0;
static MotorControlData controlCaracterizar;
static uint8_t caracterizar_connectionID;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void SendStatus(void)
{
	uint8_t encoder_data[] = "$SPEEDXTVALOR$";
	uint8_t i;
	unsigned char * ptr;
	AT_CIPSEND_DATA cipsend_data;

	cipsend_data.connectionID = MAX_MULTIPLE_CONNECTIONS;

	for (i = 0; i < MAX_MULTIPLE_CONNECTIONS; i++){
		if (esp8266_getConnectionStatus(i) == CONNECTION_STATUS_OPEN){
			cipsend_data.connectionID = i;
			break;
		}
	}


	if (cipsend_data.connectionID < MAX_MULTIPLE_CONNECTIONS){

		cipsend_data.content = (char *)encoder_data;
		cipsend_data.copyContentToBuffer = AT_CIPSEND_CONTENT_COPYTOBUFFER;

		for (i = 0; i < ENCODER_COUNT; i++)
		{
			ptr = uintToString(i, 1, &(encoder_data[6]));
			*ptr = '0' + SPEED_TYPE_INTERRUPTS;
			ptr = uintToString(encoder_getLastCount(i), 4, &(encoder_data[8]));
			*ptr++ = '$';
			*ptr = '\0';

			cipsend_data.length = AT_CIPSEND_ZERO_TERMINATED_CONTENT; /* Dejo que el comando calcule internamente la longitud */
			esp8266_queueCommand(AT_CIPSENDBUF, AT_TYPE_SET, &cipsend_data);
		}
	}

}


static void ComenzarCaracterizar(PARSER_RESULTS_CARACTERIZAR_T * infoPtr, uint8_t connectionID)
{
	AT_CIPSEND_DATA cipsend_data;

	if (infoPtr->tiempo <= 10000)
	{
		encoder_setTimeElapsedCallback(0);

		caracterizando = 1;

		caracterizar_connectionID = connectionID;

		controlCaracterizar.direction = DIR_FORWARD;
		controlCaracterizar.dutyCycle = 0;
		controlCaracterizar.motorID = infoPtr->idMotor;

		ciaaPWM_updateMotor(controlCaracterizar);

		encoder_beginCount(infoPtr->tiempo);
		encoder_setTimeElapsedCallback(SendDatosCaracterizar);
	}
	else
	{
		cipsend_data.connectionID = connectionID;
		cipsend_data.content = "$ERROR=TIEMPO fuera de rango.$";
		cipsend_data.length = 30;
		cipsend_data.copyContentToBuffer = AT_CIPSEND_CONTENT_DONT_COPY;
		esp8266_queueCommand(AT_CIPSENDBUF, AT_TYPE_SET, &cipsend_data);
	}
}


static void FinalizarCaracterizar(void)
{
	encoder_setTimeElapsedCallback(0);
	caracterizando = 0;
	encoder_beginCount(1000);
	encoder_setTimeElapsedCallback(SendStatus);
}


static void SendDatosCaracterizar(void)
{
	uint8_t buffer[] = "$MOTOR=IDMOTOR,DUTYCYCLE,CANTINTERRUPCIONES$";
	unsigned char * ptr;
	AT_CIPSEND_DATA cipsend_data;

	cipsend_data.connectionID = caracterizar_connectionID;
	cipsend_data.content = (char *)buffer;
	cipsend_data.copyContentToBuffer = AT_CIPSEND_CONTENT_COPYTOBUFFER;

	ptr = uintToString(controlCaracterizar.motorID, 1, &(buffer[7]));
	*ptr++ = ',';
	ptr = uintToString(controlCaracterizar.dutyCycle, 1, ptr);
	*ptr++ = ',';
	ptr = uintToString(encoder_getLastCount(controlCaracterizar.motorID), 1, ptr);
	*ptr++ = '$';
	*ptr = '\0';

	cipsend_data.length = AT_CIPSEND_ZERO_TERMINATED_CONTENT; /* Dejo que el comando calcule internamente la longitud */
	esp8266_queueCommand(AT_CIPSENDBUF, AT_TYPE_SET, &cipsend_data);

	if (controlCaracterizar.dutyCycle < 100)
	{
		controlCaracterizar.dutyCycle++;
		ciaaPWM_updateMotor(controlCaracterizar);
		encoder_resetCount();
	}
	else
	{
		cipsend_data.content = "$FIN_CARACTERIZAR$";
		cipsend_data.length = 18;
		cipsend_data.copyContentToBuffer = AT_CIPSEND_CONTENT_DONT_COPY;

		esp8266_queueCommand(AT_CIPSENDBUF, AT_TYPE_SET, &cipsend_data);

		FinalizarCaracterizar();
	}

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

static void ReceiveData(ReceivedDataInfo info)
{
	uint16_t i;
	AT_CIPSEND_DATA cipsend_data;
	PARSER_RESULTS_DUTYCYCLE_T * dutyCycleResults;

	if (ciaaPOSIX_strncmp("GET / HTTP/1.1", receiveBuffer, 14) == 0)
	{
		/* El mensaje es un HTTP request, entonces envío la respuesta */
		cipsend_data.connectionID = info.connectionID;
		cipsend_data.content = staticResponseHeaders;
		cipsend_data.length = sizeof(staticResponseHeaders);
		cipsend_data.copyContentToBuffer = AT_CIPSEND_CONTENT_DONT_COPY;

		esp8266_queueCommand(AT_CIPSENDBUF, AT_TYPE_SET, &cipsend_data);
	}

	/* Pongo el número de motor en un valor incorrecto. Esto es para evitar
	 * actualizar las salidas PWM si es que no se recibió un comando de
	 * usuario para ello.
	 */
	for (i = 0; i < MOTOR_COUNT; i++){
		lastDutyCycle[i].motorID = MOTOR_COUNT + 1;
	}

	for (i = 0; i < (info.payloadLength > info.bufferLength ? info.bufferLength : info.payloadLength); i++)
	{
		if (!caracterizando)
		{
			if (parser_tryMatch(&parserDutyCycle, receiveBuffer[i]) == STATUS_COMPLETE)
			{
				dutyCycleResults = parser_getResults(&parserDutyCycle);
				if (dutyCycleResults->motorID < MOTOR_COUNT)
				{
					lastDutyCycle[dutyCycleResults->motorID] = *dutyCycleResults;
				}
			}

			if (parser_tryMatch(&parserCaracterizar, receiveBuffer[i]) == STATUS_COMPLETE)
			{
				ComenzarCaracterizar(parser_getResults(&parserCaracterizar), info.connectionID);
			}
		}
		else // Se está caracterizando...
		{
			if (parser_tryMatch(&parserCancelarCaracterizar, receiveBuffer[i]) == STATUS_COMPLETE)
			{
				FinalizarCaracterizar();
			}
		}


	}

	for (i = 0; i < MOTOR_COUNT; i++){
		ciaaPWM_updateMotor(lastDutyCycle[i]);
	}

}


static void WiFiReset(void)
{
	uint8_t i;
	AT_CWSAP_DATA cwsap_data = {"wifi", "12345678", 11, AT_SAP_ENCRYPTION_WPA2_PSK};
	AT_CIPSERVER_DATA cipserver_data = {AT_CIPSERVER_CREATE, 8080};


	esp8266_queueCommand(AT_CWMODE, AT_TYPE_SET, (void*)AT_CWMODE_SOFTAP);
	esp8266_queueCommand(AT_CWSAP_CUR, AT_TYPE_SET, &cwsap_data);
	esp8266_queueCommand(AT_CIPMUX, AT_TYPE_SET, (void*)AT_CIPMUX_MULTIPLE_CONNECTION);
	esp8266_queueCommand(AT_CIPSERVER, AT_TYPE_SET, &cipserver_data);

	/* Se apagan los motores ante un reset inesperado del módulo WiFi */
	for (i = 0; i < MOTOR_COUNT; i++){
		lastDutyCycle[i].motorID = i;
		lastDutyCycle[i].dutyCycle = 0;
		ciaaPWM_updateMotor(lastDutyCycle[i]);
	}

	if (caracterizando)
	{
		FinalizarCaracterizar();
	}
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

	// Frecuencia cambiada en externals/drivers/cortexM4/lpc43xx/inc/clock_18xx_43xx.h
	// y en 				  externals/drivers/cortexM0/lpc43xx/inc/clock_18xx_43xx.h
	// Macro MAX_CLOCK_FREQ
	// Chip_SetupCoreClock(CLKIN_IRC, 102000000, 1);

	/* init CIAA kernel and devices */
	ciaak_start();

	/* Deshabilitando periféricos no utilizados */
	Chip_Clock_Disable(CLK_MX_UART0);
	Chip_Clock_Disable(CLK_APB3_DAC);
	Chip_Clock_Disable(CLK_APB3_ADC0);
	Chip_Clock_Disable(CLK_APB3_ADC1);


	/* open CIAA digital inputs */
	fd_in = ciaaPOSIX_open("/dev/dio/in/0", ciaaPOSIX_O_RDONLY);

	/* open CIAA digital outputs */
	fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);


	/* Starts PWM configuration */
	ciaaPWM_init();

	parser_initModule();
	parser_init(&parserDutyCycle);
	parser_init(&parserCaracterizar);
	parser_init(&parserCancelarCaracterizar);
	literalParser_setStringToMatch(&parserCancelarCaracterizar, "$CANCELAR_CARACTERIZAR$");

	encoder_init();
	encoder_setTimeElapsedCallback(SendStatus);
	encoder_beginCount(1000);

	/* Configuracion de los GPIO de salida. */
	/* Habilitacion de los enable, /reset y chip_enable del puente H. */
	gpio_buffer |= (ENABLE12|ENABLE34|ESP8266_EN|ESP8266_RST);
	ciaaPOSIX_write(fd_out, &gpio_buffer, 2);

	esp8266_init();
	esp8266_setReceiveBuffer(receiveBuffer, RECEIVE_BUFFER_LENGTH);
	esp8266_registerDataReceivedCallback(ReceiveData);
	esp8266_registerResetDetectedCallback(WiFiReset);
	esp8266_queueCommand(AT_RST, AT_TYPE_EXECUTE, 0);

	logger_init();

	ActivateTask(BackgroundTask);

	/* end InitTask */
	TerminateTask();
}


TASK(BackgroundTask)
{
	char buf[32];
	ssize_t ret;

	while(1)
	{

#ifdef LOGGING_ALLOWED
		// wait for any character ...
		ret = logger_read_input(buf, sizeof(buf));

		if(ret > 0)
		{
			// also write them to the other device
			esp8266_writeRawData(buf, ret);
		}
#endif

		esp8266_doWork();

		__WFI();
	}
}



/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

