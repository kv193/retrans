/*******************************************************************************
  * @file    verter.c
  * @author  Klochkov V. A.
  * @version V1.0
  * @date    27.12.2020
  * @brief   Main module for verter project.
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <retrans_main.h>
#include "string.h"
#include "cmsis_os.h"
#include "FreeRTOS_CLI.h"
#include "kvreturns.h"
#include "stamen_bsp.h"
#include "config.h"
#include "kvgnrl.h"
//#include "FreeRTOS_IO.h"
#include "afe.h"
#include "stm32f4xx_ll_cortex.h"
#include "adau1372.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_INPUT_LENGTH    128
#define MAX_OUTPUT_LENGTH   128
#define ARROW_UP 0x39
#define PROMPT ">"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
osThreadId ThreadLEDHandle, ThreadLED2Handle, ThreadCLIHandle, ThreadDSPHandle;
int LED1_Delay = 500, LED2_Delay = 100, LED_Delays[LEDn] = {750, 1500};
UART_HandleTypeDef Uart_Std;
static const char *pcWelcomeMessage =
ENDL"FreeRTOS command server.\r\nType 'help' to view a list of registered commands."ENDL PROMPT;

/* Private function prototypes -----------------------------------------------*/
static void CLI_Init();
static void vLedsTask(void const *argument);

/*******************************************************************************
 * @brief Print soft and hard  info.
 ******************************************************************************/
void board_info()
{
	printf("%s-%x.%x for %s-%x.%x, %s, %s"ENDL,
		SOFTWARE_NAME, _MAJ_(SOFTWARE_VERSION), _MIN_(SOFTWARE_VERSION),
		HARDWARE_NAME, _MAJ_(HARDWARE_VERSION), _MIN_(HARDWARE_VERSION),
		__DATE__, __TIME__);
}

/*******************************************************************************
 * @brief Print soft and hard  info.
 ******************************************************************************/
void copyright()
{
	printf("(c) 2021 Techtrans, kv193@yandex.ru"ENDL);
}

/* Private functions ---------------------------------------------------------*/
/******************************************************************************
 * @brief  Toggle LED1 thread 1
 * @param  thread not used
 * @retval None
 ******************************************************************************/
static void vLedsTask(void const *argument)
{
	for (;;) {
		bsp_LED_Toggle(LED1);
		osDelay(LED1_Delay);
	}
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************************************************************************
  * @brief  Implements "hello" command.
  * @param  pcWriteBuffer: output buffer
  * @param  xWriteBufferLen: length of output buffer.
  * @retval pdFALSE that mean end of the output.
  ****************************************************************************/
static BaseType_t prvCopyright(int8_t *pcWriteBuffer,
                                          size_t xWriteBufferLen,
                                          const int8_t *pcCommandString)
{
	board_info();
	copyright();
	pcWriteBuffer[0] = 0;
	return pdFALSE;
}

/*******************************************************************************
 * @brief Custom implementation of fputc.
 *
 * Write a character to USART# and must loop until the end of transmission.
 ******************************************************************************/
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&Uart_Std, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

int __io_getchar(void)
{	uint8_t data;
	HAL_UART_Receive(&Uart_Std, &data, 1, -1);
	return data;
}

/*******************************************************************************
 * @brief DSP Task
 ******************************************************************************/
static void vDspTask(void const *pvPars)
{
	static int cntr = 0;
	(void)cntr;
	if (afe_open("ADAU1372") != KVOK)
		Error_Handler();
	for(;;){
		HAL_Delay(1000);
//		printf("DSP Task iterate: cntr = %d\r\n", cntr++);

	}
}

/*******************************************************************************
 * @brief Console task for CLI.
 ******************************************************************************/
static void vConsoleTask(const void* pvParameters)
{
	void 		*xConsole = (void*)pvParameters;
	int8_t 		cRxedChar, input_index = 0;
	BaseType_t 	xMoreDataToFollow;
	static char   	output_str[MAX_OUTPUT_LENGTH],
			input_str[MAX_INPUT_LENGTH],
			OldCommand[MAX_INPUT_LENGTH];

	bsp_Console_Write(xConsole, pcWelcomeMessage, strlen( pcWelcomeMessage));

	for( ;; ) {
	while (0 == bsp_Console_Read(xConsole, &cRxedChar, sizeof(cRxedChar))) {
	}
	if (cRxedChar == '\n' || cRxedChar == '\r') {
		bsp_Console_Write(xConsole, "\r\n", strlen("\r\n"));
		if (strlen(input_str) != 0) {
			strcpy(OldCommand, input_str);
			do {
				xMoreDataToFollow = FreeRTOS_CLIProcessCommand (
					input_str,   /* The command string.*/
					output_str,  /* The output buffer. */
					MAX_OUTPUT_LENGTH/* The size of the output buffer. */
				);
				bsp_Console_Write(xConsole, output_str, strlen(output_str));
			} while( xMoreDataToFollow != pdFALSE );
			input_index = 0;
			memset( input_str, 0x00, MAX_INPUT_LENGTH );
		}
		bsp_Console_Write(xConsole, PROMPT, strlen(PROMPT));
	} else if (cRxedChar == '\b') {
		if (input_index > 0) {
			input_index--;
			input_str[input_index] = '\0';
			bsp_Console_Write(xConsole, "\b\x20\b", strlen("\b\x20\b"));
		}
	} else if (input_index < MAX_INPUT_LENGTH) {
		input_str[input_index] = cRxedChar;
		input_index++;
		//bsp_Console_Write(xConsole, pcInputString + cInputIndex, sizeof(char));
		if (input_str[0] != 0x1b)
			bsp_Console_Write(xConsole, &cRxedChar, sizeof(char));
	}
	if (strncmp(input_str, "\x1b\x5b\x41", 3) == 0) {
		strcpy(input_str, OldCommand);
		input_index = strlen(input_str);
		bsp_Console_Write(xConsole, input_str,
				strlen(input_str));
	}
	}
}

/******************************************************************************
  * @brief  CLI Initialization.
  * @retval None.
  ****************************************************************************/
static void CLI_Init()
{
	static const CLI_Command_Definition_t xCommandHello = {
		"info",
		"info:"
	    ENDL"    Type hard & soft info"ENDL,
		(pdCOMMAND_LINE_CALLBACK)prvCopyright,
		0
	};

	FreeRTOS_CLIRegisterCommand(&xCommandHello);

}

/*******************************************************************************
 * @brief Configure timer for FreeRTOS statistics.
 * Must be more precisely then FreeRTOS tick.
 ******************************************************************************/
int config_timer_for_stat()
{
	//FUNCTION_EMPTY(__func__);
	return KVOK;
}

int get_run_time_counter_value()
{
	//FUNCTION_EMPTY(__func__);
	return HAL_GetTick();
}

/*******************************************************************************
 * @brief  Main function
 * @param  None
 * @retval None
 ******************************************************************************/
int main(void)
{
	HAL_Init();
	SystemClock_Config(FHCLK / 1000000);
	bsp_LED_Init(LED1);
	bsp_LED_Init(LED2);
	bsp_PB_Init(BUTTON, BUTTON_MODE_GPIO);
	bsp_Console_Init(&Uart_Std);
	printf(ENDL);
	board_info();

	CLI_Init(); // This function (through FreeRTOS_CLIRegisterCommand) disable
	            // interrupts, then enabled by osKernelStart().
	//bsp_DAC_Init();
	init_harmonic();

	osThreadDef(LED, vLedsTask,    osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(CLI, vConsoleTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(DSP, vDspTask,     osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

	ThreadLEDHandle = osThreadCreate(osThread(LED), NULL);
	ThreadCLIHandle = osThreadCreate(osThread(CLI), (void*)&Uart_Std);
	ThreadDSPHandle = osThreadCreate(osThread(DSP), NULL);

	osKernelStart();
	for (;;);
}
