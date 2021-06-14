/*
 * neva.c
 *
 *  Created on: Jun 10, 2021
 *      Author: kv193
 */

/* Includes ------------------------------------------------------------------*/
#include "kvgnrl.h"
#include "kvreturns.h"
#include "neva.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int16_t input_signal_cp[FILTERCPLEN];

int16_t    cos500[FILTERLPLEN] = {
			#include "ini/cos500h.ini"
        }, cos600[FILTERLPLEN] = {
			#include "ini/cos600h.ini"
	}, cos700[FILTERLPLEN] = {
			#include "ini/cos700h.ini"
	}, cos800[FILTERLPLEN] = {
			#include "ini/cos800h.ini"
	}, cos1025[FILTERCPLEN] = {
			#include "ini/cos1025h.ini"
	}, cos1225[FILTERCPLEN] = {
			#include "ini/cos1225h.ini"
	}, cos1625[FILTERCPLEN] = {
			#include "ini/cos1625h.ini"
	}, cos1825[FILTERCPLEN] = {
			#include "ini/cos1825h.ini"
	}, cos2225[FILTERCPLEN] = {
			#include "ini/cos2225h.ini"
	}, cos2425[FILTERCPLEN] = {
			#include "ini/cos2425h.ini"
	}, cos2825[FILTERCPLEN] = {
			#include "ini/cos2825h.ini"
	}, cos3025[FILTERCPLEN] = {
			#include "ini/cos3025h.ini"
	}, sin500[FILTERLPLEN] = {
			#include "ini/sin500h.ini"
	}, sin600[FILTERLPLEN] = {
			#include "ini/sin600h.ini"
	}, sin700[FILTERLPLEN] = {
			#include "ini/sin700h.ini"
	}, sin800[FILTERLPLEN] = {
			#include "ini/sin800h.ini"
	}, sin1025[FILTERCPLEN] = {
			#include "ini/sin1025h.ini"
	}, sin1225[FILTERCPLEN] = {
			#include "ini/sin1225h.ini"
	}, sin1625[FILTERCPLEN] = {
			#include "ini/sin1625h.ini"
	}, sin1825[FILTERCPLEN] = {
			#include "ini/sin1825h.ini"
	}, sin2225[FILTERCPLEN] = {
			#include "ini/sin2225h.ini"
	}, sin2425[FILTERCPLEN] = {
			#include "ini/sin2425h.ini"
	}, sin2825[FILTERCPLEN] = {
			#include "ini/sin2825h.ini"
	}, sin3025[FILTERCPLEN] = {
			#include "ini/sin3025h.ini"
	};

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

int generator_tu(void)
{
	int ret;
	static int _flag = 1;
	if (_flag) {
		ret = print_f("%s: This function is not implemented."ENDL, __func__);
		if ( ret > 0) {
			_flag = 0;
		} else {
			fflush(stdout);
		}
	}
	//FUNCTION_EMPTY_ONCE(__func__);
	return -KVERROR;
}
int generator_ts(void)
{
	FUNCTION_EMPTY_ONCE(__func__);
	return -KVERROR;
}

int filters_tu(void)
{
	FUNCTION_EMPTY_ONCE(__func__);
	return -KVERROR;
}

int filters_ts(void)
{
	FUNCTION_EMPTY_ONCE(__func__);
	return -KVERROR;
}

int decoder_tu(void)
{
	FUNCTION_EMPTY_ONCE(__func__);
	return -KVERROR;
}

int decoders_ts(void)
{
	FUNCTION_EMPTY_ONCE(__func__);
	return -KVERROR;
}

/* Private functions ---------------------------------------------------------*/

