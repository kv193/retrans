/*
 * afe.h
 *
 *  Created on: Jan 19, 2021
 *      Author: kv193
 */

#ifndef AFE_H_
#define AFE_H_

#include "stdint.h"

#define SIGNAL_SIN     0
#define SIGNAL_RECT    1
#define SIGNAL_TRI     2
#define SIGNALS_SIN
#define SIGNALS_RECT
#define SIGNALS_TRI

typedef struct _afe_info_struct {
	int16_t signal_len;
	int16_t out_qty;
	int16_t in_qty;
} afe_config;

int afe_open(char* name);
int afe_close();
int afe_config_get(afe_config *info);
int afe_config_set(afe_config *info);
int afe_signals_set(int step0, int step1, int mode);

#endif /* AFE_H_ */
