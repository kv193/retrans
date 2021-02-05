/*
 * afe.h
 *
 *  Created on: Jan 19, 2021
 *      Author: kv193
 */

#ifndef AFE_H_
#define AFE_H_

typedef struct _afe_info_struct {
	int16_t signal_len;
	int16_t out_qty;
	int16_t in_qty;
} afe_config;

int afe_open(char* name);
int afe_close();
int afe_config_get(afe_config *info);

#endif /* AFE_H_ */
