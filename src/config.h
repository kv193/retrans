/**
  * @author  Klochkov V. A.
  ******************************************************************************
  * @file    sRobots/src/freertos/inc/config.h
  * @version V1.0
  * @date    28.12.2020
  * @brief   This file contains configuration symbols for project.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 Klochkov V. A.</center></h2>
  *
  ******************************************************************************
  */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "hwswcv.h"

#define BOARD_TYPE BOARD_STAMEN
#define CONFIG_STAMEN_H_VER 0x0200

#define BOARD_NUCLEO_F411RE	1
#define BOARD_STAMEN		2

#define HARDWARE_CLASS   HARDWARE_CLASS_STAMEN
#define HARDWARE_NAME "Stamen"
#define HARDWARE_VERSION 0x0120
#define SOFTWARE_CLASS   SOFTWARE_CLASS_RETRANS
#define SOFTWARE_NAME "Retrans"
#define SOFTWARE_VERSION 0x0100

#define CONFIG_SYS_ARCH 	CORTEX_M4
#define CONFIG_SYS_CPU  	STM32F413
#define CONFIG_SYS_OS 		FREERTOS
#define CONFIG_SYS_BOARD 	STAMEN
#define CONFIG_SYS_PLD 		none

#define FQUARTZ			16000000
#define FHCLK			96000000
#define CK_HSE			FQUARTZ

#define SIGNAL_LEN		1024

#endif /* __CONFIG_H */

