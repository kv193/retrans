/*
 * neva.h
 *
 *  Created on: Jun 10, 2021
 *      Author: kv193
 */

#ifndef NEVA_H_
#define NEVA_H_


/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define FSAMPLINGIN  8000   // f_sampling at inputs (ADC).
#define FSAMPLINGCP  1000   // f_sampling at cp filters outputs.
#define FSAMPLINGLP  125    // f_sampling ar lp filters outputs.
#define FILTERCPLEN  (8  * FSAMPLINGIN / 1000)
#define FILTERLPLEN  (24 * FSAMPLINGIN / 1000)
#define MAGCPLEN     (FSAMPLINGCP * 224 / 1000)    // 224 ms.
#define MAGLPLEN     (FSAMPLINGLP *  2000 / 1000)  // 2s.

/* Exported functions --------------------------------------------------------*/
int generator_tu(void);
int generator_ts(void);
int filters_tu(void);
int filters_ts(void);
int decoder_tu(void);
int decoders_ts(void);
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#endif /* NEVA_H_ */
