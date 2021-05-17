/*
 * adau1372.h
 *
 *  Created on: Jan 20, 2021
 *      Author: kv193
 */

#ifndef ADAU1372_H_
#define ADAU1372_H_

#define SIGNAL_LEN 	1024
#define IN_QTY		4
#define OUT_QTY		2

#define REG_CLK_CONTROL_ADDR                 0x0
#define REG_PLL_CTRL0_ADDR                   0x1
#define REG_PLL_CTRL1_ADDR                   0x2
#define REG_PLL_CTRL2_ADDR                   0x3
#define REG_PLL_CTRL3_ADDR                   0x4
#define REG_PLL_CTRL4_ADDR                   0x5
#define REG_PLL_CTRL5_ADDR                   0x6
#define REG_CLKOUT_SEL_ADDR                  0x7
#define REG_REGULATOR_ADDR                   0x8
#define REG_DAC_SOURCE_0_1_ADDR              0x11
#define REG_SOUT_SOURCE_0_1_ADDR             0x13
#define REG_SOUT_SOURCE_2_3_ADDR             0x14
#define REG_SOUT_SOURCE_4_5_ADDR             0x15
#define REG_SOUT_SOURCE_6_7_ADDR             0x16
#define REG_ADC_SDATA_CH_ADDR                0x17
#define REG_ASRCO_SOURCE_0_1_ADDR            0x18
#define REG_ASRCO_SOURCE_2_3_ADDR            0x19
#define REG_ASRC_MODE_ADDR                   0x1A
#define REG_ADC_CONTROL0_ADDR                0x1B
#define REG_ADC_CONTROL1_ADDR                0x1C
#define REG_ADC_CONTROL2_ADDR                0x1D
#define REG_ADC_CONTROL3_ADDR                0x1E
#define REG_ADC0_VOLUME_ADDR                 0x1F
#define REG_ADC1_VOLUME_ADDR                 0x20
#define REG_ADC2_VOLUME_ADDR                 0x21
#define REG_ADC3_VOLUME_ADDR                 0x22
#define REG_PGA_CONTROL_0_ADDR               0x23
#define REG_PGA_CONTROL_1_ADDR               0x24
#define REG_PGA_CONTROL_2_ADDR               0x25
#define REG_PGA_CONTROL_3_ADDR               0x26
#define REG_PGA_STEP_CONTROL_ADDR            0x27
#define REG_PGA_10DB_BOOST_ADDR              0x28
#define REG_POP_SUPPRESS_ADDR                0x29
#define REG_TALKTHRU_ADDR                    0x2A
#define REG_TALKTHRU_GAIN0_ADDR              0x2B
#define REG_TALKTHRU_GAIN1_ADDR              0x2C
#define REG_MIC_BIAS_ADDR                    0x2D
#define REG_DAC_CONTROL1_ADDR                0x2E
#define REG_DAC0_VOLUME_ADDR                 0x2F
#define REG_DAC1_VOLUME_ADDR                 0x30
#define REG_OP_STAGE_MUTES_ADDR              0x31
#define REG_SAI_0_ADDR                       0x32
#define REG_SAI_1_ADDR                       0x33
#define REG_SOUT_CONTROL0_ADDR               0x34
#define REG_MODE_MP0_ADDR                    0x38
#define REG_MODE_MP1_ADDR                    0x39
#define REG_MODE_MP4_ADDR                    0x3C
#define REG_MODE_MP5_ADDR                    0x3D
#define REG_MODE_MP6_ADDR                    0x3E
#define REG_PB_VOL_SET_ADDR                  0x3F
#define REG_PB_VOL_CONV_ADDR                 0x40
#define REG_DEBOUNCE_MODE_ADDR               0x41
#define REG_OP_STAGE_CTRL_ADDR               0x43
#define REG_DECIM_PWR_MODES_ADDR             0x44
#define REG_INTERP_PWR_MODES_ADDR            0x45
#define REG_BIAS_CONTROL0_ADDR               0x46
#define REG_BIAS_CONTROL1_ADDR               0x47
#define REG_PAD_CONTROL0_ADDR                0x48
#define REG_PAD_CONTROL1_ADDR                0x49
#define REG_PAD_CONTROL2_ADDR                0x4A
#define REG_PAD_CONTROL3_ADDR                0x4B
#define REG_PAD_CONTROL4_ADDR                0x4C
#define REG_PAD_CONTROL5_ADDR                0x4D
#define REG_ADC_OPER_ADDR                    0x60

#endif /* ADAU1372_H_ */
