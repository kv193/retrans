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

#define REG_CLK_CONTROL                 0x0
#define REG_PLL_CTRL0                   0x1
#define REG_PLL_CTRL1                   0x2
#define REG_PLL_CTRL2                   0x3
#define REG_PLL_CTRL3                   0x4
#define REG_PLL_CTRL4                   0x5
#define REG_PLL_CTRL5                   0x6
#define REG_CLKOUT_SEL                  0x7
#define REG_REGULATOR                   0x8
#define REG_DAC_SOURCE_0_1              0x11
#define REG_SOUT_SOURCE_0_1             0x13
#define REG_SOUT_SOURCE_2_3             0x14
#define REG_SOUT_SOURCE_4_5             0x15
#define REG_SOUT_SOURCE_6_7             0x16
#define REG_ADC_SDATA_CH                0x17
#define REG_ASRCO_SOURCE_0_1            0x18
#define REG_ASRCO_SOURCE_2_3            0x19
#define REG_ASRC_MODE                   0x1A
#define REG_ADC_CONTROL0                0x1B
#define REG_ADC_CONTROL1                0x1C
#define REG_ADC_CONTROL2                0x1D
#define REG_ADC_CONTROL3                0x1E
#define REG_ADC0_VOLUME                 0x1F
#define REG_ADC1_VOLUME                 0x20
#define REG_ADC2_VOLUME                 0x21
#define REG_ADC3_VOLUME                 0x22
#define REG_PGA_CONTROL_0               0x23
#define REG_PGA_CONTROL_1               0x24
#define REG_PGA_CONTROL_2               0x25
#define REG_PGA_CONTROL_3               0x26
#define REG_PGA_STEP_CONTROL            0x27
#define REG_PGA_10DB_BOOST              0x28
#define REG_POP_SUPPRESS                0x29
#define REG_TALKTHRU                    0x2A
#define REG_TALKTHRU_GAIN0              0x2B
#define REG_TALKTHRU_GAIN1              0x2C
#define REG_MIC_BIAS                    0x2D
#define REG_DAC_CONTROL1                0x2E
#define REG_DAC0_VOLUME                 0x2F
#define REG_DAC1_VOLUME                 0x30
#define REG_OP_STAGE_MUTES              0x31
#define REG_SAI_0                       0x32
#define REG_SAI_1                       0x33
#define REG_SOUT_CONTROL0               0x34
#define REG_MODE_MP0                    0x38
#define REG_MODE_MP1                    0x39
#define REG_MODE_MP4                    0x3C
#define REG_MODE_MP5                    0x3D
#define REG_MODE_MP6                    0x3E
#define REG_PB_VOL_SET                  0x3F
#define REG_PB_VOL_CONV                 0x40
#define REG_DEBOUNCE_MODE               0x41
#define REG_OP_STAGE_CTRL               0x43
#define REG_DECIM_PWR_MODES             0x44
#define REG_INTERP_PWR_MODES            0x45
#define REG_BIAS_CONTROL0               0x46
#define REG_BIAS_CONTROL1               0x47
#define REG_PAD_CONTROL0                0x48
#define REG_PAD_CONTROL1                0x49
#define REG_PAD_CONTROL2                0x4A
#define REG_PAD_CONTROL3                0x4B
#define REG_PAD_CONTROL4                0x4C
#define REG_PAD_CONTROL5                0x4D
#define REG_ADC_OPER                    0x60

#define BIT(pos, val)           (val << pos)
#define PLL_EN	                BIT(7, 1)
#define PLL_DIS	                BIT(7, 0)
#define SPK_FLT_DIS             BIT(5, 1)
#define XTAL_DIS                BIT(4, 1)
#define CLKSRC                  BIT(3, 1)
#define CC_MDIV(val)            BIT(1, val) // 0 - div 2, 1 - div 1.
#define MCLK_EN                 BIT(0, 1)
#define PLL_TYPE(type)	        BIT(0, type)  // type = {0 - integer, 1 - fract}
#define PLL_R(R)                BIT(3, R)     // R = {2, ..., 8}
#define PLL_X(X)                BIT(1, X)     // X = {0, ..., 3}, PLL div ratio = X + 1
#define CLKOUT_FREQ(val)        BIT(0, val)   // val = {0 - MCLK*2, 1 - MCLK, 2, 3, 4, 7}
#define CLKSRC_PLL              BIT(3, 1)
#define CLKSRC_EXT              BIT(3, 0)
#define MODE_MP0_VAL(val)       BIT(0, val)
#define MODE_MP1_VAL(val)       BIT(0, val)
#define MODE_MP6_VAL(val)       BIT(0, val)
#define DEC_3_EN                BIT(7, 1)
#define DEC_2_EN                BIT(6, 1)
#define DEC_1_EN                BIT(5, 1)
#define DEC_0_EN                BIT(4, 1)
#define SYNC_3_EN               BIT(3, 1)
#define SYNC_2_EN               BIT(2, 1)
#define SYNC_1_EN               BIT(1, 1)
#define SYNC_0_EN               BIT(0, 1)
#define ADC_1_EN                BIT(1, 1)
#define ADC_0_EN                BIT(0, 1)
#define ADC_3_EN                BIT(1, 1)
#define ADC_2_EN                BIT(0, 1)
#define ASRC_OUT_EN             BIT(1, 1)
#define ASRC_IN_EN              BIT(0, 1)
#define ASRC_OUT_SOURCE1(src)   BIT(4, src)
#define ASRC_OUT_SOURCE0(src)   BIT(0, src)
#define ASRC_OUT_SOURCE3(src)   BIT(4, src)
#define ASRC_OUT_SOURCE2(src)   BIT(0, src)
#define MOD_1_EN                BIT(3, 1)
#define MOD_0_EN                BIT(2, 1)
#define INT_1_EN                BIT(1, 1)
#define INT_0_EN                BIT(0, 1)
#define DAC_SOURCE1(src)        BIT(4, src)
#define DAC_SOURCE0(src)        BIT(0, src)
#define DAC1_EN                 BIT(1, 1)
#define DAC0_EN                 BIT(0, 1)
#define HP_EN_R(x)              BIT(5, x)
#define HP_EN_L(x)              BIT(4, x)
#define HP_PDN_R(x)             BIT(2, x)
#define HP_PDN_L(x)             BIT(0, x)
#define HP_MUTE_R(x)            BIT(2, x)
#define HP_MUTE_L(x)            BIT(0, x)
#define SDATA_FMT(x)            BIT(6, x)
#define SAI(x)                  BIT(4, x)
#define SER_PORT_FS(x)          BIT(0, x)
#define BCLK_TDMC(x)            BIT(6, x)
#define LR_MODE(x)              BIT(5, x)
#define BCLKRATE(x)             BIT(2, x)
#define SAI_MS(x)               BIT(0, x)

int sai_open(void);

#endif /* ADAU1372_H_ */
