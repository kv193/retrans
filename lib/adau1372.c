/*
 * adau1372.c
 *
 *  Created on: Jan 20, 2021
 *      Author: kv193
 */

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "kvreturns.h"
#include "afe.h"
#include "adau1372.h"
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "stm32f4xx_hal.h"
#include "kvgnrl.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define AFE_NAME "ADAU1372"
#define SCL_PIN    GPIO_PIN_6
#define SCL_PORT   GPIOB
#define SDA_PIN    GPIO_PIN_9
#define SDA_PORT   GPIOB

#define ADDR1  1
#define ADDR0  1

#define I2C_CHIP_ADDR 0x7E
#define I2C_DELAY 1

#define SAI1_CLOCK_ENABLE()		__HAL_RCC_SAI1_CLK_ENABLE()
#define SAI1_GPIO_CLK_ENABLE()		__HAL_RCC_GPIOE_CLK_ENABLE()
#define SAI1_AF				GPIO_AF7_SAI1
#define SAI1_AFRL_VALUE			(SAI1_AF << 2 * 4) | \
					(SAI1_AF << 3 * 4) | \
					(SAI1_AF << 4 * 4) | \
					(SAI1_AF << 5 * 4) | \
					(SAI1_AF << 6 * 4)
#define SAI1_GPIO_PORT			GPIOE
#define SAI1_MCLK_PIN			GPIO_PIN_2
#define SAI1_SD_B_PIN			GPIO_PIN_3
#define SAI1_SD_A_PIN			GPIO_PIN_4
#define SAI1_SCK_PIN			GPIO_PIN_5
#define SAI1_FS_PIN			GPIO_PIN_6

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
typedef struct {
	uint8_t addr;
	uint8_t value;
} init_data_t;
init_data_t init_data[] = {
	{ REG_CLK_CONTROL,      CC_MDIV(0) },
	{ REG_PLL_CTRL0,        _MSB_(1000) },
	{ REG_PLL_CTRL1,        _LSB_(1000) },
	{ REG_PLL_CTRL2,        _MSB_(72) },
	{ REG_PLL_CTRL3,        _LSB_(72) },
	{ REG_PLL_CTRL4,        PLL_R(3) | PLL_X(0) | PLL_TYPE(1) },
	{ REG_CLK_CONTROL,      PLL_EN | CC_MDIV(0)},
	{ REG_CLKOUT_SEL,       CLKOUT_FREQ(0b001) },
	{ REG_CLK_CONTROL,      PLL_EN | CLKSRC_PLL | CC_MDIV(0) | MCLK_EN },//PLL_EN | CLKSRC_PLL | CC_MDIV(0) |MCLK_EN },
	{ REG_CLKOUT_SEL,       CLKOUT_FREQ(1) },
	{ REG_MODE_MP6,         MODE_MP6_VAL(0x12) },
	{ REG_MODE_MP0,         MODE_MP1_VAL(0x00) }, // Serial input  0
	{ REG_MODE_MP1,         MODE_MP1_VAL(0x00) }, // Serial output 0
	{ REG_DECIM_PWR_MODES,  DEC_3_EN | DEC_2_EN | DEC_1_EN | DEC_0_EN | SYNC_3_EN | SYNC_2_EN | SYNC_1_EN | SYNC_0_EN },
	{ REG_ADC_CONTROL0,     0b00000000 },
	{ REG_ADC_CONTROL1,     0b00000000 },
	{ REG_ADC_CONTROL2,     ADC_1_EN | ADC_0_EN },
	{ REG_ADC_CONTROL3,     ADC_3_EN | ADC_2_EN },
	{ REG_ASRC_MODE,        ASRC_OUT_EN | ASRC_IN_EN },
	{ REG_ASRCO_SOURCE_0_1, ASRC_OUT_SOURCE1(0b0101) | ASRC_OUT_SOURCE0(0b0100) },
	{ REG_ASRCO_SOURCE_2_3, ASRC_OUT_SOURCE3(0b0111) | ASRC_OUT_SOURCE2(0b0110) }, // ADC3, ADC2
	{ REG_INTERP_PWR_MODES, MOD_1_EN | MOD_0_EN | INT_1_EN | INT_0_EN },
	{ REG_DAC_SOURCE_0_1,   DAC_SOURCE1(0b1101) | DAC_SOURCE0(0b1100) },
	{ REG_DAC_CONTROL1,     DAC1_EN | DAC0_EN },
	{ REG_OP_STAGE_CTRL,    HP_EN_R(0) | HP_EN_L(0) | HP_PDN_R(0b00) | HP_PDN_L(0b00) },
	{ REG_OP_STAGE_MUTES,   HP_MUTE_R(0b00) | HP_MUTE_L(0b00) },
	{ REG_SAI_0,            SDATA_FMT(0b01) | SAI(0b10) | SER_PORT_FS(0b0001) },
	{ REG_SAI_1,            BCLK_TDMC(1) | LR_MODE(0) | BCLKRATE(1) | SAI_MS(0) }
};

/* Private function prototypes -----------------------------------------------*/
int i2c_write(int8_t chip_addr, int16_t reg_addr, int8_t data);
int afe_reg_read(int addr, int *data);
int afe_reg_write(int addr, int data);
int afe_init(int cnt, init_data_t *data);
int afe_register_commands();
int sai_open();

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
 * @brief  Set SCL line to state.
 * @param  state: state to setup SCL line.
 * @retval KV status.
 ******************************************************************************/
int scl_wr(int state)
{
	HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, state);
	kvDelay(I2C_DELAY);
	return KVOK;
}


/*******************************************************************************
 * @brief  Set SCL line to state.
 * @param  state: state to setup SCL line.
 * @retval KV status.
 ******************************************************************************/
int sda_rd()
{
	kvDelay(I2C_DELAY);
	return HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN);
}

/*******************************************************************************
 * @brief  Set SDA line to state.
 * @param  state: state to setup line.
 * @retval KV status.
 ******************************************************************************/
int sda_wr(int state)
{
	HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, state & 1);
	kvDelay(I2C_DELAY);
	return KVOK;
}

static inline int i2c_start()
{
	sda_wr(1);
	scl_wr(1);
	sda_wr(0);
	scl_wr(0);
	return KVOK;
}

static inline int i2c_stop()
{
	sda_wr(1);
	scl_wr(1);
	return KVOK;
}

/*******************************************************************************
 * @brief  Initialize I2C interface.
 * @param  none
 * @retval KV status.
 ******************************************************************************/
int i2c_open()
{
	static GPIO_InitTypeDef  GPIO_InitStruct;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin   = SCL_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SCL_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin   = SDA_PIN;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);

	i2c_stop();
	return KVOK;
}

static inline int i2c_ack_slave()
{
	scl_wr(1);
	if (sda_rd())
		ERROR_HANDLER("Ack slave error.");
	scl_wr(0);
	return KVOK;
}

static inline int i2c_bit_rd(uint8_t *bit)
{
	sda_wr(1);
	scl_wr(1);
	*bit = sda_rd();
	scl_wr(0);
	return KVOK;
}

static inline int i2c_bit_wr(uint8_t bit)
{
	sda_wr(bit);
	scl_wr(1);
	scl_wr(0);
	return KVOK;
}

static inline int16_t i2c_byte_rd(uint8_t *byte)
{
	uint8_t bit;
	*byte = 0;
	i2c_bit_rd(&bit); *byte |= bit << 7;
	i2c_bit_rd(&bit); *byte |= bit << 6;
	i2c_bit_rd(&bit); *byte |= bit << 5;
	i2c_bit_rd(&bit); *byte |= bit << 4;
	i2c_bit_rd(&bit); *byte |= bit << 3;
	i2c_bit_rd(&bit); *byte |= bit << 2;
	i2c_bit_rd(&bit); *byte |= bit << 1;
	i2c_bit_rd(&bit); *byte |= bit << 0;/**//*
	i2c_bit_rd(&bit); *byte = *byte | bit << 7;
	i2c_bit_rd(&bit);
	bit = bit << 6;
	x = x | bit;
	*byte = *byte | bit;
	*byte = x;
	i2c_bit_rd(&bit); *byte |= bit << 5;
	i2c_bit_rd(&bit); *byte |= bit << 4;
	i2c_bit_rd(&bit); *byte |= bit << 3;
	i2c_bit_rd(&bit); *byte |= bit << 2;
	i2c_bit_rd(&bit); *byte |= bit << 1;
	i2c_bit_rd(&bit); *byte |= bit << 0;*/

	return KVOK;
}

static inline int16_t i2c_byte_wr(uint8_t byte)
{
	i2c_bit_wr(byte >> 7);
	i2c_bit_wr(byte >> 6);
	i2c_bit_wr(byte >> 5);
	i2c_bit_wr(byte >> 4);
	i2c_bit_wr(byte >> 3);
	i2c_bit_wr(byte >> 2);
	i2c_bit_wr(byte >> 1);
	i2c_bit_wr(byte >> 0);
	return KVOK;
}

/*******************************************************************************
 * @brief Write data to reg_addr of chip_addr.
 * @param chip_addr:  Chip address.
 * @param reg_addr:   Register address in chip.
 * @param data:       Data byte.
 ******************************************************************************/
int i2c_read(int8_t chip_addr, int16_t reg_addr, uint8_t *data)
{
	i2c_start();
	i2c_byte_wr(chip_addr);		// Chip addr & oper.
	i2c_ack_slave();
	i2c_byte_wr(reg_addr >> 8);	// Subaddr hi.
	i2c_ack_slave();
	i2c_byte_wr(reg_addr);		// Subaddr lo.
	i2c_ack_slave();
	i2c_start();
	i2c_byte_wr(chip_addr | 1);	// Chip addr & oper.
	i2c_ack_slave();
	i2c_byte_rd(data);		// Data byte.
	i2c_stop();
	return KVOK;
	//error
}

/*******************************************************************************
 * @brief Write data to reg_addr of chip_addr.
 * @param chip_addr:  Chip address.
 * @param reg_addr:   Regoster address in chip.
 * @param data:       Data byte.
 ******************************************************************************/
int i2c_write(int8_t chip_addr, int16_t reg_addr, int8_t data)
{
	i2c_start();
	i2c_byte_wr(chip_addr);		// Chip addr & oper.
	i2c_ack_slave();
	i2c_byte_wr(reg_addr >> 8);	// Subaddr hi.
	i2c_ack_slave();
	i2c_byte_wr(reg_addr);		// Subaddr lo.
	i2c_ack_slave();
	i2c_byte_wr(data);		// Data byte.
	i2c_ack_slave();
	i2c_stop();
	return KVOK;
}

/*******************************************************************************
 * @brief  Implements CLI command and output info about AFE.
 * 	   For param and retval see FreeRTOS_CLI.
 ******************************************************************************/
BaseType_t cmd_afe_info(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	sprintf(pcWriteBuffer, "ADAU1372 is low power codec from ADI."
			       " See datasheet on www.analog.com\r\n");
	return pdFALSE;
}

/*******************************************************************************
 * @brief Implements CLI command for reading register of AFE.
 ******************************************************************************/
BaseType_t cmd_afe_reg_read(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	int addr, data;
	BaseType_t x;

	pcWriteBuffer[0] = 0;
	addr = strtol(FreeRTOS_CLIGetParameter(pcCommandString, 1, &x), NULL, 16);
	afe_reg_read(addr, &data);
	printf("0x%02x\r\n",data);
	return pdFALSE;
}

/*******************************************************************************
 * @brief Implements CLI command for writing register of AFE.
 ******************************************************************************/
BaseType_t cmd_afe_reg_write(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	int addr, data;
	BaseType_t x;

	(void)addr;
	(void)data;

	pcWriteBuffer[0] = 0;
	addr = strtol(FreeRTOS_CLIGetParameter(pcCommandString, 1, &x), NULL, 16);
	data = strtol(FreeRTOS_CLIGetParameter(pcCommandString, 2, &x), NULL, 16);
	afe_reg_write(addr, data);
	return pdFALSE;
}


/*******************************************************************************
 * @brief  Open AFE (analog front end), initialize it.
 * @param  name - afe name. As generally it is chip name.
 * @retval KV status.
 ******************************************************************************/
int afe_open(char* name)
{
	if (strcmp(name, AFE_NAME) != 0) {
		printf("Error: AFE name %s is not compatible!\r\n", name);
		Error_Handler();
		return -KVERROR;
	}

	i2c_open();
	sai_open(); 	//(void) init_data;
	afe_init(ARRAY_SIZE(init_data), init_data);
	afe_register_commands();

	return KVOK;
}

/*******************************************************************************
 * @brief  Close opened afe.
 * @retval KV status.
 ******************************************************************************/
int afe_close()
{
	FUNCTION_EMPTY(__func__);
	return KVOK;
}

/*******************************************************************************
 * @brief  Return afe information in structure info.
 * @param  info - structure for information about afe.
 * @retval KV status.
 ******************************************************************************/
int afe_cfg_read(afe_config *info)
{
	info->signal_len = SIGNAL_LEN;
	info->in_qty = IN_QTY;
	info->out_qty = OUT_QTY;
	return KVOK;
}

/*******************************************************************************
 * @brief Read register in AFE.
 ******************************************************************************/
int afe_reg_read(int addr, int *data)
{
	i2c_read(I2C_CHIP_ADDR, addr, (uint8_t*)data);
	return KVOK;
}

/*******************************************************************************
 * @brief Write register in AFE.
 ******************************************************************************/
int afe_reg_write(int addr, int data)
{
	i2c_write(I2C_CHIP_ADDR, addr, (int8_t)data);
	return KVOK;
}

/*******************************************************************************
 * @brief Initialize AFE.
 * @param cnt  - qty of data pair to write.
 * @param data - data buffer with pairs reg_addr & value.
 * @return KV status.
 ******************************************************************************/
int afe_init(int cnt, init_data_t *data)
{
	for (int i = 0; i < cnt; i++) {
		afe_reg_write(data[i].addr, data[i].value);
	}
	return KVOK;
}

/*******************************************************************************
 * @brief  Register afe commands for CLI.
 * @param  None.
 * @retval KV status.
 ******************************************************************************/
int afe_register_commands()
{

	static const CLI_Command_Definition_t command[] = {
		{
			"afe-info",
			"afe-info:"ENDL
			"    Type info about afe chip."ENDL,
			cmd_afe_info,
			0
		},{
			"afe-reg-rd",
			"afe-reg-rd addr:"ENDL
			"    Read data from register at addr (hex)."ENDL,
			cmd_afe_reg_read,
			1
		},{
			"afe-reg-wr",
			"afe-reg-wr addr data:"ENDL
			"    Write data to register at addr (hex)."ENDL,
			cmd_afe_reg_write,
			2
		}
	};
	for (int i = 0; i < ARRAY_SIZE(command); i++) {
		FreeRTOS_CLIRegisterCommand(&command[i]);
	}
//	FreeRTOS_CLIRegisterCommand(&command[0]);
//	FreeRTOS_CLIRegisterCommand(&command[1]);
//	FreeRTOS_CLIRegisterCommand(&command[2]);

	return KVOK;
}

/*******************************************************************************
 * @brief  Register afe commands for CLI.
 * @param  None.
 * @retval KV status.
 ******************************************************************************/
int sai_open(void)
{

	//SAI_HandleTypeDef _SAIHandle, *SAI_Handle = &_SAIHandle;
	//SAI_Handle->Instance =
	//HAL_SAI_Init
	SAI1_GPIO_CLK_ENABLE();
	SAI1_CLOCK_ENABLE();
	GPIOE->MODER  |= 0b1010101010 << 2 * 2;		// Alt fun for PE2,3,4,5,6
	GPIOE->AFR[0] |= SAI1_AFRL_VALUE;
	SAI1_Block_A->CR1 = 0b00000000000000000000000010000001; // Block_A - master receiver
	SAI1_Block_B->CR1 = 0b00000000000000000000010010000010; /* Block_B - slave transmitter
	                              ||||| ||  ||||||||| ||++-- MODE
	                              ||||| ||  ||||||||| ++---- PRTCFG
	                              ||||| ||  ||||||+++------- DS (16 bit)
	                              ||||| ||  |||||+---------- LSBFIRST
	                              ||||| ||  ||||+----------- CKSTR
	                              ||||| ||  ||++------------ SYNCEN (A - async, B - sync)
	                              ||||| ||  |+-------------- MONO
	                              ||||| ||  +--------------- OUTDRIV
	                              ||||| |+------------------ SAIxEN
	                              ||||| +------------------- DMAEN
	                              ||||+--------------------- NODIV
	                              ++++---------------------- MCKDIV = 0 */
	SAI1_Block_A->FRCR = 0b00000000000000000001111100111111;
	SAI1_Block_B->FRCR = 0b00000000000000000001111100111111;  /*
                                            ||| |||||||++++++++-- FRL
                                            ||| +++++++---------- FSALL
                                            ||+------------------ FSDEF
                                            |+------------------- FSPOL
                                            +-------------------- FSOFF */
	SAI1_Block_A->SLOTR = 0b00000000000000110000000110000000;
	SAI1_Block_B->SLOTR = 0b00000000000000110000000110000000;  /*
	                        ||||||||||||||||    |||||| +++++-- FBOFF
	                        ||||||||||||||||    ||||++-------- SLOTSZ
	                        ||||||||||||||||    ++++---------- NBSLOT
	                        ++++++++++++++++------------------ SLOTEN  */
	SAI1_Block_B->CR1 |= SAI_xCR1_SAIEN; // Slave enable first before master.
	SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
	SAI1_Block_B->DR = 0x5555;

	return KVOK;
}
