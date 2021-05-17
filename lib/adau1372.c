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
/*#define BIT(bit, data) ((data >> bit) & 1)
#define I2C_START() \
	sda_wr(1); \
	scl_wr(1); \
	sda_wr(0); \
	scl_wr(0);
#define I2C_STOP() sda_wr(1); scl_wr(1);
#define I2C_ACK_SLAVE() \
		scl_wr(1); \
		if (sda_rd()) \
			Error_Handler(); \
		scl_wr(0);
#define I2C_BIT_WR(bit) sda_wr(bit); scl_wr(1); scl_wr(0);
#define I2C_BIT_RD(bit)	sda_wr(1); scl_wr(1); bit = sda_rd(); scl_wr(0);
#define I2C_BYTE_RD(byte) { int bit;\
		I2C_BIT_RD(bit); *byte  = bit << 7;\
		I2C_BIT_RD(bit); *byte |= bit << 6;\
		I2C_BIT_RD(bit); *byte |= bit << 5;\
		I2C_BIT_RD(bit); *byte |= bit << 4;\
		I2C_BIT_RD(bit); *byte |= bit << 3;\
		I2C_BIT_RD(bit); *byte |= bit << 2;\
		I2C_BIT_RD(bit); *byte |= bit << 1;\
		I2C_BIT_RD(bit); *byte |= bit << 0;\
		}
#define I2C_BYTE_WR(byte) \
		I2C_BIT_WR(BIT(7, byte)); \
		I2C_BIT_WR(BIT(6, byte)); \
		I2C_BIT_WR(BIT(5, byte)); \
		I2C_BIT_WR(BIT(4, byte)); \
		I2C_BIT_WR(BIT(3, byte)); \
		I2C_BIT_WR(BIT(2, byte)); \
		I2C_BIT_WR(BIT(1, byte)); \
		I2C_BIT_WR(BIT(0, byte));
*/
/* Private variables ---------------------------------------------------------*/
static uint8_t init_data[] = {
		REG_CLK_CONTROL_ADDR,      0x00,
		REG_PLL_CTRL0_ADDR,        0x03,
		REG_PLL_CTRL1_ADDR,        0xe8,
		REG_PLL_CTRL2_ADDR,        0x00,
		REG_PLL_CTRL3_ADDR,        0x48,
		REG_PLL_CTRL4_ADDR,        0x19,
		REG_CLK_CONTROL_ADDR,      0x80,
		REG_CLKOUT_SEL_ADDR,       0x01, // Wait for PLL lock instead read #6 reg.
		REG_CLK_CONTROL_ADDR,      0x89,
		REG_CLKOUT_SEL_ADDR,       0x01,

		REG_MODE_MP6_ADDR,         0x12, // Clock output
		REG_MODE_MP0_ADDR,         0x00, // Serial input  0
		REG_MODE_MP1_ADDR,         0x00, // Serial output 0
		REG_DECIM_PWR_MODES_ADDR,  0b11111111, //0xff,
		REG_ADC_CONTROL0_ADDR,     0b00000000,
		REG_ADC_CONTROL1_ADDR,     0b00000000,
		REG_ADC_CONTROL2_ADDR,     0b00000011,
		REG_ADC_CONTROL3_ADDR,     0b00000011,
		REG_ASRC_MODE_ADDR,        0b00000011, // 0x03,
		REG_ASRCO_SOURCE_0_1_ADDR, 0b01010100, // ADC1, ADC0
		REG_ASRCO_SOURCE_2_3_ADDR, 0b01110110, // ADC3, ADC2

		REG_INTERP_PWR_MODES_ADDR, 0x0f,
		REG_DAC_SOURCE_0_1_ADDR,   0xdc,
		REG_DAC_CONTROL1_ADDR,     0x03,
		REG_OP_STAGE_CTRL_ADDR,    0x00,
		REG_OP_STAGE_MUTES_ADDR,   0x00,
		REG_SAI_0_ADDR,            0x61,
		REG_SAI_1_ADDR,            0b01100101, /*//0x65,
                                             |||||||+-- SAI_MS (master)
			                     ||||||+--- BCLKEDGE
			                     |||||+---- BCLKRATE
			                     ||||+----- SAI_MSB
			                     |||+------ LR_POL
			                     ||+------- LR_MODE
		             	             |+-------- BCLK_TDMC
		             	             +--------- TDM_TS */
};

/* Private function prototypes -----------------------------------------------*/
int i2c_write(int8_t chip_addr, int16_t reg_addr, int8_t data);
int afe_reg_read(int addr, int *data);
int afe_reg_write(int addr, int data);
int afe_init(int cnt, uint8_t *data);
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
	afe_init(ARRAY_SIZE(init_data) / 2, init_data);
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
int afe_init(int cnt, uint8_t *data)
{
	for (int i = 0; i < cnt; i++) {
		afe_reg_write(data[i * 2], data[i * 2 + 1]);
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

	static const CLI_Command_Definition_t
	afe_info_def = {
		"afe-info",
		"afe-info:\r\n Type info about afe chip.\r\n",
		cmd_afe_info,
		0
	},
	afe_reg_read_def = {
		"afe-reg-rd",
		"afe-reg-rd addr:\r\n Read data from register at addr (hex).\r\n",
		cmd_afe_reg_read,
		1
	},
	afe_reg_write_def = {
		"afe-reg-wr",
		"afe-reg-wr addr data:\r\n Write data to register at addr (hex).\r\n",
		cmd_afe_reg_write,
		2
	};

	FreeRTOS_CLIRegisterCommand(&afe_info_def);
	FreeRTOS_CLIRegisterCommand(&afe_reg_read_def);
	FreeRTOS_CLIRegisterCommand(&afe_reg_write_def);

	return KVOK;
}

/*******************************************************************************
 * @brief  Register afe commands for CLI.
 * @param  None.
 * @retval KV status.
 ******************************************************************************/
int sai_open()
{

	//SAI_HandleTypeDef _SAIHandle, *SAI_Handle = &_SAIHandle;
	//SAI_Handle->Instance =
	//HAL_SAI_Init
	SAI1_GPIO_CLK_ENABLE();
	SAI1_CLOCK_ENABLE();
	GPIOE->MODER  |= 0b1010101010 << 2 * 2;		// Alt fun for PE2,3,4,5,6
	GPIOE->AFR[0] |= SAI1_AFRL_VALUE;
	SAI1_Block_A->CR1 = 0b00000000000000000000000001000011; // Block_A - rx
	SAI1_Block_B->CR1 = 0b00000000000000000000010001000010; /* Block_B - tx
	                      ||||||||||||| ||  ||||||||||||++-- MODE
	                      ||||||||||||| ||  ||||||||||++---- PRTCFG
	                      ||||||||||||| ||  |||||||+++------ DS (16 bit)
	                      ||||||||||||| ||  ||||||+--------- LSBFIRST
	                      ||||||||||||| ||  |||||+---------- CKSTR
	                      ||||||||||||| ||  ||++------------ SYNCEN (async)
	                      ||||||||||||| ||  |+-------------- MONO
	                      ||||||||||||| ||  +--------------- OUTDRIV
	                      ||||||||||||| |+------------------ SAIxEN
	                      ||||||||||||| +------------------- DMAEN
	                      ||||||||||||+--------------------- NODIV
	                      ||||||||++++---------------------- MCKDIV
	                      ++++++++-------------------------- Reserved (as well missed bits) */
	SAI1_Block_A->FRCR = 0b00000000000001000000000000111111;
	SAI1_Block_B->FRCR = 0b00000000000001000000000000111111;/*
                               |||||||||||||||| |||||||++++++++-- FRL
                               |||||||||||||||| +++++++---------- FSALL
                               |||||||||||||||+------------------ FSDEF
                               ||||||||||||||+------------------- FSPOL
                               |||||||||||||+-------------------- FSOFF
                               +++++++++++++--------------------- Reserved */
	SAI1_Block_A->SLOTR = 0b00000000000011110000001101000000;
	SAI1_Block_B->SLOTR = 0b00000000000000110000001101000000;/*
	                        ||||||||||||||||    |||||| +++++-- FBOFF
	                        ||||||||||||||||    ||||++-------- SLOTSZ
	                        ||||||||||||||||    ++++---------- NBSLOT
	                        ++++++++++++++++------------------ SLOTEN  */
	SAI1_Block_A->CR1 |= SAI_xCR1_SAIEN;
	SAI1_Block_B->CR1 |= SAI_xCR1_SAIEN;
	SAI1_Block_B->DR = 0x5555;

	return KVOK;
}
