/*
 * adau1372.c
 *
 *  Created on: Jan 20, 2021
 *      Author: kv193
 */

/* Includes ------------------------------------------------------------------*/
#include "string.h"
#include "kvreturns.h"
#include "kvgnrl.h"
#include "afe.h"
#include "adau1372.h"
#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "stm32f4xx_hal.h"

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
/* Private function prototypes -----------------------------------------------*/
int i2c_write(int8_t chip_addr, int16_t reg_addr, int8_t data);
int afe_reg_read(int addr, int *data);
int afe_reg_write(int addr, int data);

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
/*	scl_wr(0);
	scl_wr(1);
	scl_wr(0);
	scl_wr(1);
	scl_wr(0);
	scl_wr(1);
	scl_wr(0);
	scl_wr(1);
	scl_wr(0);
	scl_wr(1);
	scl_wr(0);
	scl_wr(1);
	scl_wr(0);
	scl_wr(1);
	scl_wr(0);
	scl_wr(1);
*/
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

	i2c_open();

	return KVOK;
}

/*******************************************************************************
 * @brief  Close opened afe.
 * @retval KV status.
 ******************************************************************************/
int afe_close()
{
	function_empty;
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

