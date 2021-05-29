#include <stddef.h>
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "sl_i2cspm_instances.h"
#include "app.h"

#include "em_cmu.h"
#include "i2c_utils.h"

#if T_TYPE == T_SWITCH
// Defines
#define THUNDIPI_SLAVE_I2C_ADDRESS                     0x30 //E2
#define THUNDIPI_SLAVE_I2C_BUFFER_SIZE                 100

// Buffers
uint8_t thundi_pi_slave_i2c_Buffer[THUNDIPI_SLAVE_I2C_BUFFER_SIZE];
uint8_t thundi_pi_slave_i2c_BufferIndex;

// Transmission flags
#endif


sl_status_t i2c_read_data(sl_i2cspm_t *i2cspm, uint8_t i2c_addr,
		uint8_t reg, uint16_t *data) {
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_read_data[2];
	i2c_read_data[0] = 0;
	i2c_read_data[1] = 0;
	uint8_t i2c_write_data[1];
	i2c_write_data[0] = reg;

	seq.addr = i2c_addr << 1;
	//seq.flags = I2C_FLAG_WRITE_READ;
	seq.flags = I2C_FLAG_WRITE;
	/* Select command to issue */
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = 1;

	ret = I2CSPM_Transfer(i2cspm, &seq);

	if (ret != i2cTransferDone) { // check if i2cTransferDone
		*data = 0;
		return ret;
	}

	seq.addr = i2c_addr << 1;
	//seq.flags = I2C_FLAG_WRITE_READ;
	seq.flags = I2C_FLAG_READ;
	/* Select command to issue */
	/* Select location/length of data to be read */
	seq.buf[0].data = i2c_read_data;
	seq.buf[0].len = 2;

	ret = I2CSPM_Transfer(i2cspm, &seq);

	if (ret != i2cTransferDone) {
		*data = 0;
		return ret;
	}

	//*data = ((uint32_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);
	*data = (i2c_read_data[0] << 8) + (i2c_read_data[1]);

	return SL_STATUS_OK;
}

sl_status_t sl_INA3221_write_data(sl_i2cspm_t *i2cspm,
		uint8_t i2c_addr, uint8_t reg, int16_t value) {
	sl_status_t retval = SL_STATUS_OK;
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_read_data[2];
	uint8_t i2c_write_data[3];

	seq.addr = i2c_addr << 1;
	seq.flags = I2C_FLAG_WRITE;
	/* Select command to issue */
	i2c_write_data[0] = reg;
	i2c_write_data[1] = value >> 8;
	i2c_write_data[2] = value & 0xFF;
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = 3;
	/* Select location/length of data to be read */
	seq.buf[1].data = i2c_read_data;
	seq.buf[1].len = 0;

	ret = I2CSPM_Transfer(i2cspm, &seq);

	if (ret != i2cTransferDone) {
		retval = SL_STATUS_TRANSMIT;
	}
	return retval;
}



#if T_TYPE==T_SWITCH
/** I2C slave ***/
/**************************************************************************//**
 * @brief  Setup I2C
 *****************************************************************************/
void thundipi_slave_initI2C(void) {
	printf("WTF SETUP I2C\r\n\n");


	//set up the registers

	thundi_pi_slave_i2c_Buffer[1]=0xF1;
	thundi_pi_slave_i2c_Buffer[2]=0xE2;


	// Using default settings
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

	// Configure to be addressable as slave
	i2cInit.master = false;

	// Using PA5 (SDA) and PA6 (SCL)
	GPIO_PinModeSet(gpioPortD, 2, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(gpioPortD, 3, gpioModeWiredAndPullUpFilter, 1);

	// Enable pins at location 15 as specified in datasheet
	GPIO->I2CROUTE[1].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE
			& ~_GPIO_I2C_SDAROUTE_MASK)
			| (gpioPortD << _GPIO_I2C_SDAROUTE_PORT_SHIFT
					| (2 << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
	GPIO->I2CROUTE[1].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE
			& ~_GPIO_I2C_SCLROUTE_MASK)
			| (gpioPortD << _GPIO_I2C_SCLROUTE_PORT_SHIFT
					| (3 << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
	GPIO->I2CROUTE[1].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN
			| GPIO_I2C_ROUTEEN_SCLPEN;

	// Initializing the I2C
	I2C_Init(I2C1, &i2cInit);

	// Initializing the buffer index
	thundi_pi_slave_i2c_BufferIndex = 0;

	// Setting up to enable slave mode
	I2C_SlaveAddressSet(I2C1, THUNDIPI_SLAVE_I2C_ADDRESS);
	I2C_SlaveAddressMaskSet(I2C1, 0xFE); // must match exact address

	CMU_ClockEnable(cmuClock_I2C1, true);
	// Configure interrupts
	I2C_IntClear(I2C1, _I2C_IF_MASK);
	I2C_IntEnable(I2C1,
			I2C_IEN_ADDR | I2C_IEN_RXDATAV | I2C_IEN_ACK | I2C_IEN_SSTOP
					| I2C_IEN_BUSERR | I2C_IEN_ARBLOST);
	NVIC_EnableIRQ(I2C1_IRQn);
}

/**************************************************************************//**
 * @brief I2C Interrupt Handler.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void I2C1_IRQHandler(void) {
	uint32_t pending;
	uint32_t rxData;

	pending = I2C1->IF;
	printf("FLAGS %x\r\n\n", pending);
	/* If some sort of fault, abort transfer. */
	if (pending & (I2C_IF_BUSERR | I2C_IF_ARBLOST)) {
		thundi_pi_slave_i2c_rxInProgress = false;
		//GPIO_PinOutSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
	} else {
		if (pending & I2C_IF_ADDR) {
			// Address Match
			// Indicating that reception is started
			rxData = I2C1->RXDATA;
			//printf("<ADDY+RXDATA:%x\r\n\n", rxData);
			I2C1->CMD = I2C_CMD_ACK;
			//printf(">ACK\r\n\n");
			thundi_pi_slave_i2c_rxInProgress = true;

			if (rxData & 0x1) // read bit set
					{
				if (thundi_pi_slave_i2c_BufferIndex < THUNDIPI_SLAVE_I2C_BUFFER_SIZE) {
					// transfer data
					I2C1->TXDATA = thundi_pi_slave_i2c_Buffer[thundi_pi_slave_i2c_BufferIndex++];
				} else {
					// invalid buffer index; transfer data as if slave non-responsive
					I2C1->TXDATA = 0xFF;
				}
				//printf(">TX %x\r\n\n",I2C1->TXDATA);
			} else {
				thundi_pi_slave_i2c_gotTargetAddress = false;
			}

			I2C_IntClear(I2C1, I2C_IF_ADDR | I2C_IF_RXDATAV);

			//GPIO_PinOutSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
		} else if (pending & I2C_IF_RXDATAV) {
			rxData = I2C1->RXDATA;

			//printf("<RXDATA:%x\r\n\n", rxData);
			if (!thundi_pi_slave_i2c_gotTargetAddress) {
				/******************************************************/
				/* Read target address from master.                   */
				/******************************************************/
				// verify that target address is valid
				if (rxData < THUNDIPI_SLAVE_I2C_BUFFER_SIZE) {
					// store target address
					thundi_pi_slave_i2c_BufferIndex = rxData;
					I2C1->CMD = I2C_CMD_ACK;
					//printf(">ACK\r\n\n");
					thundi_pi_slave_i2c_gotTargetAddress = true;
				} else {
					I2C1->CMD = I2C_CMD_NACK;
					//printf(">NACK\r\n\n");
				}
			} else {
				/******************************************************/
				/* Read new data and write to target address          */
				/******************************************************/
				// verify that target address is valid
				if (thundi_pi_slave_i2c_BufferIndex < THUNDIPI_SLAVE_I2C_BUFFER_SIZE) {
					// write new data to target address; auto increment target address
					thundi_pi_slave_i2c_Buffer[thundi_pi_slave_i2c_BufferIndex++] = rxData;
					I2C1->CMD = I2C_CMD_ACK;
					//printf(">ACK\r\n\n");
				} else {
					I2C1->CMD = I2C_CMD_NACK;
					//printf(">NACK\r\n\n");
				}
			}

			I2C_IntClear(I2C1, I2C_IF_RXDATAV);
		}

		if (pending & I2C_IF_ACK) {
			/******************************************************/
			/* Master ACK'ed, so requesting more data.            */
			/******************************************************/
			if (thundi_pi_slave_i2c_BufferIndex < THUNDIPI_SLAVE_I2C_BUFFER_SIZE) {
				// transfer data
				I2C1->TXDATA = thundi_pi_slave_i2c_Buffer[thundi_pi_slave_i2c_BufferIndex++];
			} else {
				// invalid buffer index; transfer data as if slave non-responsive
				I2C1->TXDATA = 0xFF;
			}
			//printf(">TX %x\r\n\n",I2C1->TXDATA);

			I2C_IntClear(I2C1, I2C_IF_ACK);
		}

		if (pending & I2C_IF_SSTOP) {
			// end of transaction
			thundi_pi_slave_i2c_rxInProgress = false;

			I2C_IntClear(I2C1, I2C_IF_SSTOP);
		}
	}
}
#endif










