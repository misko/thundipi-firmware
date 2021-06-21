
#include <stddef.h>
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "sl_i2cspm_instances.h"
#include "THUNDIPII2C.h"
#include "i2c_utils.h"
#include "em_cmu.h"
#include <machine/endian.h>

/**************************************************************************/
/*!
    @file     THUNDIPII2C.c
*/
/**************************************************************************/

#if T_TYPE == T_SWITCH
// Defines

#define THUNDIPI_SLAVE_I2C_BUFFER_SIZE                 100

// Buffers
volatile uint8_t thundi_pi_slave_i2c_Buffer[THUNDIPI_SLAVE_I2C_BUFFER_SIZE];
volatile uint8_t thundi_pi_slave_i2c_BufferIndex;

// Transmission flags


uint32_t get_their_key() {
	uint32_t their_key=0;
	memcpy(&their_key,thundi_pi_slave_i2c_Buffer+THUNDIPI_SLAVE_I2C_THEIR_PASSKEY_OFFSET,sizeof(uint32_t));
	return their_key;
}


/** I2C slave ***/
/**************************************************************************//**
 * @brief  Setup I2C
 *****************************************************************************/
void thundipi_slave_initI2C(void) {
	printf("WTF SETUP I2C\r\n\n");


	//set up the registers
	memset(thundi_pi_slave_i2c_Buffer,0,THUNDIPI_SLAVE_I2C_BUFFER_SIZE);
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
	//printf("FLAGS %x\r\n\n", pending);
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
				//printf(">TX\r\n\n"); // %x %d\r\n\n",I2C1->TXDATA,thundi_pi_slave_i2c_BufferIndex);
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
					//printf(">WROTE DATA %d %d %d\r\n\n",thundi_pi_slave_i2c_Buffer,thundi_pi_slave_i2c_BufferIndex,rxData);
					thundi_pi_slave_i2c_Buffer[thundi_pi_slave_i2c_BufferIndex++] = rxData;
					I2C1->CMD = I2C_CMD_ACK;
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
			//printf(">TX\r\n\n"); // %x %d\r\n\n",I2C1->TXDATA,thundi_pi_slave_i2c_BufferIndex);

			I2C_IntClear(I2C1, I2C_IF_ACK);
		}

		if (pending & I2C_IF_SSTOP) {
			// end of transaction
			thundi_pi_slave_i2c_rxInProgress = false;

			I2C_IntClear(I2C1, I2C_IF_SSTOP);
		}
	}
}


void thundipi_write_passkey_to_mem(uint32_t passkey) {
	uint32_t passkey_network_order=__htonl(passkey);
	memcpy(thundi_pi_slave_i2c_Buffer+THUNDIPI_SLAVE_I2C_OUR_PASSKEY_OFFSET,&passkey_network_order,sizeof(uint32_t));
}

uint8_t * thundipi_read_master_addr() {
	return thundi_pi_slave_i2c_Buffer+THUNDIPI_SLAVE_I2C_MASTER_BLE_ADDR_OFFSET;
}

uint8_t thundipi_read_master_indicator() {
	return thundi_pi_slave_i2c_Buffer[THUNDIPI_SLAVE_I2C_MASTER_BLE_INDICATOR_OFFSET];
}

void thundipi_write_slave_addr(uint8_t * address) {
	memcpy(thundi_pi_slave_i2c_Buffer+THUNDIPI_SLAVE_I2C_SLAVE_BLE_ADDR_OFFSET,address,6);
}


#endif


uint16_t thundipi_read_id(struct I2C_THUNDIPII2C * sensor) {
  uint16_t value;

  sl_status_t ret=i2c_read_data_uint16(sensor->i2cspm, sensor->m_i2cAddress, THUNDIPI_SLAVE_I2C_ID_OFFSET, &value);

  if (ret != SL_STATUS_OK) {
	  return 0;
  }
  return value;
}

void thundipi_read_slave_address(struct I2C_THUNDIPII2C * sensor, uint8_t * address) {
  sl_status_t sc =  i2c_read_data(sensor->i2cspm, sensor->m_i2cAddress, THUNDIPI_SLAVE_I2C_SLAVE_BLE_ADDR_OFFSET, 6, (uint8_t*)address);
  return;
}

void thundipi_write_passkey(struct I2C_THUNDIPII2C * sensor, uint32_t passkey) {
   i2c_write_data_uint32(sensor->i2cspm, sensor->m_i2cAddress,THUNDIPI_SLAVE_I2C_THEIR_PASSKEY_OFFSET, passkey);
}
void thundipi_write_address(struct I2C_THUNDIPII2C * sensor, uint8_t * address) {
	i2c_write_data(sensor->i2cspm, sensor->m_i2cAddress,
			THUNDIPI_SLAVE_I2C_MASTER_BLE_ADDR_OFFSET, 6, address);
	uint8_t one=1;
	i2c_write_data(sensor->i2cspm, sensor->m_i2cAddress,
			THUNDIPI_SLAVE_I2C_MASTER_BLE_INDICATOR_OFFSET, 1, &one);
}
uint32_t thundipi_read_passkey(struct I2C_THUNDIPII2C * sensor) {
	uint32_t passkey=0;
   i2c_read_data_uint32(sensor->i2cspm, sensor->m_i2cAddress,THUNDIPI_SLAVE_I2C_OUR_PASSKEY_OFFSET, &passkey);
   return passkey;
}


void sl_thundipii2c_init(struct I2C_THUNDIPII2C * sensor, uint8_t i2c_addr) {
  sensor->i2cspm = sl_i2cspm_THUNDIPII2C;
  sensor->m_i2cAddress=i2c_addr;

}

