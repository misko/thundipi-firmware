#include <stddef.h>
#include <machine/endian.h>
#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "sl_sleeptimer.h"
#include "sl_i2cspm_instances.h"
#include "app.h"
#include "THUNDIPII2C.h"
#include "em_cmu.h"
#include "i2c_utils.h"


sl_status_t i2c_read_data(sl_i2cspm_t *i2cspm, uint8_t i2c_addr, uint8_t reg,
		uint16_t len, uint8_t *data) {
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	//uint8_t i2c_read_data[2];
	//i2c_read_data[0] = 0;
	//i2c_read_data[1] = 0;
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
	seq.buf[0].data = data;
	seq.buf[0].len = len;

	ret = I2CSPM_Transfer(i2cspm, &seq);

	if (ret != i2cTransferDone) {
		*data = 0;
		return ret;
	}

	//*data = ((uint32_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);
	//*data = (i2c_read_data[0] << 8) + (i2c_read_data[1]);

	return SL_STATUS_OK;
}

sl_status_t i2c_read_data_uint32(sl_i2cspm_t *i2cspm, uint8_t i2c_addr,
		uint8_t reg, uint32_t *data) {
	sl_status_t sc =  i2c_read_data(i2cspm, i2c_addr, reg, 4, (uint8_t*)data);
	*data = __ntohl(*data);
	return sc;
}

sl_status_t i2c_read_data_uint16(sl_i2cspm_t *i2cspm, uint8_t i2c_addr,
		uint8_t reg, uint16_t *data) {
	//*data = __ntohs(*data);
	/*data = (data[0] << 8) + (data[1]);
	/uint8_t tmp = data[1];
	data[1]=data[0];
	data[0]=data[1];888
	printf("WTF %d %d\r\n\n",data[0],data[1])*/
	sl_status_t sc = i2c_read_data(i2cspm, i2c_addr, reg, 2, (uint8_t*)data);
	//printf("GOT DATA %d\r\n\n",*data);
	*data = __ntohs(*data);
	return sc;
}

sl_status_t i2c_write_data(sl_i2cspm_t *i2cspm, uint8_t i2c_addr,
		uint8_t reg, uint16_t len, uint8_t * value) {
	sl_status_t retval = SL_STATUS_OK;
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_read_data[2];
	uint8_t i2c_write_data[16] ;
	if (len>16-1)  {
		printf("WAYT O BIG TO SEND!\n");
		return 0;
	}
	memcpy(i2c_write_data+1,value,len);

	seq.addr = i2c_addr << 1;
	seq.flags = I2C_FLAG_WRITE;
	/* Select command to issue */
	i2c_write_data[0] = reg;
	//i2c_write_data[1] = value >> 8;
	//i2c_write_data[2] = value & 0xFF;
	seq.buf[0].data = i2c_write_data;
	seq.buf[0].len = len+1;
	/* Select location/length of data to be read */
	seq.buf[1].data = i2c_read_data;
	seq.buf[1].len = 0;

	ret = I2CSPM_Transfer(i2cspm, &seq);

	if (ret != i2cTransferDone) {
		printf("ERROR ON SEND!\n");
		retval = SL_STATUS_TRANSMIT;
	}
	return retval;
}

sl_status_t i2c_write_data_uint16(sl_i2cspm_t *i2cspm, uint8_t i2c_addr,
		uint8_t reg, int16_t value) {
	return i2c_write_data(i2cspm, i2c_addr,
			reg, 2, (uint8_t*)&value);
}

sl_status_t i2c_write_data_uint32(sl_i2cspm_t *i2cspm, uint8_t i2c_addr,
		uint8_t reg, int32_t value) {
	return i2c_write_data(i2cspm, i2c_addr,
			reg, 4, (uint8_t*)&value);
}



