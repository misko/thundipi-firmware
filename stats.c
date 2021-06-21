#include <stdlib.h>
#include "stats.h"
#include "app.h"
#include "timers.h"
#include "nvm3_default.h"
#include "INA3221.h"

uint16_t stats_version;
/*
 *
 * Computing life time of writes
 * size of nvm3 is 40960 bytes
 * have 2(sets) * 3(doubles) * 4bytes = 2*3*4 = 24 bytes
 * with 2(keys)*4bytes = 8bytes
 * for a total of 32bytes per write
 * Can write 40960/32  = 128 times before clear full nvm
 * and have 10000 writes before we break the flash
 * 1,280,000 writes
 * At one write per minute we have
 * 1280000/(60minutes*24hours*365days) ~ 2.4 years
 * If we write every 30minutes -> 30*2.4 ~ 73 years
 *
 */
void read_stats() {
	uint32_t objectType;
	size_t data_len;

	// Find size of data for object with key identifier 1 and 2 and read out
    nvm3_getObjectInfo(nvm3_defaultHandle, STATS_LIFE_ACCUMULATOR_KEY, &objectType, &data_len);
	if (objectType == NVM3_OBJECTTYPE_DATA) {
	  nvm3_readData(nvm3_defaultHandle, STATS_LIFE_ACCUMULATOR_KEY, life_accumulator, data_len);
	}
    nvm3_getObjectInfo(nvm3_defaultHandle, STATS_TRIP_ACCUMULATOR_KEY, &objectType, &data_len);
	if (objectType == NVM3_OBJECTTYPE_DATA) {
	  nvm3_readData(nvm3_defaultHandle, STATS_TRIP_ACCUMULATOR_KEY, life_accumulator, data_len);
	}
}

void init_stats() {
	stats_version=STATS_VERSION;
	size_t num_obj = nvm3_countObjects(nvm3_defaultHandle);
	printf("FOUND %d objects in NVM\r\n\n",num_obj);
	if (1==1 || num_obj==0) {
		printf("SETTING UP NVM\r\n\n");
		nvm3_writeData(nvm3_defaultHandle, STATS_VERSION_KEY, &stats_version, sizeof(uint16_t));
		memset(life_accumulator,0,sizeof(double)*NRELAYS);
		memset(trip_accumulator,0,sizeof(double)*NRELAYS);

		nvm3_writeData(nvm3_defaultHandle, STATS_LIFE_ACCUMULATOR_KEY, life_accumulator, sizeof(double)*NRELAYS);
		nvm3_writeData(nvm3_defaultHandle, STATS_TRIP_ACCUMULATOR_KEY, trip_accumulator, sizeof(double)*NRELAYS);
	}
	num_obj = nvm3_countObjects(nvm3_defaultHandle);
	read_stats();

}

void update_stats() {
	for (int i=0; i<NRELAYS; i++) {
		double power_in_kwh = power[i]*MONITOR_DLAY_SEC/(60.0*60.0*1000.0);
		life_accumulator[i]+=power_in_kwh;
		trip_accumulator[i]+=power_in_kwh;
		power[i]=0.0;
	}
	nvm3_writeData(nvm3_defaultHandle, STATS_LIFE_ACCUMULATOR_KEY, life_accumulator, sizeof(double)*NRELAYS);
	nvm3_writeData(nvm3_defaultHandle, STATS_TRIP_ACCUMULATOR_KEY, trip_accumulator, sizeof(double)*NRELAYS);
}

void print_stats() {
	for (int i=0; i<NRELAYS; i++)  {
		int32_t x= life_accumulator[i];
		printf("%d\t",x);
	}
	printf("\r\n\n");
	for (int i=0; i<NRELAYS; i++)  {
		int32_t x= trip_accumulator[i];
		printf("%d\t",x);
	}
	printf("\r\n\n");
}
