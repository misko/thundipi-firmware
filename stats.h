

#ifndef STATS_H
#define STATS_H


#include "app.h"
#define STATS_VERSION 0x1
#define STATS_VERSION_KEY 0x100
#define STATS_LIFE_ACCUMULATOR_KEY 0x101
#define STATS_TRIP_ACCUMULATOR_KEY 0x102
extern uint16_t stats_version;

extern double life_accumulator[NRELAYS];
extern double trip_accumulator[NRELAYS];

void init_stats();
void print_stats();
void update_stats();


#endif
