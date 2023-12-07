#pragma once
#include "stdint.h"

extern volatile int32_t uptime_seconds;

uint64_t get_uptime_cycles(void);
void timeKeeperISR (void);
void configureDebounceTimer(void);
