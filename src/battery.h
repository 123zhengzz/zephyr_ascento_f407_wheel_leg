#pragma once

#include <stdbool.h>

typedef struct {
	float voltage_v;
	bool valid;
} battery_sample_t;

int battery_init(void);
battery_sample_t battery_read(void);
