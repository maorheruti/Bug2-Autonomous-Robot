#pragma once
#include <Arduino.h>

#define SENSOR_INVALID          8191
#define SENSOR_OFFLINE_RAW      65535
#define SENSOR_INVALID_RAW_MIN  8000

#define SENSOR_GRID_W    8
#define SENSOR_GRID_SIZE 64

enum SensorKind : uint8_t {
	SENS_VALID = 0,
	SENS_CLEAR = 1,
	SENS_INVALID = 2,
	SENS_CORRUPT = 3
};

inline SensorKind sensor_kind(uint16_t d){
	if (d == SENSOR_OFFLINE_RAW || d == 0) return SENS_INVALID;
	if (d > SENSOR_INVALID) return SENS_CORRUPT;
	if (d >= 8190) return SENS_CLEAR;
	return SENS_VALID;
}

inline char sensor_kind_char(SensorKind k){
	switch (k){
		case SENS_VALID: return 'V';
		case SENS_CLEAR: return 'C';
		case SENS_INVALID: return 'I';
		case SENS_CORRUPT: return 'X';
		default: return '?';
	}
}

struct SensorMatrix {
	int16_t distance[SENSOR_GRID_SIZE];
	uint8_t status[SENSOR_GRID_SIZE];
};

struct SensorZone4 {
	uint16_t z[4];
	uint8_t  nValid[4];   // valid cells per zone (status 5 or 9)
};

size_t sensor_format_mm(uint16_t d, char* out, size_t outSize);

void sensors_init();

// Compatibility API (returns single representative value per sensor)
void sensors_read(uint16_t &dL, uint16_t &dM, uint16_t &dR);

// Native matrix API
void sensors_read_matrix(SensorMatrix &left, SensorMatrix &mid, SensorMatrix &right);

// Snapshot latest 4 zone values (2 columns per zone)
void sensors_copy_zone4(SensorZone4 &left, SensorZone4 &mid, SensorZone4 &right);

void sensors_request_reset_all(int whyCode = 0);
void sensors_request_reset_one(int idx, int whyCode = 0);
void sensors_service();
bool sensors_is_recovering();
void sensors_force_reset_all();
void sensors_reset_one(int idx, int whyCode = 2);

uint32_t sensors_last_good_ms();
uint32_t sensors_last_timeout_ms();

void sensors_test_window_start(uint32_t durationMs);
void sensors_test_slow_reads_start(uint32_t durationMs, uint32_t extraDelayUs);
