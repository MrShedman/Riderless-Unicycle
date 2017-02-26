#pragma once

#include "Arduino.h"

#define MAX_FIR_DENOISE_WINDOW_SIZE 120

typedef struct pt1Filter_s {
	float state;
	float k;
	float RC;
	float dT;
} pt1Filter_t;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
	float b0, b1, b2, a1, a2;
	float d1, d2;
} biquadFilter_t;

typedef struct firFilterDenoise_s {
	int filledCount;
	int targetCount;
	int index;
	float movingSum;
	float state[MAX_FIR_DENOISE_WINDOW_SIZE];
} firFilterDenoise_t;

typedef enum {
	FILTER_PT1 = 0,
	FILTER_BIQUAD,
	FILTER_FIR
} filterType_e;

typedef enum {
	FILTER_LPF,
	FILTER_NOTCH
} biquadFilterType_e;

typedef struct firFilter_s {
	float *buf;
	const float *coeffs;
	float movingSum;
	uint8_t index;
	uint8_t count;
	uint8_t bufLength;
	uint8_t coeffsLength;
} firFilter_t;

typedef float(*filterApplyFnPtr)(void *filter, float input);

float nullFilterApply(void *filter, float input);

void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);
float biquadFilterApply(biquadFilter_t *filter, float input);
float filterGetNotchQ(uint16_t centerFreq, uint16_t cutoff);

void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT);
float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT);

void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs);
void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength, const float *coeffs, uint8_t coeffsLength);
void firFilterUpdate(firFilter_t *filter, float input);
void firFilterUpdateAverage(firFilter_t *filter, float input);
float firFilterApply(const firFilter_t *filter);
float firFilterUpdateAndApply(firFilter_t *filter, float input);
float firFilterCalcPartialAverage(const firFilter_t *filter, uint8_t count);
float firFilterCalcMovingAverage(const firFilter_t *filter);
float firFilterLastInput(const firFilter_t *filter);

void firFilterDenoiseInit(firFilterDenoise_t *filter, uint8_t gyroSoftLpfHz, uint16_t targetLooptime);
float firFilterDenoiseUpdate(firFilterDenoise_t *filter, float input);
