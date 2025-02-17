/*
 * effects.h
 *
 *  Created on: Nov 20, 2024
 *      Author: Paolo
 */

#ifndef INC_EFFECTS_H_
#define INC_EFFECTS_H_
#include <stdbool.h>

struct CircularBuffer {
	unsigned short* buffer;
	unsigned short nextElementIndex;
	unsigned short bufferSize;
};
#define DELAY_BUFFER_LENGTH 32768

unsigned short getDelaySample(struct CircularBuffer* buffer, unsigned short delay);
unsigned short delay(unsigned short currentSample, unsigned short delayAmount);
unsigned short octave(unsigned short currentInput, bool octaveDownActive, bool cleanSignalActive, bool octaveUpActive);
unsigned short distortion(unsigned short input, unsigned short gain);
float lowPassFilter(float currentInput, float previousOutput, float beta);
unsigned short envelopeFilter(unsigned short currentSample, unsigned short delayAmount);
float calculateBeta(unsigned short cutoffFrequency, float samplingPeriod);
unsigned short testNote();

#endif /* INC_EFFECTS_H_ */
