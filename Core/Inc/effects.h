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
unsigned short lowPassFilter(unsigned short currentInput, unsigned short previousOutput, float beta);
unsigned short envelopeFilter(unsigned short currentSample, float beta);

#endif /* INC_EFFECTS_H_ */
