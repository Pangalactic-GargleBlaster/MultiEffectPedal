/*
 * Implementation of the effects.
 *
 *  Created on: Nov 20, 2024
 *      Author: Paolo
 */

#include "effects.h"
#include <math.h>

// delay effects foundation
void recordCurrentSampleForDelayEffects(struct CircularBuffer* buffer, unsigned short currentSample) {
	buffer->buffer[buffer->nextElementIndex] = currentSample;
	buffer->nextElementIndex = (buffer->nextElementIndex + 1) % buffer->bufferSize;
}

unsigned short getDelaySample(struct CircularBuffer* buffer, unsigned short delay) {
	return buffer->buffer[(buffer->nextElementIndex - 1u - delay) % buffer->bufferSize];
}

unsigned short delayBuffer[DELAY_BUFFER_LENGTH];
struct CircularBuffer delayCircularBuffer = {
		delayBuffer,
		0,
		DELAY_BUFFER_LENGTH
};
unsigned short delay(unsigned short currentSample, unsigned short delayAmount) {
	unsigned short delaySample = getDelaySample(&delayCircularBuffer, delayAmount);
	unsigned short currentOutput = (currentSample + delaySample) / 2;
	recordCurrentSampleForDelayEffects(&delayCircularBuffer, currentOutput);
	return currentOutput;
}

// Octave effects
#define OCTAVE_BUFFER_LENGTH 2048
unsigned short octaveBuffer[OCTAVE_BUFFER_LENGTH];
struct CircularBuffer octaveCircularBuffer = {
	octaveBuffer,
	0,
	OCTAVE_BUFFER_LENGTH
};
#define OCTAVE_UP_SAMPLE_LENGTH (OCTAVE_BUFFER_LENGTH/2)
#define OCTAVE_DOWN_SAMPLE_LENGTH (OCTAVE_UP_SAMPLE_LENGTH/2)
#define CROSSFADE_LENGTH 256
unsigned short crossfade(unsigned short input1, unsigned short input2, unsigned short time) {
	return (input1*(CROSSFADE_LENGTH-time) + input2*time) / CROSSFADE_LENGTH;
}

unsigned short t_octave_up = 0;
unsigned short octaveUp(unsigned short currentInput) {
	unsigned short delayInput = getDelaySample(&octaveCircularBuffer, OCTAVE_UP_SAMPLE_LENGTH - t_octave_up);
	if (t_octave_up > OCTAVE_UP_SAMPLE_LENGTH - CROSSFADE_LENGTH) {
		unsigned short crossfadeInput = getDelaySample(&octaveCircularBuffer, 2*OCTAVE_UP_SAMPLE_LENGTH - t_octave_up);
		delayInput = crossfade(delayInput, crossfadeInput, t_octave_up + CROSSFADE_LENGTH - OCTAVE_UP_SAMPLE_LENGTH);
	}
	t_octave_up = (t_octave_up + 1) % OCTAVE_UP_SAMPLE_LENGTH;
	return delayInput;
}

unsigned short t_octave_down = 0;
unsigned short octaveDown(unsigned short currentInput) {
	unsigned short delayInput = getDelaySample(&octaveCircularBuffer, (t_octave_down/2)+CROSSFADE_LENGTH);
	if (t_octave_down > 2*OCTAVE_DOWN_SAMPLE_LENGTH - CROSSFADE_LENGTH) {
		unsigned short crossfadeInput = getDelaySample(&octaveCircularBuffer, (t_octave_down/2) - OCTAVE_DOWN_SAMPLE_LENGTH + CROSSFADE_LENGTH);
		delayInput = crossfade(delayInput, crossfadeInput, t_octave_down + CROSSFADE_LENGTH - 2*OCTAVE_DOWN_SAMPLE_LENGTH);
	}
	t_octave_down = (t_octave_down+1)%(2*OCTAVE_DOWN_SAMPLE_LENGTH);
	return delayInput;
}

unsigned short octave(unsigned short currentSample, bool octaveDownActive, bool cleanSignalActive, bool octaveUpActive){
	recordCurrentSampleForDelayEffects(&octaveCircularBuffer, currentSample);
	unsigned short octaveUpSample = octaveUp(currentSample);
	unsigned short octaveDownSample = octaveDown(currentSample);
	int numOfSignalsToMix = octaveUpActive + cleanSignalActive + octaveDownActive;
	if (numOfSignalsToMix != 0){
		return (octaveUpActive*octaveUpSample +
				cleanSignalActive*currentSample +
				octaveDownActive*octaveDownSample)
				/numOfSignalsToMix;
	} else {
		return currentSample;
	}
}

unsigned short distortion(unsigned short input, unsigned short gain){
	unsigned int distortedOutput = 4096/(1.0f+expf((2048.0f-input)/(1024u/gain)));
	return (distortedOutput*(gain+1u) + 2048u*(gain-1u))/(2u*gain); // to counteract the distorted output being a lot louder
}

unsigned short lowPassFilter(unsigned short currentInput, unsigned short previousOutput, float beta) {
	return beta*previousOutput + (1-beta)*currentInput;
}

float calculateCutoffFrequency(unsigned short peakValue) {
	return (peakValue - 2048u)/2u + 40u;
}

#define ENVELOPE_FILTER_WINDOW_SIZE 4410
unsigned short lastOutput = 2048;
float beta = 0.5;
unsigned short sampleCount = 0;
unsigned short peakValue = 0;
unsigned short envelopeFilter(unsigned short currentSample) {
	unsigned short currentOutput = lowPassFilter(currentSample, lastOutput, beta);
	lastOutput = currentOutput;
	peakValue = currentSample > peakValue ? currentSample : peakValue;
	sampleCount = (sampleCount + 1) % ENVELOPE_FILTER_WINDOW_SIZE;
	if (sampleCount == 0){
		beta = calculateBeta(calculateCutoffFrequency(peakValue));
		peakValue = 0;
	}
	return currentOutput;
}

