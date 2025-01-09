/*
 * Implementation of the effects.
 *
 *  Created on: Nov 20, 2024
 *      Author: Paolo
 */

#include "effects.h"
#include "main.h"
#include "stm32l4xx_hal.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

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
#define CROSSFADE_LENGTH OCTAVE_DOWN_SAMPLE_LENGTH
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

// envelope filter

float calculateAlpha(unsigned short cutoffFrequency, float samplingPeriod) {
    float omega = 2.0f * 3.14159265359f * cutoffFrequency;
	return 1 / (omega*samplingPeriod + 1);
}

float calculateBeta(unsigned short cutoffFrequency, float samplingPeriod) {
    float omega = 2.0f * 3.14159265359f * cutoffFrequency;
    return expf(-omega * samplingPeriod);
}

unsigned short lowPassFilter(unsigned short currentInput, unsigned short previousOutput, float beta) {
	return beta*previousOutput + (1-beta)*currentInput;
}

short highPassFilter(unsigned short currentInput, unsigned short previousInput, unsigned short previousOutput, float alpha) {
	return alpha*((short)previousOutput + currentInput - previousInput);
}

#define ENVELOPE_FILTER_WINDOW_SIZE 250
float samplingPeriod =  0.0000220125;
unsigned short cutoffFrequency = 250;
bool wahTriggered = false;
bool sweepingUp= true;
#define WAH_THRESHOLD 1000
void updateCutoffFrequency(unsigned short peakValue) {
	if (wahTriggered) {
		if (sweepingUp){
			cutoffFrequency += 40;
			if (cutoffFrequency > 1000) {
				sweepingUp = false;
			}
		} else {
			cutoffFrequency -= 40;
			if (cutoffFrequency <= 40) {
				wahTriggered = false;
				cutoffFrequency = 250;
			}
		}
	} else if (peakValue > WAH_THRESHOLD) {
		wahTriggered = true;
		sweepingUp = true;
	}
}

unsigned short lastLPFOutput = 2048;
unsigned short lastHPFOutput = 2048;
unsigned short lastInput = 2048;
float alpha = 0.5;
float beta = 0.5;
unsigned short sampleCount = 0;
unsigned short peakValue = 0;
unsigned short envelopeFilter(unsigned short currentSample) {
	unsigned short HPFOutput = highPassFilter(currentSample, lastInput, lastHPFOutput, alpha);
	lastInput = currentSample;
	lastHPFOutput = HPFOutput;
	unsigned short LPFOutput = lowPassFilter(2048u + HPFOutput, lastLPFOutput, beta);
	lastLPFOutput = LPFOutput;

	peakValue = abs(currentSample - 2048) > peakValue ? abs(currentSample - 2048) : peakValue;
	sampleCount = (sampleCount + 1) % ENVELOPE_FILTER_WINDOW_SIZE;
	if (sampleCount == 0){
		updateCutoffFrequency(peakValue);
		//sendNumberToComputer(cutoffFrequency);
		beta = calculateBeta(cutoffFrequency, samplingPeriod);
		alpha = calculateAlpha(cutoffFrequency, samplingPeriod);
		peakValue = 0;
	}
	short gainCompensatedOutput = (LPFOutput - 2048)*2 + 2048;
	if (gainCompensatedOutput < 0) {
		return 0;
	} else if (gainCompensatedOutput > 4095) {
		return 4095;
	} else {
		return gainCompensatedOutput;
	}
}

// test note generation
unsigned int sampleNumber = 0;
char note = 'C';
float wave[10] = {0, 0.5877852, 0.9510565, 0.95105651, 0.5877852, 0, -0.5877852, -0.9510565, -0.95105651, -0.58778525};
unsigned short testNote() {

	unsigned short frequency;
	if (note == 'C') {
		frequency = 261;
	} else if (note == 'D') {
		frequency = 293;
	} else {
		frequency = 330;
	}
	unsigned int period = 45000/frequency;
	unsigned int index = (sampleNumber % period) * 10 / period;
	sampleNumber++;
	if (sampleNumber == 45000){
		if (note == 'E') {
			note = 'C';
		} else {
			note++;
		}
		sampleNumber = 0;
	}

	return 2048+(wave[index] * 1024);
}
