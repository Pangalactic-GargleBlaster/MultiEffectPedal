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

unsigned short sampleAbsoluteVoltage(unsigned short sample) {
	return abs(sample - 2048);
}

float samplingPeriod =  0.0000220125; // timer period / clock frequency
#define NOISE_GATE_THRESHOLD 150
#define NOISE_GATE_HOLD_PERIOD 4500 // 100 ms
unsigned short timeSinceLastSound = 0;
bool noiseGateActive = true;
unsigned short lastNoiseGateFilterOutput = 2048;
float noiseGateFilterBeta = 0.99448293057; // 40Hz cutoff
unsigned short noiseGate(unsigned short currentSample) {
	if (sampleAbsoluteVoltage(currentSample) < NOISE_GATE_THRESHOLD) {
		if (!noiseGateActive) {
			noiseGateActive = true;
			lastNoiseGateFilterOutput = currentSample;
		}
		if (timeSinceLastSound > NOISE_GATE_HOLD_PERIOD) {
			unsigned short output = lowPassFilter(currentSample, lastNoiseGateFilterOutput, noiseGateFilterBeta);
			lastNoiseGateFilterOutput = output;
			return output;
		} else {
			timeSinceLastSound++;
			return currentSample;
		}
	} else {
		noiseGateActive = false;
		timeSinceLastSound = 0;
		return currentSample;
	}
}

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
	return currentOutput * 3 / 2;
}

// Octave effects
#define OCTAVE_BUFFER_LENGTH 4096
unsigned short octaveBuffer[OCTAVE_BUFFER_LENGTH];
struct CircularBuffer octaveCircularBuffer = {
	octaveBuffer,
	0,
	OCTAVE_BUFFER_LENGTH
};
#define OCTAVE_UP_SAMPLE_LENGTH (OCTAVE_BUFFER_LENGTH/2)
#define OCTAVE_DOWN_SAMPLE_LENGTH (OCTAVE_UP_SAMPLE_LENGTH/2)
#define CROSSFADE_LENGTH (OCTAVE_DOWN_SAMPLE_LENGTH)
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
	int distortedOutput = 4096/(1.0f+expf((2048.0f-input)/(1024u/gain)));
	return (distortedOutput-2048)*(gain+1)/(4*gain) + 2048; // to counteract the distorted output being a lot louder
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

float lowPassFilter(float currentInput, float previousOutput, float beta) {
	return beta*previousOutput + (1-beta)*currentInput;
}

float highPassFilter(float currentInput, float previousInput, float previousOutput, float alpha) {
	return alpha*((short)previousOutput + currentInput - previousInput);
}

#define ENVELOPE_FILTER_WINDOW_SIZE 440  // 10 ms
#define WAH_TRIGGER_THRESHOLD 600
#define RESET_THRESHOLD 150
#define MIN_FREQUENCY 160
#define MAX_FREQUENCY 4000
#define FREQUENCY_STEP (DELAY_BUFFER_LENGTH/8*ENVELOPE_FILTER_WINDOW_SIZE/delayAmount)
float samplingPeriod =  0.0000220125; // timer period / clock frequency
unsigned short cutoffFrequency = MIN_FREQUENCY;
bool wahTriggered = false;
bool sweepingUp = true;
bool resetHit = true; // after sweeping, did peakValue go below threshold? needed to determine if it's the same note.
void updateCutoffFrequency(unsigned short peakValue, unsigned short delayAmount) {
	if (peakValue > WAH_TRIGGER_THRESHOLD &&!wahTriggered && resetHit) { // threshold hit, it's not already sweeping and it's a new note
		wahTriggered = true;
		resetHit = false;
	} else if (wahTriggered) {
		if (sweepingUp){
			cutoffFrequency += 2 * FREQUENCY_STEP;
			if (cutoffFrequency > MAX_FREQUENCY) {
				cutoffFrequency = MAX_FREQUENCY;
				sweepingUp = false;
			}
		} else {
			cutoffFrequency -= FREQUENCY_STEP;
			if (cutoffFrequency <= MIN_FREQUENCY) {
				cutoffFrequency = MIN_FREQUENCY;
				wahTriggered = false;
				sweepingUp = true;
			}
		}
	}
	if (peakValue < RESET_THRESHOLD && !resetHit) {
		resetHit = true;
	}
}

float lastLPF1Output = 2048;
float lastLPF2Output = 2048;
float lastLPF3Output = 2048;
float lastHPF1Input = 2048;
float lastHPF2Input = 0;
float lastHPF3Input = 0;
float lastHPF1Output = 0;
float lastHPF2Output = 0;
float lastHPF3Output = 0;
float alpha = 0.5;
float beta = 0.5;
unsigned short sampleCount = 0;
unsigned short peakValue = 0;
unsigned short envelopeFilter(unsigned short currentSample, unsigned short delayAmount) {
	float HPF1Output = highPassFilter(currentSample, lastHPF1Input, lastHPF1Output, alpha);
	lastHPF1Input = currentSample;
	lastHPF1Output = HPF1Output;
	float HPF2Output = highPassFilter(HPF1Output, lastHPF2Input, lastHPF2Output, alpha);
	lastHPF2Input = HPF1Output;
	lastHPF2Output = HPF2Output;
	float HPF3Output = highPassFilter(HPF2Output, lastHPF3Input, lastHPF3Output, alpha);
	lastHPF3Input = HPF2Output;
	lastHPF3Output = HPF3Output;
	float LPF1Output = lowPassFilter(HPF3Output, lastLPF1Output, beta);
	lastLPF1Output = LPF1Output;
	float LPF2Output = lowPassFilter(LPF1Output, lastLPF2Output, beta);
	lastLPF2Output = LPF2Output;
	float LPF3Output = lowPassFilter(LPF2Output, lastLPF3Output, beta);
	lastLPF3Output = LPF3Output;

	unsigned short sampleVoltage = sampleAbsoluteVoltage(currentSample);
	peakValue = sampleVoltage > peakValue ? sampleVoltage : peakValue;
	sampleCount = (sampleCount + 1) % ENVELOPE_FILTER_WINDOW_SIZE;
	if (sampleCount == 0){
		updateCutoffFrequency(peakValue, delayAmount);
		beta = calculateBeta(cutoffFrequency, samplingPeriod);
		alpha = calculateAlpha(cutoffFrequency, samplingPeriod);
		peakValue = 0;
	}
	short gainCompensatedOutput = LPF3Output*24 + (currentSample - 2048)*2/3 + 2048;
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
