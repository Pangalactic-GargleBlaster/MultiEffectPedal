# Project
This is the code for a digital multi-effect guitar/bass pedal.
# Chip
It's based on an STM32L4 chip. It uses the on-board analog to digital converter (ADC), digital to analog Converter (DAC), Op-Amp, direct memory access (DMA) controller and timers.
# Software Design
The device needs to do 3 tasks:
1. Sample the input signal into a buffer and produce the output signal from a different buffer
2. Calculate the output waveform based on the input and on which effects are currently enabled
3. Sample the control switches and potentiometers and update which effects are enabled and their parameters
The first task is handled by the DMA controller and runs at 44KHz. That's an industry standard which is chosen because the human ear can perceive frequencies of up to 20KHz and our sampling frequency needs to be at least twice as high as the highest frequency we want to capture.
The second task is run within an interrup handler that is called whenever the input buffer is half full or completely full. This implementation is called ping-pong buffer. The buffer is 88 samples wide so the task runs every millisecond, which is below the threshold of delay that would be perceived when playing.
The third task is run in the main loop of the microcontroller every 17 milliseconds.
# Code Organization
The chip peripherals (DMA, ADC, DAC, Op-Amp and Timers) are configured in the .ioc file. This was edited with the STM Cube IDE.
The ping-pong buffer and control logic are implemented in the [main.c](Core/Src/main.c) file.
The effects are implemented in the [effects.c](Core/Src/effects.c) file.
# Effects
This project implements 5 effects:
1. Distortion
2. Echo
3. Octave up
4. Octave down
5. Envelope filter
The distortion is obtained by applying a sigmoid function to the input with a variable gain set with a potentiometer.
The Echo is implemented with a circular buffer where we store a signal obtained by mixing the current input and a delayed version. The delay amount is controlled with a potentiometer.
The octave effects are built atop the foundation of the echo effect. Shifting the signal up an octave means playing it twice as fast, whereas shifting it down an octave means playing it twice as slow. This is obtained by varying the delay amount over time. Two such signals are crossfaded back and forth to avoid artifacts created by the circular buffer.
The envelope filter sweeps the cutoff frequency of a band pass filter up and down whenever the input is higher than a threshold. This creates the "WahWah" sound which is common in funk music.
