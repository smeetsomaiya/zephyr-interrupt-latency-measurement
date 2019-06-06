# zephyr-interrupt-latency-measurement
GPIO Interrupt Latency and Context Switch Measurement in Zephyr RTOS for Galileo Gen2

###########

Overview
********
This program takes 3 measurements on Zephyr - Galileo
1. Interrupt latency without background tasks	
There are no other tasks/threads running here, apart from the one that controls the GPIO pin transitions to generate the interrupt.

2. Interrupt latency with background tasks
This uses the PWM controller, instead of the GPIO used for Measurement1. The advantage of using a PWM controller is that, the pin transitions to generate the interrupt are offloaded. This allows the background tasks to run, and context switch to an ISR when the interrupt arrives. The background task is a producer thread, passing messages to the consumer thread using a message queue.


Building and Running
********************
To build for the galileo and write the zephyr.strip image to an sdcard, execute the script provided in the root of the project.
	measure80/run

Steps to execute the script -
1. sudo chmod +x measure80/run
2. ./run

The script assumes that your sdcard is named as "ZEPHYR", you can change it accordingly.
