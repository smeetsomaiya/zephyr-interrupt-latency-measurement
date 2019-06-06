/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <device.h>
#include <version.h>
#include <gpio.h>
#include <pwm.h>
#include <pinmux.h>
#include <errno.h>
#include <posix/time.h>
#include "../boards/x86/galileo/board.h"
#include "../boards/x86/galileo/pinmux_galileo.h"
#include "../drivers/gpio/gpio_dw_registers.h"
#include "../drivers/gpio/gpio_dw.h"
#include <shell/shell.h>
#include <misc/printk.h>
#include <misc/util.h>

#define EDGE_FALLING    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)
#define EDGE_RISING	(GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)

static struct device *pinmux, *gpiob, *pwm0;
static struct gpio_callback gpio_cb;

struct data_item_type {
    u32_t field1;
    u32_t field2;
    u32_t field3;
};

char __aligned(4) my_msgq_buffer[12 * sizeof(struct data_item_type)];
struct k_msgq my_msgq;

struct k_sem my_sem, semA, semB;


int csStart = 0;
int csStop = 0;
int csCount = 0;
int giveStop = 0;

#define STACK_SIZE 500
#define TA_PRIORITY 5
#define TB_PRIORITY 5

K_THREAD_STACK_DEFINE(ta_stack_area, STACK_SIZE);
K_THREAD_STACK_DEFINE(tb_stack_area, STACK_SIZE);
struct k_thread ta_thread_data, tb_thread_data;

unsigned int start, stop;
unsigned volatile int interruptCount = 0, count = 0;
unsigned int msg_val = 0;

int minOne = 0, maxOne = 0, minTwo = 0, maxTwo = 0, minThree = 0, maxThree = 0;
int averageOne = 0, averageTwo = 0, averageThree = 0;

int array1[100] = {0};
int array2[100] = {0}; int diffArray[100] = {0};
int array3[100] = {0};

void measureOneCallback(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	stop = k_cycle_get_32();
	array1[count] = SYS_CLOCK_HW_CYCLES_TO_NS(stop - start);
	++count;
//	printk("measureOneCallback %d\n", count);
}

void measureTwoCallback(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	array2[interruptCount] = k_cycle_get_32();
	++interruptCount;
	printk("measureTwoCallback %d\n", interruptCount);
//	printk("Button pressed at %d\n", SYS_CLOCK_HW_CYCLES_TO_NS());
}

static int initGPIO(bool oneOrTwo) {
	pinmux = device_get_binding(CONFIG_PINMUX_NAME);
	struct galileo_data *dev = pinmux->driver_data;

	gpiob = dev->gpio_dw; 	//retrieving gpio_dw driver struct from pinmux driver
				// Alternatively, gpiob = device_get_binding(PORT);
	if (!gpiob) {
		printk("error\n");
		return -1;
	}

	pwm0 = dev->pwm0; 	//retrieving gpio_dw driver struct from pinmux driver
				// Alternatively, gpiob = device_get_binding(PORT);
	if (!gpiob) {
		printk("error\n");
		return -1;
	}

	int ret;
	if(oneOrTwo) {
		ret = pinmux_pin_set(pinmux,1,PINMUX_FUNC_A); 	//IO1 or zephyr GPIO4 as output
		if(ret < 0)
			printk("error setting pin for IO1\n");
	} else {
		//PWM for output
		ret=pinmux_pin_set(pinmux,5,PINMUX_FUNC_C); 	//IO5 for pwm 7
		if(ret<0)
			printk("error setting pin for IO5\n");
	}
	ret = pinmux_pin_set(pinmux,2,PINMUX_FUNC_B); 	//IO2 or zephyr GPIO5 as input
	if(ret < 0)
		printk("error setting pin for IO2\n");
	
	//Configure interrupt
	ret = gpio_pin_configure(gpiob, 5, GPIO_DIR_IN | GPIO_INT | EDGE_RISING);
	if(ret < 0) {
		printk("error setting pin for IO2\n");
		return ret;
	}
	if(oneOrTwo) {
		gpio_init_callback(&gpio_cb, measureOneCallback, BIT(5));
	} else {
		gpio_init_callback(&gpio_cb, measureTwoCallback, BIT(5));
	}
	ret = gpio_add_callback(gpiob, &gpio_cb);
	if(ret < 0) {
		printk("error in gpio_add_callback %d\n", ret);
		return ret;
	}
	ret = gpio_pin_enable_callback(gpiob, 5);
	if(ret < 0) {
		printk("error in gpio_pin_enable_callback %d\n", ret);
		return ret;
	}
	// printk("Sizeof int %d\n", sizeof(int));
	// printk("Sizeof unsgined int %d\n", sizeof(unsigned int));
	return 0;
}

extern void tb_entry_point_sync(void * a, void * b, void * c) {
	printk("tb_entry_point_sync\n");

	while(csCount < 100) {
		printk("Switched from A\n");
		k_sem_take(&semB, K_FOREVER);


		printk("Switching context to A\n");
		csStart = k_cycle_get_32();
		k_sem_give(&semA); //This triggers context switch to Thread A
	}
}

extern void ta_entry_point_sync(void * a, void * b, void * c) {
	printk("ta_entry_point_sync\n");

	k_tid_t tb_tid = k_thread_create(&tb_thread_data, tb_stack_area,
                                 	K_THREAD_STACK_SIZEOF(tb_stack_area),
                                 	tb_entry_point_sync,
                                 	NULL, NULL, NULL,
                                 	TA_PRIORITY+1, 0, K_NO_WAIT); //Assign lower priority to Thread B
	k_thread_name_set(tb_tid, "threadB");

	csCount = 0;
	while(csCount < 100) {
		printk("Wait for semA\n");
		k_sem_take(&semA, K_FOREVER); //SemA is unavailable, so thread A goes into waiting
		csStop = k_cycle_get_32();
		k_sem_give(&semB);
		giveStop = k_cycle_get_32();
	
		printk("Switched from B\n");

		array3[csCount++] = SYS_CLOCK_HW_CYCLES_TO_NS((csStop - csStart) - (giveStop - csStop));
	
//		printk("ThreadA going to sleep\n");
//		k_sleep(10);
//		printk("ThreadA after wakeup\n");
	}
}

extern void ta_entry_point(void * a, void * b, void * c) {

	struct data_item_type data;

	while(interruptCount < 100) {
		if(k_sem_take(&my_sem, K_FOREVER) != 0) {
			printk("Input data not available!");
		}
		data.field1 = ++msg_val;
		while (k_msgq_put(&my_msgq, &data, K_NO_WAIT) != 0) {
            /* message queue is full: purge old data & try again */
			if(k_sem_take(&my_sem, K_MSEC(50)) != 0) {
				printk("Buffer full!");
			}
	    }
	    /* data item was successfully added to message queue */
	    printk("Produced %d\n", data.field1);
	    //sem post
	    k_sem_give(&my_sem);
	    k_sleep(10);
	}
}

extern void tb_entry_point(void * a, void * b, void * c) {
	struct data_item_type data;
	while (interruptCount < 100) {
		if(k_sem_take(&my_sem, K_FOREVER) != 0) {
			printk("Input data not available!");
		} else {
        	/* get a data item */
        	k_msgq_get(&my_msgq, &data, K_FOREVER);

    	    /* process data item */
	        printk("Received %d\n", data.field1);
	        k_sem_give(&my_sem);
	        k_sleep(10);
	    }
    }
}

//Interrupt latency w/o background tasks
static int measureOne(const struct shell *shell, size_t argc,
			     char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = initGPIO(true); //Set the GPIO and PWM configs
	if(ret) {
		printk("Error initializing GPIO or PWM\n");
		return -1;
	}

//	int val = 0;

	count = maxOne = averageOne = 0;
	minOne = 100000000;

	while(count < 100) {
 		ret = gpio_pin_write(gpiob, 4, 0);
		start = k_cycle_get_32();
  		ret = gpio_pin_write(gpiob, 4, 1);
// 		ret = gpio_pin_read(gpiob, 5, &val);
// 		printk("IO2 read %d Err %d\n", val, ret);

//  		ret = gpio_pin_read(gpiob, 5, &val);
// 		printk("IO2 read %d Err %d\n", val, ret);
 		k_sleep(20);
	}
	ret = gpio_pin_write(gpiob, 4, 0);

	for(int i = 0; i < 100; ++i) {
		printk("%d\t", array1[i]);
		if(array1[i] > maxOne) {
			maxOne = array1[i];
		}
		if(array1[i] < minOne) {
			minOne = array1[i];
		}
		averageOne += array1[i];
	}
	averageOne /= 100;
	printk("Min value %d ns\n", minOne);
	printk("Max value %d ns\n", maxOne);
	printk("Average value %d ns\n", averageOne);
	return 0;
}

//Interrupt latency w/ background tasks
static int measureTwo(const struct shell *shell, size_t argc,
			     char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int ret = initGPIO(false);
	if(ret) {
		printk("Error initializing GPIO or PWM\n");
		return -1;
	}

	interruptCount = maxTwo = averageTwo = 0;
	minTwo = 100000000;

	k_sem_init(&my_sem, 1, 1);
	k_msgq_init(&my_msgq, my_msgq_buffer, sizeof(struct data_item_type), 12);

	//Spawn 2 message passing threads to run in the background
	k_tid_t ta_tid = k_thread_create(&ta_thread_data, ta_stack_area,
                                 	K_THREAD_STACK_SIZEOF(ta_stack_area),
                                 	ta_entry_point,
                                 	NULL, NULL, NULL,
                                 	TA_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(ta_tid, "threadA");
	k_tid_t tb_tid = k_thread_create(&tb_thread_data, tb_stack_area,
                                 	K_THREAD_STACK_SIZEOF(tb_stack_area),
                                 	tb_entry_point,
                                 	NULL, NULL, NULL,
                                 	TB_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(tb_tid, "threadB");
	ret = pwm_pin_set_cycles(pwm0, 3, 0, 1638); //1638 equvivalent to 40% duty cycle @200Hz (default freq of pca9685)	// start pwm 3 with a small duty cycle
	if(ret<0)
		printk("error pwm_pin_set_cycles on PWM7\n");

	while(interruptCount < 100) {
		printk("");
	}
	ret = pwm_pin_set_cycles(pwm0, 3, 0, 0);	// Turn off pwm3
	if(ret<0)
		printk("error pwm_pin_set_cycles on PWM7\n");

	//Process the measurements

	for(int i = 0; i < 99; ++i) {
		diffArray[i] = SYS_CLOCK_HW_CYCLES_TO_NS(array2[i+1] - array2[i]);
		printk("%d\t", diffArray[i]);
		if(diffArray[i] > maxTwo) {
			maxTwo = diffArray[i];
		}
		if(diffArray[i] < minTwo) {
			minTwo = diffArray[i];
		}
		averageTwo += diffArray[i];
	}
	averageTwo /= 100;

	int max_delta = (maxTwo - minTwo + 2*(maxOne - minOne))/2;
	
//	printk("Min value %d ns\n", minTwo);
//	printk("Max value %d ns\n", maxTwo);
//	printk("Average value %d ns\n", averageTwo);
	printk("\nWorst case delta %d ns\n", max_delta);
	return 0;
}

//Context switch overhead measurement
static int measureThree(const struct shell *shell, size_t argc,
			     char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	k_sem_init(&semA, 0, 1); //Starts as "unavailable". Second field is inital count, last is max count 
	k_sem_init(&semB, 1, 1); //Starts as "available"

	//Spawn 2 message passing threads to run in the background
	k_tid_t ta_tid = k_thread_create(&ta_thread_data, ta_stack_area,
                                 	K_THREAD_STACK_SIZEOF(ta_stack_area),
                                 	ta_entry_point_sync,
                                 	NULL, NULL, NULL,
                                 	TA_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(ta_tid, "threadA");
	k_tid_t tb_tid = k_thread_create(&tb_thread_data, tb_stack_area,
                                 	K_THREAD_STACK_SIZEOF(tb_stack_area),
                                 	tb_entry_point,
                                 	NULL, NULL, NULL,
                                 	TA_PRIORITY+1, 0, K_NO_WAIT);
	k_thread_name_set(tb_tid, "threadB");

	maxThree = averageThree = 0;
	minThree = 100000000;

	while(csCount < 100) {
		printk("");
	}

	for(int i = 0; i < 100; ++i) {
		printk("%d\t", array3[i]);
		if(array3[i] > maxThree) {
			maxThree = array3[i];
		}
		if(array3[i] < minThree) {
			minThree = array3[i];
		}
		averageThree += array3[i];
	}
	averageThree /= 100;
	printk("Min value %d ns\n", minThree);
	printk("Max value %d ns\n", maxThree);
	printk("Average value %d ns\n", averageThree);

	return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(sub_cmd_measure,
	SHELL_CMD_ARG(1, NULL, "Measure value 1", measureOne, 0, 0),
	SHELL_CMD_ARG(2, NULL, "Measure value 2", measureTwo, 0, 0),
	SHELL_CMD_ARG(3, NULL, "Measure value 3", measureThree, 0, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(measure, &sub_cmd_measure, "Start measurement", NULL);

void main(void)
{
	printk("Init done. %s\n", CONFIG_BOARD);
}
