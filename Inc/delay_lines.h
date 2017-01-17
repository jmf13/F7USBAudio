/*
 * delay_lines.h
 *
 *  Created on: 12 janv. 2017
 *      Author: JMF
 */

#ifndef DELAY_LINES_H_
#define DELAY_LINES_H_

#include "arm_math.h"
#include "stdbool.h"
// Needed to get the sampling frequency constant
#include "usbd_conf.h"

#define  DELAY_LINE_MAX	10 	//Max length of the delay lines correspond to about 0.2ms
#define  MAX_CHANNELS	4 	//maximum number of audio i/o channels

typedef struct {
  float32_t sd; //fractional delay for this channel in samples
  uint8_t idl; //length of a simple delay line for this channel
  uint8_t delay_insertion_index; //buffer index for delay line
  float32_t delay_line[DELAY_LINE_MAX]; //a simple delay line buffer
} delay_lines_data_struct;


void delay_lines_init(delay_lines_data_struct *delay_line, float32_t *delays);
void delay_lines_run(delay_lines_data_struct *instance,float32_t* buffer_input, float32_t* buffer_output, uint16_t size);

#endif /* DELAY_LINES_H_ */
