/**
 * @file         dsp.h
 * @version      1.0
 * @date         2015
 * @author       Christoph Lauer
 * @compiler     armcc
 * @copyright    Christoph Lauer engineering
 */
 
#ifndef __DSP_H
#define __DSP_H

#include <stdint.h>

void dsp(int16_t* buffer_input, int16_t* buffer_outputA, int16_t* buffer_outputB, uint16_t size, uint8_t channel);
void initFilter(void);

#endif
