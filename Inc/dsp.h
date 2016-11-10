/**
 * @file         dsp.h
 * @version      1.0
 * @date         2016
 * @author       JM. Fourneron
 * @compiler     armcc
 * @copyright    Christoph Lauer engineering
 */
 
#ifndef __DSP_H
#define __DSP_H

#include <stdint.h>

void dsp(int16_t* buffer, int length, uint8_t channel);
void initFilter(void);

#endif
