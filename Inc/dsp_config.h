/* @file         dsp_config.h
 * @version      1.0
 * @date         27/11/2016
 * @author       JM. Fourneron
 * @compiler     GCC
 * @copyright    JM. Fourneron
 */
 
#ifndef __DSP_CONFIG_H
#define __DSP_CONFIG_H

/* ----------------------------------------------------------------------
** definition of all the parameters for the filters and digital signal
** processing to be performed.
**
** The function process the input buffer with the below sequence:
** - convert the buffer from q15 to float
** - scale the input by SCALE_INPUT value
** - apply Filter1
** - on the result of Filter1, apply Filter2 and fill a new buffer,
**	scale by SCALE_OUTPUT1_3 value, and invert if requested by INVERT_OUTPUT1_3
**	convert from float to q15
** - on the result of Filter1, apply Filter3 and fill a new buffer,
** 	scale by SCALE_OUTPUT2_4 value, and invert if requested by INVERT_OUTPUT2_4
**	convert from float to q15
**
** Filter1, Filter 2 and Filter3 are Biquad cascades defined by:
** - NUMSTAGES_FILTERx
** - CoeffTableFilterx
** Biquad filters format is b0, b1, b2, a1, a2
**
** The digital processing is the same for the Left and Right channels
** This part is the only part to modify for the specific application
** ------------------------------------------------------------------- */

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!! Below biquad parameters values have been truncated for IP protection purpose !!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


#define SCALE_INPUT		0.3	//scaling factor applied to input buffer

#define NUMSTAGES_FILTER1	2	//number of biquads in this filter
float32_t coeffTableFilter1[5*NUMSTAGES_FILTER1] = {
  /* xxxxxx */
		1.0037916, -1.91626167, 0.96692099, 1.9162616, -0.97071262,  
		1.0521409, 0.93130519, 0.8996312, -0.9313051, -0.951772	  
};



#define SCALE_OUTPUT1_3_dB		0	//scaling factor applied to output buffer 1 and 3 in dB
#define INVERT_OUTPUT1_3 	false

#define NUMSTAGES_FILTER2	4	//number of biquads in this filter
float32_t coeffTableFilter2[5*NUMSTAGES_FILTER2] = {
		0.0019216, 0.0038433, 0.0019216, 1.8246513, -0.8323380, 	
		1.003858, -1.9937278, 0.989912, 1.9937278, -0.993770,	
		0.925455, -1.4315014, 0.897376, 1.4315014, -0.822832,	
		0.8841380, -1.204926, 0.77697153, 1.20492634, -0.661109	
};


#define SCALE_OUTPUT2_4_dB		-9.6	//scaling factor applied to output buffers 2 and 4 in dB
#define INVERT_OUTPUT2_4 		true

#define NUMSTAGES_FILTER3	6	//number of biquads in this filter
float32_t coeffTableFilter3[5*NUMSTAGES_FILTER3] = {
  /* xxxxxx */
		0.91424735, -1.82849470, 0.9142473, 1.82465130, -0.83233809,	
		1.123687, -1.824401, 0.74051730, 1.841148795, -0.8474572,	
		0.964914, -1.706046, 0.844942, 1.7060466, -0.809857,	
		1.825672, -1.5456684, 0.5433245, 0.3675637, -0.1908928,	
		1.0380449, -0.469900, 0.7775114, 0.469900, -0.8155,	
		0.962297, 0.26226282, 0.86541170, -0.2622628, -0.827709
};

/* ----------------------------------------------------------------------
** End of definition of all the parameters for the filters and digital signal
** processing to be performed.
** ------------------------------------------------------------------- */

#endif