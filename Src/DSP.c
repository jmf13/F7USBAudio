/*
 * DSP.c
 *
 *  Created on: 19 juil. 2016
 *      Author: JMF
 */

#include <delay_lines.h>
#include "dsp.h"
#include "usbd_audio.h"
#include "stdbool.h"
#include "arm_math.h"
#include "dsp_config.h"

#define SCALE_OUTPUT1_3	pow(10,SCALE_OUTPUT1_3_dB/20)
#define SCALE_OUTPUT2_4	pow(10,SCALE_OUTPUT2_4_dB/20)


/* ----------------------------------------------------------------------
** Instance structures and state buffers for biquads cascades associated 
** to the different filters 
** ------------------------------------------------------------------- */

arm_biquad_casd_df1_inst_f32 F1L, F1R, F2L, F2R, F3L, F3R;

static float32_t biquadStatef32_F1L [4 * NUMSTAGES_FILTER1];
static float32_t biquadStatef32_F1R [4 * NUMSTAGES_FILTER1];
static float32_t biquadStatef32_F2L [4 * NUMSTAGES_FILTER2];
static float32_t biquadStatef32_F2R [4 * NUMSTAGES_FILTER2];
static float32_t biquadStatef32_F3L [4 * NUMSTAGES_FILTER3];
static float32_t biquadStatef32_F3R [4 * NUMSTAGES_FILTER3];


/* ----------------------------------------------------------------------
** f32 input and output buffers
** ------------------------------------------------------------------- */
float32_t inputf32[AUDIO_OUTPUT_BUF_SIZE/2];
float32_t outputf32[AUDIO_OUTPUT_BUF_SIZE/2];

/* ----------------------------------------------------------------------
** Instance structures for delay lines
** ------------------------------------------------------------------- */
static delay_lines_data_struct delay_lines[MAX_CHANNELS]; //one per channel

/* ?? check if we can have an initialization of the filters by the client application
bool firstStart = false; */



/* ----------------------------------------------------------------------
** the core dsp function
** This functions applies the filters cascade described above
** in the definition of the filters
** ------------------------------------------------------------------- */

void dsp(int16_t* buffer_input, int16_t* buffer_outputA, int16_t* buffer_outputB, uint16_t size, uint8_t channel)
{
	arm_biquad_casd_df1_inst_f32 *F1;
	arm_biquad_casd_df1_inst_f32 *F2;
	arm_biquad_casd_df1_inst_f32 *F3;
	delay_lines_data_struct *D1; // Delay on Woofer
	delay_lines_data_struct *D2; // Delay on Tweeter


	// ?? check if needed
	//float32_t *biquadStatef32;

	if (channel == 0){
		F1 = &F1L;
		F2 = &F2L;
		F3 = &F3L;
		D1 = &delay_lines[0];
		D2 = &delay_lines[1];
	}
	else
	{
		F1 = &F1R;
		F2 = &F2R;
		F3 = &F3R;
		D1 = &delay_lines[2];
		D2 = &delay_lines[3];
	};
	

	/* ----------------------------------------------------------------------
	** Convert block of input data from float to Q31
	** ------------------------------------------------------------------- */
	arm_q15_to_float(buffer_input, inputf32, size);
	
	/* ----------------------------------------------------------------------
	** Scale  by SCALE_INPUT.  This provides additional headroom
	** ------------------------------------------------------------------- */
	if (SCALE_INPUT != 1) {
		arm_scale_f32(inputf32, SCALE_INPUT,inputf32, size);
	}
	
	/* ----------------------------------------------------------------------
	** Apply F1 to the input
	** ------------------------------------------------------------------- */
	arm_biquad_cascade_df1_f32(F1, inputf32, inputf32, size);

	/* ----------------------------------------------------------------------
	** Apply F2 to the result - creates a new filter "branch"
	** ------------------------------------------------------------------- */
	arm_biquad_cascade_df1_f32(F2, inputf32, outputf32, size);

	if (SCALE_OUTPUT1_3 != 1) {
		arm_scale_f32(outputf32, SCALE_OUTPUT1_3,outputf32, size);
	}

	if (INVERT_OUTPUT1_3) {
		arm_negate_f32 (outputf32,outputf32, size);
	}

	/* ----------------------------------------------------------------------
	** Apply F3 to the result of F1 - prolonging the existing filter "branch"
	** ------------------------------------------------------------------- */

	if (SCALE_OUTPUT2_4 != 1) {
		arm_scale_f32(inputf32, SCALE_OUTPUT2_4,inputf32, size);
	}

	arm_biquad_cascade_df1_f32(F3, inputf32, inputf32, size);

	if (INVERT_OUTPUT2_4) {
		arm_negate_f32 (inputf32,inputf32, size);
	}

	/* ----------------------------------------------------------------------
	** Apply fractionnal delay lines to both lines
	** ------------------------------------------------------------------- */
	// delay on woofer
	delay_lines_run(D1, outputf32, outputf32, size);

	// delay on tweeter
	delay_lines_run(D2, inputf32, inputf32, size);


	/* ----------------------------------------------------------------------
	** Convert float results of both filter branches back to Q15
	** ------------------------------------------------------------------- */
	arm_float_to_q15(inputf32,  buffer_outputA, size);
	arm_float_to_q15(outputf32, buffer_outputB, size);

}

/* ----------------------------------------------------------------------
** Initialization of the filters structures
** There are 6 of them: 3 filters and Left/Right channels
** ------------------------------------------------------------------- */
void initFilter(void)
{
  
	/* Initialize the state and coefficient buffers for all Biquad sections */

	arm_biquad_cascade_df1_init_f32(&F1L, NUMSTAGES_FILTER1,
            &coeffTableFilter1[0],
			&biquadStatef32_F1L[0]);

	arm_biquad_cascade_df1_init_f32(&F1R, NUMSTAGES_FILTER1,
            &coeffTableFilter1[0],
			&biquadStatef32_F1R[0]);

	arm_biquad_cascade_df1_init_f32(&F2L, NUMSTAGES_FILTER2,
            &coeffTableFilter2[0],
			&biquadStatef32_F2L[0]);

	arm_biquad_cascade_df1_init_f32(&F2R, NUMSTAGES_FILTER2,
            &coeffTableFilter2[0],
			&biquadStatef32_F2R[0]);

	arm_biquad_cascade_df1_init_f32(&F3L, NUMSTAGES_FILTER3,
            &coeffTableFilter3[0],
			&biquadStatef32_F3L[0]);

	arm_biquad_cascade_df1_init_f32(&F3R, NUMSTAGES_FILTER3,
            &coeffTableFilter3[0],
			&biquadStatef32_F3R[0]);
	// Init fractional delay lines
	delay_lines_init(delay_lines, delays);

}
