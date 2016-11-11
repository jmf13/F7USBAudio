/*
 * DSP.c
 *
 *  Created on: 19 juil. 2016
 *      Author: JMF
 */

#include "dsp.h"
#include "usbd_audio.h"
// arm c library includes
#include "stdbool.h"

// arm cmsis library includes <= added as symbols in project properties

#include "arm_math.h"
// #include "math_helper.h" //## check usage ???




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

#define SCALE_INPUT		1	//scaling factor applied to input buffer

#define NUMSTAGES_FILTER1	2	//number of biquads in this filter
float32_t coeffTableFilter1[5*NUMSTAGES_FILTER1] = {
  /* xxxxxx */
		1.003791623405750, -1.916261673869340, 0.966920997039797, 1.916261673869340, -0.970712620445551,  //PEQ 1800Hz Q=7 G = 2dB
		1.052140969066160, 0.931305193641266, 0.899631210773041, -0.931305193641266, -0.951772179839199	  //PEQ 15800Hz Q=10 G = 10dB
};



#define SCALE_OUTPUT1_3_dB		0	//scaling factor applied to output buffer 1 and 3 in dB
#define INVERT_OUTPUT1_3 	false

#define NUMSTAGES_FILTER2	4	//number of biquads in this filter
float32_t coeffTableFilter2[5*NUMSTAGES_FILTER2] = {
		0.001921697757295, 0.003843395514590, 0.001921697757295, 1.824651307057290, -0.832338098086468, 	//LR2 LowPass 700Hz
		1.003858302013990, -1.993727804265180, 0.989912205497575, 1.993727804265180, -0.993770507511564,	//PEQ 50Hz 7dB 0.7Q
		0.925455940386243, -1.431501433736160, 0.897376826164088, 1.431501433736160, -0.822832766550331,	//PEQ 5100Hz -16dB 8Q
		0.884138082603731, -1.204926343048180, 0.776971533616450, 1.204926343048180, -0.661109616220181	//PEQ 5800Hz -10dB 3Q
};


#define SCALE_OUTPUT2_4_dB		-9.6	//scaling factor applied to output buffers 2 and 4 in dB
#define INVERT_OUTPUT2_4 		true

#define NUMSTAGES_FILTER3	6	//number of biquads in this filter
float32_t coeffTableFilter3[5*NUMSTAGES_FILTER3] = {
  /* xxxxxx */
		0.914247351285939, -1.828494702571880, 0.914247351285939, 1.824651307057290, -0.832338098086468,	//LR2 HighPass 700Hz
		1.123687441779500, -1.824401268684300, 0.740517306108243, 1.841148795124730, -0.847457221447315,	//LowShelf 1000Hz 16dB 0.5Q
		0.964914620734976, -1.706046605406540, 0.844942702041952, 1.706046605406540, -0.809857322776928,	//PEQ 2600Hz -4dB 2Q
		1.825672979584440, -1.545668441710860, 0.543324542467308, 0.367563755077364, -0.190892835418248,	//HighShelf 8000Hz 8dB 0.7Q
		1.038044950607060, -0.469900579205428, 0.777511470363804, 0.469900579205428, -0.815556420970861,	//PEQ 10000Hz 3dB 4Q
		0.962297780890294, 0.262262826325308, 0.865411704698658, -0.262262826325308, -0.827709485588952	//PEQ 13100Hz -5dB 7Q
};

/* ----------------------------------------------------------------------
** End of definition of all the parameters for the filters and digital signal 
** processing to be performed.
** ------------------------------------------------------------------- */




#define SCALE_OUTPUT1_3	pow(10,SCALE_OUTPUT1_3_dB/20)
#define SCALE_OUTPUT2_4	pow(10,SCALE_OUTPUT2_4_dB/20)


//?? check if we can rely on size parameter or constant Local buffer size is AUDIO_OUTPUT_BUFF_SIZE/2
//#define BLOCKSIZE    512


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

	// ?? check if needed
	//float32_t *biquadStatef32;

	if (channel == 0){
		F1 = &F1L;
		F2 = &F2L;
		F3 = &F3L;
	}
	else
	{
		F1 = &F1R;
		F2 = &F2R;
		F3 = &F3R;
	};
	

	/* ----------------------------------------------------------------------
	** Convert block of input data from float to Q31
	** ------------------------------------------------------------------- */
	arm_q15_to_float(buffer_input, inputf32, size);
	
	/* ----------------------------------------------------------------------
	** Scale  by SCALE_INPUT.  This provides additional headroom so that the
	** graphic EQ can apply gain.
	** ?? This is to be checked
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
	arm_biquad_cascade_df1_f32(F3, inputf32, inputf32, size);

	if (SCALE_OUTPUT2_4 != 1) {
		arm_scale_f32(inputf32, SCALE_OUTPUT2_4,inputf32, size);
	}

	if (INVERT_OUTPUT2_4) {
		arm_negate_f32 (inputf32,inputf32, size);
	}


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

}
