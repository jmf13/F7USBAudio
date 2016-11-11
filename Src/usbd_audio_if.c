/**
  ******************************************************************************
  * @file    USB_Device/AUDIO_Standalone/Src/usbd_audio_if.c
  * @author  MCD Application Team
  * @version jmf
  * @date    09-08-2016
  * @brief   USB Device Audio interface file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2016 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/

#include "usbd_audio_if.h"
#include "dsp.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  PLAYER_STARTED = 1,
  PLAYER_STOPPING,
  PLAYER_STOPPED
}PLAYER_STATE_TypeDef;

typedef enum
{
	NEXT_BUFFER_0 = 0,
	NEXT_BUFFER_1,
	NEXT_BUFFER_NO
}NEXT_BUFFER_TypeDef;

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static int8_t Audio_Init(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t Audio_DeInit(uint32_t options);
static int8_t Audio_PlaybackCmd(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t Audio_VolumeCtl(uint8_t vol);
static int8_t Audio_MuteCtl(uint8_t cmd);
static int8_t Audio_PeriodicTC(uint8_t cmd);
static int8_t Audio_GetState(void);

void fill_buffer (int buffer, uint8_t *pbuf, uint32_t size);

/* Private variables ---------------------------------------------------------*/


USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops = {
  Audio_Init,
  Audio_DeInit,
  Audio_PlaybackCmd,
  Audio_VolumeCtl,
  Audio_MuteCtl,
  Audio_PeriodicTC,
  Audio_GetState,
};

// State of the player
volatile PLAYER_STATE_TypeDef player_state = PLAYER_STOPPED;

/* Private functions ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
//?? Test with Word and word transfer
//int16_t Audio_output_buffer[AUDIO_OUTPUT_BUF_SIZE * 2];  // This represents two buffers in ping pong arrangement stereo samples
int32_t Audio_output_buffer[AUDIO_OUTPUT_BUF_SIZE * 2];  // This represents two buffers in ping pong arrangement stereo samples
//??int16_t Audio_buffer_L[AUDIO_OUTPUT_BUF_SIZE/2]; //one ping pong buffer, Left channel
//??int16_t Audio_buffer_R[AUDIO_OUTPUT_BUF_SIZE/2]; //one ping pong buffer, Right channel
int16_t Audio_buffer_1[AUDIO_OUTPUT_BUF_SIZE/2]; //one ping pong buffer, Channel 1
int16_t Audio_buffer_2[AUDIO_OUTPUT_BUF_SIZE/2]; //one ping pong buffer, Channel 2
int16_t Audio_buffer_3[AUDIO_OUTPUT_BUF_SIZE/2]; //one ping pong buffer, Channel 3
int16_t Audio_buffer_4[AUDIO_OUTPUT_BUF_SIZE/2]; //one ping pong buffer, Channel 4

extern USBD_HandleTypeDef hUsbDeviceFS;
extern SAI_HandleTypeDef hsai_BlockA1;
//??
//extern SAI_HandleTypeDef hsai_BlockB1;

//?? Be careful: the USBDbuffer is bytes and not int16_t
// int16_t Audio_input_ring_buffer[AUDIO_INPUT_BUFF_SIZE]; // We want to work between 1/3 and 2/3 buffer, pulling AUDIO_OUTPUT_BUFF_SIZE at a time


//?? No need anymore of Audio_input_ring_buffer_length, replaced by buffer in USBD-Audio 
// static volatile int Audio_input_ring_buffer_length; // ## check if static relevant or not
//static volatile int Audio_input_ring_buffer_start;
//static volatile int Audio_input_ring_buffer_end;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
//?? check if volume usefull
// static uint8_t Volume = 70;

// variables to keep as global variables the information about the buffer 
// pulled from the usbd_audio

volatile uint8_t* pbuf_input;
volatile uint32_t buf_input_size;

// Size of the output buffer
volatile uint32_t Audio_output_buffer_size =0;

//variable for next output buffer to write - 2 means no buffer to fill (wait state)
volatile NEXT_BUFFER_TypeDef next_buff = NEXT_BUFFER_NO;

//??
HAL_StatusTypeDef ErrorCode;

// Endless loop that fills the buffers when needed
// this procedure is called by main() once initialization is finished
void Audio_Loop (void)
{
	
	while(1){

      /*if(player_state == PLAYER_STARTED) {
    	  BSP_LED_On(LED2);
      } else {
    	  BSP_LED_Off(LED2);
      }*/
	  switch(next_buff) {
  
  	    case NEXT_BUFFER_0:
  	      fill_buffer (0, pbuf_input, buf_input_size);
	      next_buff = NEXT_BUFFER_NO;
	      break;

	    case NEXT_BUFFER_1:
	      fill_buffer (1, pbuf_input, buf_input_size);
	      next_buff = NEXT_BUFFER_NO;
	      break;
    
	    case NEXT_BUFFER_NO:
	      // do nothing
	      break;
  	  }
	}
}

/**
  * @brief  Initializes the AUDIO media low layer.
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  options: Reserved for future use 
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_Init(uint32_t  AudioFreq, uint32_t Volume, uint32_t options)
{
//?? Test purpose
  //BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, AUDIO_DEFAULT_VOLUME, USBD_AUDIO_FREQ);
  return 0;
}

/**
  * @brief  De-Initializes the AUDIO media low layer.      
  * @param  options: Reserved for future use
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_DeInit(uint32_t options)
{
	//?? To adapt to SAI interfaces
	//BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_ON);
	//BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
	//BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_OFF);
  return 0;
}

/**
  * @brief  Handles AUDIO command.        
  * @param  pbuf: Pointer to buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: Command opcode
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_PlaybackCmd(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  
  int i =0;

  switch(cmd)
  {
  //Called by usbd_audio when the input buffer is ready, to start the music playing
  case AUDIO_CMD_START:
	    // init filters
	    //##BSP_LED_Toggle (LED3);
	  	initFilter();

		// fills first half of the output buffer with zeros, playing will start with half buffer of zeros
		for(i=0; i<AUDIO_OUTPUT_BUF_SIZE; i++){
			 Audio_output_buffer[i]= 0;
		}

	  	// Size in bytes, for a complete buffer
		//?? To Adapt to SAI
		//?? BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_OFF);
	  	//?? BSP_AUDIO_OUT_Play((uint16_t *)&Audio_output_buffer, AUDIO_OUTPUT_BUF_SIZE*4);
		ErrorCode = HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)Audio_output_buffer, AUDIO_OUTPUT_BUF_SIZE*2);
		if( ErrorCode != 0){
		    Error_Handler();
		}
		// we then wait for a DMA event of DMA half transferred
	  	// fill second half of the audio_output_buffer, first half will start with 0000s
	  	//Pull_Data from USBD_Audio - note that data are provided by USBD_Audio using the Audio_PlaybackCmd callback
	    Audio_output_buffer_size = 0;

	    USBD_AUDIO_DataPull (&hUsbDeviceFS);
	    next_buff = NEXT_BUFFER_1;
	  	player_state = PLAYER_STARTED;
    break;


  // called by USBD_Audio in the pull_data callback => we save the infos related to the buffer ready: pointer and size
  case AUDIO_CMD_PLAY:
    pbuf_input = pbuf;
    buf_input_size = size/2;
    //##BSP_LED_Toggle (LED6);
    break;

  case AUDIO_CMD_STOP:
      pbuf_input = pbuf;
      buf_input_size = size/2;
      player_state = PLAYER_STOPPING;
      //##BSP_LED_Toggle (LED5);
      break;


  }
  return 0;
}

/**
  * @brief  Controls AUDIO Volume.             
  * @param  vol: Volume level (0..100)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */

//?? Who sets the volume?
static int8_t Audio_VolumeCtl(uint8_t vol)
{
    //?? To adapt to SAI
	//?? BSP_AUDIO_OUT_SetVolume(vol);
  return 0;
}

/**
  * @brief  Controls AUDIO Mute.              
  * @param  cmd: Command opcode
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_MuteCtl(uint8_t cmd)
{
	//?? To adapt to SAI
	//?? BSP_AUDIO_OUT_SetMute(cmd);
  return 0;
}

/**
  * @brief  Audio_PeriodicTC              
  * @param  cmd: Command opcode
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_PeriodicTC(uint8_t cmd)
{
  return 0;
}

/**
  * @brief  Gets AUDIO State.              
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t Audio_GetState(void)
{
  return 0;
}

/**
  * @brief  Manages the DMA full Transfer complete event.
  * @param  None
  * @retval None
  */
//?? To adapt to SAI
//?? void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{

 switch(player_state)
  {
    case PLAYER_STARTED:
	  //start playing the prepared buffer
    	//?? To adapt to SAI
      //?? BSP_AUDIO_OUT_ChangeBuffer((uint16_t *) &Audio_output_buffer,  AUDIO_OUTPUT_BUF_SIZE*2);
      //if( HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)Audio_output_buffer, AUDIO_OUTPUT_BUF_SIZE*2)!= 0){
      if( HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)Audio_output_buffer, AUDIO_OUTPUT_BUF_SIZE*2)!= 0){
    	Error_Handler();
      }

	  //Pull_Data from USBD_Audio - note that data are provided by USBD_Audio using the Audio_PlaybackCmd callback
	  USBD_AUDIO_DataPull (&hUsbDeviceFS);
	  next_buff = NEXT_BUFFER_1;
	  Audio_output_buffer_size = 0;
      break;
    //?? See how to fill buffer with zeros to complement the incomplete buffer
    case PLAYER_STOPPING:
        //Start playing the prepared buffer before being stopped
    	//?? To adapt to SAI
    	//??BSP_AUDIO_OUT_ChangeBuffer((uint16_t *) &Audio_output_buffer, Audio_output_buffer_size);

    	//?? to test length on constant
    	//if( HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)Audio_output_buffer, Audio_output_buffer_size)!= 0){
    	if( HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *)Audio_output_buffer, AUDIO_OUTPUT_BUF_SIZE*2)!= 0){
    	    Error_Handler();
    	}
	    Audio_output_buffer_size = 0;
	    //we don't pull data, set next_buffer to no filling, and stop the player
	    next_buff = NEXT_BUFFER_NO;

	    //?? See how to fill buffer with zeros to complement the incomplete buffer
	    player_state = PLAYER_STOPPED;
        break;

    case PLAYER_STOPPED:
    	//?? To adapt SAI
    	//?? BSP_AUDIO_OUT_SetMute(AUDIO_MUTE_ON);
      break;
   }
}

/**
  * @brief  Manages the DMA Half Transfer complete event.
  * @param  None
  * @retval None
  */
//?? To adapt to SAI
//?? void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{ 
switch(player_state)
  {
    case PLAYER_STARTED:
	//Pull_Data from USBD_Audio - note that data are provided by USBD_Audio using the Audio_PlaybackCmd callback
	USBD_AUDIO_DataPull (&hUsbDeviceFS);
	next_buff = NEXT_BUFFER_0;
      break;

    case PLAYER_STOPPING:
    	// We do nothing
      break;

    case PLAYER_STOPPED:
      // We do nothing
      break;
   }
}

//size in int_16
void fill_buffer (int buffer, uint8_t *pbuf, uint32_t size) // buffer=0 for first half of the buffer buffer = 1 for second half
{
	int i=0;
	// This is needed to translate the bytes buffer from usbd_audio in int_16 music data
	uint16_t * pbuf_uint16 = (uint16_t *)pbuf;

	   // The size is in bytes, we need to read 2xInt16 at each loop exec
	   // So the loop has to be performed on size/2
	   for(i=0; i<size/2; i++){
			Audio_buffer_1[i]= *pbuf_uint16++;
			Audio_buffer_3[i]= *pbuf_uint16++;

		}

		//if user button pressed, we apply the DSP
		//## add the init of the pushbutton
		/*if (BSP_PB_GetState(BUTTON_KEY)== 1) {
				BSP_LED_On(LED3);
				dsp((int16_t*)&Audio_buffer_L[0], size/2, 0);
				dsp((int16_t*)&Audio_buffer_R[0], size/2, 1);
		} else {
			BSP_LED_Off(LED3);
		}*/
		// Build stereo Audio_output_buffer from Audio_buffer_L and Audio_buffer_R, filling the requested
		// ping pong buffer: first half offset 0 or second half offset AUDIO_OUTPUT_BUFF_SIZE

	    dsp((int16_t*)&Audio_buffer_1[0], (int16_t*)&Audio_buffer_1[0], (int16_t*)&Audio_buffer_2[0], size/2, 0);
	   	//dsp((int16_t*)&Audio_buffer_3[0], (int16_t*)&Audio_buffer_3[0], (int16_t*)&Audio_buffer_4[0], size/2, 1);

	    if (size != AUDIO_OUTPUT_BUF_SIZE){
	    	BSP_LED_Toggle(LED3);
	    }

	    for(i=0; i<size/2; i++){
			 Audio_output_buffer[AUDIO_OUTPUT_BUF_SIZE*buffer+2*i]= (((int32_t)Audio_buffer_1[i]) <<8); /*Left Channel*/
			 Audio_output_buffer[AUDIO_OUTPUT_BUF_SIZE*buffer+2*i + 1]= (((int32_t)Audio_buffer_2[i]) <<8); /*Right Channel*/
		}

		// if the buffer to fill is the 2nd half and it is an incomplete buffer
		// then we have to fill with zeros as the playback has already started and is difficult to stop
		if ((buffer==1)&&(size < AUDIO_OUTPUT_BUF_SIZE )){
			for(i=size; i<AUDIO_OUTPUT_BUF_SIZE; i++){
				Audio_output_buffer[AUDIO_OUTPUT_BUF_SIZE*buffer+i]= 0;
					    }
		}

		Audio_output_buffer_size = Audio_output_buffer_size + size;

}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
