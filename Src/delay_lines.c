/* delay_lines function, version 1.0
   Copyright 2016 JM. Fourneron, credits to Charlie Laub, GPLv3 */
#include <delay_lines.h>

/*
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  CREDITS:
  Main basis for this code is the mTAP LADSPA Plugin by Charlie Laub
  stripped down to only keep main delay lines
*/


static float32_t SR = 48000; //stores the sample rate of the data stream


void delay_lines_init(delay_lines_data_struct *delay_line, float32_t *delays) {
  unsigned int channel=0;
  unsigned int j;
  for (channel=0;channel<MAX_CHANNELS;channel++) {
    if (delays[channel] == 0.0) {
    	delay_line[channel].sd = 0.0;
    	delay_line[channel].idl = 0;
    } else {
      //convert user supplied delay time to delay in samples and find the minimum value over all channels
    	delay_line[channel].sd = delays[channel] * 0.001 * SR; //0.001 factor is because delay is in milliseconds
    	delay_line[channel].idl = (int)round(delay_line[channel].sd); //set delay line length
    }
  }

  //loop over all channels and initialize their storage
  for (channel=0;channel<MAX_CHANNELS;channel++) {
      for ( j=0; j<delay_line[channel].idl; j++) {
    	  delay_line[channel].delay_line[j] = 0; //intialize values to small non-zero signal
      }
    
      delay_line[channel].delay_insertion_index = 0; //initialize the delay_insertion_index
  } //done initializing storage

} //end delay_lines_init


static inline float32_t Deus_ex_machina(delay_lines_data_struct *delay_line, const float32_t input) {

  float32_t delay_line_input = input;

  //if no delay line used for this channel immediately return the delay line input:
  if (delay_line->idl == 0 ) return delay_line_input;
  //process the data through a delay line
  //move the delayed output from the delay line into delay_line_output
  const int delay_insertion_index = delay_line->delay_insertion_index;
  float32_t delay_line_output = delay_line->delay_line[delay_insertion_index];
  delay_line->delay_line[delay_insertion_index] = delay_line_input; //replace it with the delay_line_input
  //increment the delay_insertion_index, wrapping back to zero if necessary
  delay_line->delay_insertion_index = (delay_insertion_index + 1) % delay_line->idl;
  return delay_line_output; //return delay line output
}


// This function applies the delay process for size number of samples to buffer_input
// and delivers the result in buffer_output
void delay_lines_run(delay_lines_data_struct *instance,float32_t* buffer_input, float32_t* buffer_output, uint16_t size) {
  uint16_t pos;
  for (pos = 0; pos < size; pos++) {
	 // We apply the DSP process to each element of the input buffer
	  buffer_output[pos] = Deus_ex_machina(instance, buffer_input[pos]);
  }
} //end delay_lines_run.


