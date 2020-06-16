/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "gen_code_legs/solo_autocollision_1pair.c"
#include "gen_code_legs/solo_autocollision_2pairs.c"
#include "gen_code_legs/solo_autocollision_3pairs.c"
#include "gen_code_legs/solo_autocollision_4pairs.c"
#include "gen_code_legs/solo_autocollision_5pairs.c"
#include "gen_code_legs/solo_autocollision_6pairs.c"
#include "gen_code_legs/solo_autocollision_7pairs.c"
#include "gen_code_legs/solo_autocollision_8pairs.c"
#include "gen_code_legs/solo_autocollision_9pairs.c"
#include "gen_code_legs/solo_autocollision_10pairs.c"
#include "gen_code_legs/solo_autocollision_11pairs.c"
#include "gen_code_legs/solo_autocollision_12pairs.c"
#include "gen_code_legs/solo_autocollision_13pairs.c"
#include "gen_code_legs/solo_autocollision_14pairs.c"
#include "gen_code_legs/solo_autocollision_15pairs.c"
#include "gen_code_legs/solo_autocollision_16pairs.c"
#include "gen_code_legs/solo_autocollision_17pairs.c"
#include "gen_code_legs/solo_autocollision_18pairs.c"
#include "gen_code_legs/solo_autocollision_19pairs.c"
#include "gen_code_legs/solo_autocollision_20pairs.c"
#include "gen_code_legs/solo_autocollision_21pairs.c"
#include "gen_code_legs/solo_autocollision_22pairs.c"
#include "gen_code_legs/solo_autocollision_23pairs.c"
#include "gen_code_legs/solo_autocollision_24pairs.c"



void dist_check_FLL_HLL(const float x[], float y[]);

void dist_check_FLL_HLL(const float x[], float y[])
{	
	float v[29];

	v[0] = cosf(x[1]);
	v[1] = cosf(x[2]);
	v[2] = sinf(x[1]);
	v[3] = sinf(x[2]);
	v[4] = 0 - v[3];
	v[5] = v[0] * v[1] + v[2] * v[4];
	v[6] = 0.1946 + -0.16 * v[2];
	v[7] = sinf(x[0]);
	v[8] = 0 - v[7];
	v[9] = 0 - v[2];
	v[10] = v[8] * v[9];
	v[8] = v[8] * v[0];
	v[11] = v[10] * v[1] + v[8] * v[4];
	v[12] = cosf(x[0]);
	v[13] = 0.0875 + 0.014 * v[12] + 0.03745 * v[12] + -0.16 * v[8];
	v[9] = v[12] * v[9];
	v[14] = v[12] * v[0];
	v[4] = v[9] * v[1] + v[14] * v[4];
	v[15] = 0.014 * v[7] + 0.03745 * v[7] + -0.16 * v[14];
	v[16] = sinf(x[7]);
	v[17] = -0.1946 + -0.16 * v[16];
	v[18] = cosf(x[6]);
	v[19] = sinf(x[6]);
	v[20] = 0 - v[19];
	v[21] = cosf(x[7]);
	v[22] = v[20] * v[21];
	v[23] = 0.0875 + 0.014 * v[18] + 0.03745 * v[18] + -0.16 * v[22];
	v[24] = v[18] * v[21];
	v[19] = 0.014 * v[19] + 0.03745 * v[19] + -0.16 * v[24];
	v[25] = (0 - v[5]) * v[6] + (0 - v[11]) * v[13] + (0 - v[4]) * v[15] + v[5] * v[17] + v[11] * v[23] + v[4] * v[19];
	v[26] = 0 - v[25];
	v[27] = sinf(x[8]);
	v[28] = cosf(x[8]);
	v[21] = v[21] * v[27] + v[16] * v[28];
	v[16] = 0 - v[16];
	v[22] = v[20] * v[16] * v[27] + v[22] * v[28];
	v[16] = v[18] * v[16] * v[27] + v[24] * v[28];
	v[25] = -0.2 * (v[5] * v[21] + v[11] * v[22] + v[4] * v[16]) + v[25] - v[25];
	v[4] = (0 - v[12]) * v[13] + (0 - v[7]) * v[15] + v[12] * v[23] + v[7] * v[19];
	v[12] = -0.2 * (v[12] * v[22] + v[7] * v[16]) + v[4] - v[4];
	v[2] = v[0] * v[3] + v[2] * v[1];
	v[8] = v[10] * v[3] + v[8] * v[1];
	v[14] = v[9] * v[3] + v[14] * v[1];
	v[19] = (0 - v[2]) * v[6] + (0 - v[8]) * v[13] + (0 - v[14]) * v[15] + v[2] * v[17] + v[8] * v[23] + v[14] * v[19];
	v[14] = -0.2 * (v[2] * v[21] + v[8] * v[22] + v[14] * v[16]) + v[19] - v[19];
	v[8] = v[25] * v[25] + v[12] * v[12] + v[14] * v[14];
	v[2] = -0.2 * v[14];
	v[16] = 0.04 * v[8] - v[2] * v[2];
	v[4] = 0 - v[4];
	v[19] = 0 - v[19];
	v[22] = v[25] * v[26] + v[12] * v[4] + v[14] * v[19];
	v[21] = -0.2 * v[19];
	if( v[16] < 1e-09 ) {
	  v[23] = 0;
	} else {
	  v[23] = v[2] * v[22] - v[8] * v[21];
	}
	if( v[16] < 1e-09 ) {
	  v[17] = 1;
	} else {
	  v[17] = v[16];
	}
	if( v[16] < 1e-09 ) {
	  v[15] = v[22];
	} else {
	  v[15] = 0.04 * v[22] - v[2] * v[21];
	}
	if( v[23] < 0 ) {
	  v[15] = v[22];
	} else {
	  v[15] = v[15];
	}
	if( v[23] > v[17] ) {
	  v[15] = v[22] + v[2];
	} else {
	  v[15] = v[15];
	}
	if( v[15] < 0 ) {
	  v[22] = 0;
	} else {
	  v[22] = v[15];
	}
	if( v[16] < 1e-09 ) {
	  v[16] = v[8];
	} else {
	  v[16] = v[16];
	}
	if( v[23] < 0 ) {
	  v[16] = v[8];
	} else {
	  v[16] = v[16];
	}
	if( v[23] > v[17] ) {
	  v[16] = v[8];
	} else {
	  v[16] = v[16];
	}
	if( v[22] > v[16] ) {
	  v[22] = v[16];
	} else {
	  v[22] = v[22];
	}
	if( fabs(v[22]) < 1e-09 ) {
	  v[22] = 0;
	} else {
	  v[22] = v[22] / v[16];
	}
	v[25] = v[26] - v[22] * v[25];
	v[4] = v[4] - v[22] * v[12];
	if( 0 - v[21] > 0.04 ) {
	  v[12] = v[17];
	} else {
	  v[12] = 0 - v[21];
	}
	if( 0 - v[21] < 0 ) {
	  v[12] = 0;
	} else {
	  v[12] = v[12];
	}
	if( 0 - v[21] + v[2] > 0.04 ) {
	  v[26] = v[17];
	} else {
	  v[26] = 0 - v[21] + v[2];
	}
	if( 0 - v[21] + v[2] < 0 ) {
	  v[26] = 0;
	} else {
	  v[26] = v[26];
	}
	if( v[23] < 0 ) {
	  v[23] = 0;
	} else {
	  v[23] = v[23];
	}
	if( v[23] > v[17] ) {
	  v[23] = v[17];
	} else {
	  v[23] = v[23];
	}
	if( v[15] > v[16] ) {
	  v[23] = v[26];
	} else {
	  v[23] = v[23];
	}
	if( v[15] < 0 ) {
	  v[23] = v[12];
	} else {
	  v[23] = v[23];
	}
	if( 0 - v[21] > 0.04 ) {
	  v[12] = v[17];
	} else {
	  v[12] = 0.04;
	}
	if( 0 - v[21] < 0 ) {
	  v[12] = v[17];
	} else {
	  v[12] = v[12];
	}
	if( 0 - v[21] + v[2] > 0.04 ) {
	  v[26] = v[17];
	} else {
	  v[26] = 0.04;
	}
	if( 0 - v[21] + v[2] < 0 ) {
	  v[26] = v[17];
	} else {
	  v[26] = v[26];
	}
	if( v[15] > v[16] ) {
	  v[26] = v[26];
	} else {
	  v[26] = v[17];
	}
	if( v[15] < 0 ) {
	  v[26] = v[12];
	} else {
	  v[26] = v[26];
	}
	if( fabs(v[23]) < 1e-09 ) {
	  v[26] = 0;
	} else {
	  v[26] = v[23] / v[26];
	}
	v[26] = v[19] + -0.2 * v[26] - v[22] * v[14];
	y[0] = sqrt(v[25] * v[25] + v[4] * v[4] + v[26] * v[26]) - 0.04;
}

void app_main(void)
{
	float test_config[12];
	float test_y[64];
	int64_t duration;
	int64_t start_time;
	int64_t end_time;
	float a = 3.14;
	
	float tot_duration = 0;
	float var_duration = 0;
	int counter = 0;
	int counterOff = 0;

	while(true)
	{
		test_config[0] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[1] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[2] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[3] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[4] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[5] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[6] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[7] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[8] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[9] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[10] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		test_config[11] = -a + 2*((float)rand()/(float)(RAND_MAX)) * a;
		
		
		start_time = esp_timer_get_time();
		solo_autocollision_legs_24pairs(test_config, test_y);
		end_time = esp_timer_get_time();
		

		/*start_time = esp_timer_get_time();
		for(int i=0; i<1; i++)
		{
			dist_check_FLL_HLL(test_config, test_y);
		}		
		end_time = esp_timer_get_time();*/

		duration = end_time - start_time;
		
		if(counter>5){
			counterOff++;
			tot_duration = tot_duration + duration*1.;
			var_duration = var_duration + (duration*1. - tot_duration*1./counterOff)*(duration*1. - tot_duration*1./counterOff);
		}
		
		if(counterOff%100 == 99)
		{
			printf("D : %lld, Avg : %f, StdDev : %F )\n", duration, tot_duration/counterOff, sqrt(var_duration/counterOff));
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
		counter++;
		
	}
}

