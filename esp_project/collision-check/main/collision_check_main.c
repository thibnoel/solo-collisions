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

void dist_check_FLL_HLL(float x[], float y[]);

void dist_check_FLL_HLL(float x[], float y[])
{	
	float v[29];

	v[0] = cos(x[1]);
	v[1] = cos(x[2]);
	v[2] = sin(x[1]);
	v[3] = sin(x[2]);
	v[4] = 0 - v[3];
	v[5] = v[0] * v[1] + v[2] * v[4];
	v[6] = 0.1946 + -0.16 * v[2];
	v[7] = sin(x[0]);
	v[8] = 0 - v[7];
	v[9] = 0 - v[2];
	v[10] = v[8] * v[9];
	v[8] = v[8] * v[0];
	v[11] = v[10] * v[1] + v[8] * v[4];
	v[12] = cos(x[0]);
	v[13] = 0.0875 + 0.014 * v[12] + 0.03745 * v[12] + -0.16 * v[8];
	v[9] = v[12] * v[9];
	v[14] = v[12] * v[0];
	v[4] = v[9] * v[1] + v[14] * v[4];
	v[15] = 0.014 * v[7] + 0.03745 * v[7] + -0.16 * v[14];
	v[16] = sin(x[7]);
	v[17] = -0.1946 + -0.16 * v[16];
	v[18] = cos(x[6]);
	v[19] = sin(x[6]);
	v[20] = 0 - v[19];
	v[21] = cos(x[7]);
	v[22] = v[20] * v[21];
	v[23] = 0.0875 + 0.014 * v[18] + 0.03745 * v[18] + -0.16 * v[22];
	v[24] = v[18] * v[21];
	v[19] = 0.014 * v[19] + 0.03745 * v[19] + -0.16 * v[24];
	v[25] = (0 - v[5]) * v[6] + (0 - v[11]) * v[13] + (0 - v[4]) * v[15] + v[5] * v[17] + v[11] * v[23] + v[4] * v[19];
	v[26] = 0 - v[25];
	v[27] = sin(x[8]);
	v[28] = cos(x[8]);
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
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    // printf("Hello world!\n");
	int64_t start_time;
	int64_t end_time;
	int64_t duration = 0;
	float test_config[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	float test_y[1] = {0};

	start_time = esp_timer_get_time();
	for(int i=0; i<1000; i++)
	{
		dist_check_FLL_HLL(test_config, test_y);
	}
	end_time = esp_timer_get_time();

	duration = end_time-start_time;
    printf("Dist result : %f \n",test_y[0]);
	printf("Start time : %" PRId64 "\n", start_time);
	printf("End time : %" PRId64 "\n", end_time);
	printf("Avg. duration : %f \n", duration*0.001);
    printf("\n");

    /* Print chip information 
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Free heap: %d\n", esp_get_free_heap_size());
	*/
    for (int i = 5; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
