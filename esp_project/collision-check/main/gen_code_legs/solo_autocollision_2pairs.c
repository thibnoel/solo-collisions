#include <math.h>
#include <stdio.h>

void solo_autocollision_legs_2pairs(const float x[], float y[]) {

   // auxiliary variables
   float  v[33];

   v[0] = cosf(x[1]);
   v[1] = sinf(x[4]);
   v[2] = sinf(x[0]);
   v[3] = 0 - v[2];
   v[4] = sinf(x[1]);
   v[5] = 0 - v[4];
   v[6] = v[3] * v[5];
   v[7] = sinf(x[3]);
   v[8] = 0 - v[7];
   v[9] = cosf(x[4]);
   v[10] = v[8] * v[9];
   v[11] = cosf(x[0]);
   v[5] = v[11] * v[5];
   v[12] = cosf(x[3]);
   v[13] = v[12] * v[9];
   v[14] = v[0] * v[1] + v[6] * v[10] + v[5] * v[13];
   v[15] = 0.0875 + 0.014 * v[11];
   v[16] = 0.014 * v[2];
   v[17] = -0.0875 + -0.014 * v[12];
   v[18] = -0.014 * v[7];
   v[19] = 0.1946 * (0 - v[0]) + (0 - v[6]) * v[15] + (0 - v[5]) * v[16] + 0.1946 * v[0] + v[6] * v[17] + v[5] * v[18];
   v[20] = 0.1 * v[14] + v[19];
   v[21] = 0 - v[20];
   v[20] = -0.1 * v[14] + v[19] - v[20];
   v[19] = v[11] * v[10] + v[2] * v[13];
   v[14] = (0 - v[11]) * v[15] + (0 - v[2]) * v[16] + v[11] * v[17] + v[2] * v[18];
   v[22] = 0.1 * v[19] + v[14];
   v[14] = -0.1 * v[19] + v[14] - v[22];
   v[3] = v[3] * v[0];
   v[19] = v[11] * v[0];
   v[23] = v[4] * v[1] + v[3] * v[10] + v[19] * v[13];
   v[24] = 0.1946 * (0 - v[4]) + (0 - v[3]) * v[15] + (0 - v[19]) * v[16] + 0.1946 * v[4] + v[3] * v[17] + v[19] * v[18];
   v[25] = 0.1 * v[23] + v[24];
   v[24] = -0.1 * v[23] + v[24] - v[25];
   v[23] = v[20] * v[20] + v[14] * v[14] + v[24] * v[24];
   v[26] = -0.2 * v[24];
   v[27] = 0.04 * v[23] - v[26] * v[26];
   v[22] = 0 - v[22];
   v[25] = 0.1 - v[25];
   v[28] = v[20] * v[21] + v[14] * v[22] + v[24] * v[25];
   v[29] = -0.2 * v[25];
   if( v[27] < 1e-09 ) {
      v[30] = 0;
   } else {
      v[30] = v[26] * v[28] - v[23] * v[29];
   }
   if( v[27] < 1e-09 ) {
      v[31] = 1;
   } else {
      v[31] = v[27];
   }
   if( v[27] < 1e-09 ) {
      v[32] = v[28];
   } else {
      v[32] = 0.04 * v[28] - v[26] * v[29];
   }
   if( v[30] < 0 ) {
      v[32] = v[28];
   } else {
      v[32] = v[32];
   }
   if( v[30] > v[31] ) {
      v[32] = v[28] + v[26];
   } else {
      v[32] = v[32];
   }
   if( v[32] < 0 ) {
      v[28] = 0;
   } else {
      v[28] = v[32];
   }
   if( v[27] < 1e-09 ) {
      v[27] = v[23];
   } else {
      v[27] = v[27];
   }
   if( v[30] < 0 ) {
      v[27] = v[23];
   } else {
      v[27] = v[27];
   }
   if( v[30] > v[31] ) {
      v[27] = v[23];
   } else {
      v[27] = v[27];
   }
   if( v[28] > v[27] ) {
      v[28] = v[27];
   } else {
      v[28] = v[28];
   }
   if( fabs(v[28]) < 1e-09 ) {
      v[28] = 0;
   } else {
      v[28] = v[28] / v[27];
   }
   v[20] = v[21] - v[28] * v[20];
   v[22] = v[22] - v[28] * v[14];
   if( 0 - v[29] > 0.04 ) {
      v[14] = v[31];
   } else {
      v[14] = 0 - v[29];
   }
   if( 0 - v[29] < 0 ) {
      v[14] = 0;
   } else {
      v[14] = v[14];
   }
   if( 0 - v[29] + v[26] > 0.04 ) {
      v[21] = v[31];
   } else {
      v[21] = 0 - v[29] + v[26];
   }
   if( 0 - v[29] + v[26] < 0 ) {
      v[21] = 0;
   } else {
      v[21] = v[21];
   }
   if( v[30] < 0 ) {
      v[30] = 0;
   } else {
      v[30] = v[30];
   }
   if( v[30] > v[31] ) {
      v[30] = v[31];
   } else {
      v[30] = v[30];
   }
   if( v[32] > v[27] ) {
      v[30] = v[21];
   } else {
      v[30] = v[30];
   }
   if( v[32] < 0 ) {
      v[30] = v[14];
   } else {
      v[30] = v[30];
   }
   if( 0 - v[29] > 0.04 ) {
      v[14] = v[31];
   } else {
      v[14] = 0.04;
   }
   if( 0 - v[29] < 0 ) {
      v[14] = v[31];
   } else {
      v[14] = v[14];
   }
   if( 0 - v[29] + v[26] > 0.04 ) {
      v[21] = v[31];
   } else {
      v[21] = 0.04;
   }
   if( 0 - v[29] + v[26] < 0 ) {
      v[21] = v[31];
   } else {
      v[21] = v[21];
   }
   if( v[32] > v[27] ) {
      v[21] = v[21];
   } else {
      v[21] = v[31];
   }
   if( v[32] < 0 ) {
      v[21] = v[14];
   } else {
      v[21] = v[21];
   }
   if( fabs(v[30]) < 1e-09 ) {
      v[21] = 0;
   } else {
      v[21] = v[30] / v[21];
   }
   v[21] = v[25] + -0.2 * v[21] - v[28] * v[24];
   y[0] = sqrt(v[20] * v[20] + v[22] * v[22] + v[21] * v[21]) - 0.032;
   v[21] = sinf(x[5]);
   v[22] = cosf(x[5]);
   v[9] = v[9] * v[21] + v[1] * v[22];
   v[20] = 0 - v[1];
   v[8] = v[8] * v[20] * v[21] + v[10] * v[22];
   v[20] = v[12] * v[20] * v[21] + v[13] * v[22];
   v[22] = v[0] * v[9] + v[6] * v[8] + v[5] * v[20];
   v[1] = 0.1946 + -0.16 * v[1];
   v[17] = v[17] + -0.03745 * v[12] + -0.16 * v[10];
   v[18] = v[18] + -0.03745 * v[7] + -0.16 * v[13];
   v[5] = 0.1946 * (0 - v[0]) + (0 - v[6]) * v[15] + (0 - v[5]) * v[16] + v[0] * v[1] + v[6] * v[17] + v[5] * v[18];
   v[6] = 0.07 * v[22] + v[5];
   v[0] = 0 - v[6];
   v[6] = -0.07 * v[22] + v[5] - v[6];
   v[5] = v[11] * v[8] + v[2] * v[20];
   v[11] = (0 - v[11]) * v[15] + (0 - v[2]) * v[16] + v[11] * v[17] + v[2] * v[18];
   v[2] = 0.07 * v[5] + v[11];
   v[11] = -0.07 * v[5] + v[11] - v[2];
   v[20] = v[4] * v[9] + v[3] * v[8] + v[19] * v[20];
   v[18] = 0.1946 * (0 - v[4]) + (0 - v[3]) * v[15] + (0 - v[19]) * v[16] + v[4] * v[1] + v[3] * v[17] + v[19] * v[18];
   v[17] = 0.07 * v[20] + v[18];
   v[18] = -0.07 * v[20] + v[18] - v[17];
   v[20] = v[6] * v[6] + v[11] * v[11] + v[18] * v[18];
   v[1] = -0.2 * v[18];
   v[19] = 0.04 * v[20] - v[1] * v[1];
   v[2] = 0 - v[2];
   v[17] = 0.1 - v[17];
   v[3] = v[6] * v[0] + v[11] * v[2] + v[18] * v[17];
   v[16] = -0.2 * v[17];
   if( v[19] < 1e-09 ) {
      v[15] = 0;
   } else {
      v[15] = v[1] * v[3] - v[20] * v[16];
   }
   if( v[19] < 1e-09 ) {
      v[4] = 1;
   } else {
      v[4] = v[19];
   }
   if( v[19] < 1e-09 ) {
      v[8] = v[3];
   } else {
      v[8] = 0.04 * v[3] - v[1] * v[16];
   }
   if( v[15] < 0 ) {
      v[8] = v[3];
   } else {
      v[8] = v[8];
   }
   if( v[15] > v[4] ) {
      v[8] = v[3] + v[1];
   } else {
      v[8] = v[8];
   }
   if( v[8] < 0 ) {
      v[3] = 0;
   } else {
      v[3] = v[8];
   }
   if( v[19] < 1e-09 ) {
      v[19] = v[20];
   } else {
      v[19] = v[19];
   }
   if( v[15] < 0 ) {
      v[19] = v[20];
   } else {
      v[19] = v[19];
   }
   if( v[15] > v[4] ) {
      v[19] = v[20];
   } else {
      v[19] = v[19];
   }
   if( v[3] > v[19] ) {
      v[3] = v[19];
   } else {
      v[3] = v[3];
   }
   if( fabs(v[3]) < 1e-09 ) {
      v[3] = 0;
   } else {
      v[3] = v[3] / v[19];
   }
   v[6] = v[0] - v[3] * v[6];
   v[2] = v[2] - v[3] * v[11];
   if( 0 - v[16] > 0.04 ) {
      v[11] = v[4];
   } else {
      v[11] = 0 - v[16];
   }
   if( 0 - v[16] < 0 ) {
      v[11] = 0;
   } else {
      v[11] = v[11];
   }
   if( 0 - v[16] + v[1] > 0.04 ) {
      v[0] = v[4];
   } else {
      v[0] = 0 - v[16] + v[1];
   }
   if( 0 - v[16] + v[1] < 0 ) {
      v[0] = 0;
   } else {
      v[0] = v[0];
   }
   if( v[15] < 0 ) {
      v[15] = 0;
   } else {
      v[15] = v[15];
   }
   if( v[15] > v[4] ) {
      v[15] = v[4];
   } else {
      v[15] = v[15];
   }
   if( v[8] > v[19] ) {
      v[15] = v[0];
   } else {
      v[15] = v[15];
   }
   if( v[8] < 0 ) {
      v[15] = v[11];
   } else {
      v[15] = v[15];
   }
   if( 0 - v[16] > 0.04 ) {
      v[11] = v[4];
   } else {
      v[11] = 0.04;
   }
   if( 0 - v[16] < 0 ) {
      v[11] = v[4];
   } else {
      v[11] = v[11];
   }
   if( 0 - v[16] + v[1] > 0.04 ) {
      v[0] = v[4];
   } else {
      v[0] = 0.04;
   }
   if( 0 - v[16] + v[1] < 0 ) {
      v[0] = v[4];
   } else {
      v[0] = v[0];
   }
   if( v[8] > v[19] ) {
      v[0] = v[0];
   } else {
      v[0] = v[4];
   }
   if( v[8] < 0 ) {
      v[0] = v[11];
   } else {
      v[0] = v[0];
   }
   if( fabs(v[15]) < 1e-09 ) {
      v[0] = 0;
   } else {
      v[0] = v[15] / v[0];
   }
   v[0] = v[17] + -0.2 * v[0] - v[3] * v[18];
   y[1] = sqrt(v[6] * v[6] + v[2] * v[2] + v[0] * v[0]) - 0.031;
}

