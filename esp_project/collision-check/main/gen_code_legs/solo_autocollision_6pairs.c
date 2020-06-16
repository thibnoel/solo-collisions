#include <math.h>
#include <stdio.h>

void solo_autocollision_legs_6pairs(const float x[], float y[]) {

   // auxiliary variables
   float v[39];

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
   v[21] = 0.1946 + -0.16 * v[1];
   v[12] = v[17] + -0.03745 * v[12] + -0.16 * v[10];
   v[7] = v[18] + -0.03745 * v[7] + -0.16 * v[13];
   v[28] = 0.1946 * (0 - v[0]) + (0 - v[6]) * v[15] + (0 - v[5]) * v[16] + v[0] * v[21] + v[6] * v[12] + v[5] * v[7];
   v[25] = 0.07 * v[22] + v[28];
   v[24] = 0 - v[25];
   v[25] = -0.07 * v[22] + v[28] - v[25];
   v[28] = v[11] * v[8] + v[2] * v[20];
   v[22] = (0 - v[11]) * v[15] + (0 - v[2]) * v[16] + v[11] * v[12] + v[2] * v[7];
   v[30] = 0.07 * v[28] + v[22];
   v[22] = -0.07 * v[28] + v[22] - v[30];
   v[28] = v[4] * v[9] + v[3] * v[8] + v[19] * v[20];
   v[14] = 0.1946 * (0 - v[4]) + (0 - v[3]) * v[15] + (0 - v[19]) * v[16] + v[4] * v[21] + v[3] * v[12] + v[19] * v[7];
   v[32] = 0.07 * v[28] + v[14];
   v[14] = -0.07 * v[28] + v[14] - v[32];
   v[28] = v[25] * v[25] + v[22] * v[22] + v[14] * v[14];
   v[27] = -0.2 * v[14];
   v[31] = 0.04 * v[28] - v[27] * v[27];
   v[30] = 0 - v[30];
   v[32] = 0.1 - v[32];
   v[29] = v[25] * v[24] + v[22] * v[30] + v[14] * v[32];
   v[26] = -0.2 * v[32];
   if( v[31] < 1e-09 ) {
      v[23] = 0;
   } else {
      v[23] = v[27] * v[29] - v[28] * v[26];
   }
   if( v[31] < 1e-09 ) {
      v[33] = 1;
   } else {
      v[33] = v[31];
   }
   if( v[31] < 1e-09 ) {
      v[34] = v[29];
   } else {
      v[34] = 0.04 * v[29] - v[27] * v[26];
   }
   if( v[23] < 0 ) {
      v[34] = v[29];
   } else {
      v[34] = v[34];
   }
   if( v[23] > v[33] ) {
      v[34] = v[29] + v[27];
   } else {
      v[34] = v[34];
   }
   if( v[34] < 0 ) {
      v[29] = 0;
   } else {
      v[29] = v[34];
   }
   if( v[31] < 1e-09 ) {
      v[31] = v[28];
   } else {
      v[31] = v[31];
   }
   if( v[23] < 0 ) {
      v[31] = v[28];
   } else {
      v[31] = v[31];
   }
   if( v[23] > v[33] ) {
      v[31] = v[28];
   } else {
      v[31] = v[31];
   }
   if( v[29] > v[31] ) {
      v[29] = v[31];
   } else {
      v[29] = v[29];
   }
   if( fabs(v[29]) < 1e-09 ) {
      v[29] = 0;
   } else {
      v[29] = v[29] / v[31];
   }
   v[25] = v[24] - v[29] * v[25];
   v[30] = v[30] - v[29] * v[22];
   if( 0 - v[26] > 0.04 ) {
      v[22] = v[33];
   } else {
      v[22] = 0 - v[26];
   }
   if( 0 - v[26] < 0 ) {
      v[22] = 0;
   } else {
      v[22] = v[22];
   }
   if( 0 - v[26] + v[27] > 0.04 ) {
      v[24] = v[33];
   } else {
      v[24] = 0 - v[26] + v[27];
   }
   if( 0 - v[26] + v[27] < 0 ) {
      v[24] = 0;
   } else {
      v[24] = v[24];
   }
   if( v[23] < 0 ) {
      v[23] = 0;
   } else {
      v[23] = v[23];
   }
   if( v[23] > v[33] ) {
      v[23] = v[33];
   } else {
      v[23] = v[23];
   }
   if( v[34] > v[31] ) {
      v[23] = v[24];
   } else {
      v[23] = v[23];
   }
   if( v[34] < 0 ) {
      v[23] = v[22];
   } else {
      v[23] = v[23];
   }
   if( 0 - v[26] > 0.04 ) {
      v[22] = v[33];
   } else {
      v[22] = 0.04;
   }
   if( 0 - v[26] < 0 ) {
      v[22] = v[33];
   } else {
      v[22] = v[22];
   }
   if( 0 - v[26] + v[27] > 0.04 ) {
      v[24] = v[33];
   } else {
      v[24] = 0.04;
   }
   if( 0 - v[26] + v[27] < 0 ) {
      v[24] = v[33];
   } else {
      v[24] = v[24];
   }
   if( v[34] > v[31] ) {
      v[24] = v[24];
   } else {
      v[24] = v[33];
   }
   if( v[34] < 0 ) {
      v[24] = v[22];
   } else {
      v[24] = v[24];
   }
   if( fabs(v[23]) < 1e-09 ) {
      v[24] = 0;
   } else {
      v[24] = v[23] / v[24];
   }
   v[24] = v[32] + -0.2 * v[24] - v[29] * v[14];
   y[1] = sqrt(v[25] * v[25] + v[30] * v[30] + v[24] * v[24]) - 0.031;
   v[24] = cosf(x[2]);
   v[30] = sinf(x[2]);
   v[25] = 0 - v[30];
   v[29] = v[0] * v[24] + v[4] * v[25];
   v[32] = v[6] * v[24] + v[3] * v[25];
   v[25] = v[5] * v[24] + v[19] * v[25];
   v[14] = v[29] * v[1] + v[32] * v[10] + v[25] * v[13];
   v[23] = 0.1946 + -0.16 * v[4];
   v[22] = v[15] + 0.03745 * v[11] + -0.16 * v[3];
   v[34] = v[16] + 0.03745 * v[2] + -0.16 * v[19];
   v[31] = (0 - v[29]) * v[23] + (0 - v[32]) * v[22] + (0 - v[25]) * v[34] + 0.1946 * v[29] + v[32] * v[17] + v[25] * v[18];
   v[33] = 0.1 * v[14] + v[31];
   v[26] = 0 - v[33];
   v[33] = -0.1 * v[14] + v[31] - v[33];
   v[31] = v[11] * v[10] + v[2] * v[13];
   v[14] = (0 - v[11]) * v[22] + (0 - v[2]) * v[34] + v[11] * v[17] + v[2] * v[18];
   v[27] = 0.1 * v[31] + v[14];
   v[14] = -0.1 * v[31] + v[14] - v[27];
   v[31] = v[0] * v[30] + v[4] * v[24];
   v[28] = v[6] * v[30] + v[3] * v[24];
   v[30] = v[5] * v[30] + v[19] * v[24];
   v[13] = v[31] * v[1] + v[28] * v[10] + v[30] * v[13];
   v[18] = (0 - v[31]) * v[23] + (0 - v[28]) * v[22] + (0 - v[30]) * v[34] + 0.1946 * v[31] + v[28] * v[17] + v[30] * v[18];
   v[17] = 0.1 * v[13] + v[18];
   v[18] = -0.1 * v[13] + v[18] - v[17];
   v[13] = v[33] * v[33] + v[14] * v[14] + v[18] * v[18];
   v[10] = -0.14 * v[18];
   v[1] = 0.0196 * v[13] - v[10] * v[10];
   v[27] = 0 - v[27];
   v[17] = 0.07 - v[17];
   v[24] = v[33] * v[26] + v[14] * v[27] + v[18] * v[17];
   v[35] = -0.14 * v[17];
   if( v[1] < 1e-09 ) {
      v[36] = 0;
   } else {
      v[36] = v[10] * v[24] - v[13] * v[35];
   }
   if( v[1] < 1e-09 ) {
      v[37] = 1;
   } else {
      v[37] = v[1];
   }
   if( v[1] < 1e-09 ) {
      v[38] = v[24];
   } else {
      v[38] = 0.0196 * v[24] - v[10] * v[35];
   }
   if( v[36] < 0 ) {
      v[38] = v[24];
   } else {
      v[38] = v[38];
   }
   if( v[36] > v[37] ) {
      v[38] = v[24] + v[10];
   } else {
      v[38] = v[38];
   }
   if( v[38] < 0 ) {
      v[24] = 0;
   } else {
      v[24] = v[38];
   }
   if( v[1] < 1e-09 ) {
      v[1] = v[13];
   } else {
      v[1] = v[1];
   }
   if( v[36] < 0 ) {
      v[1] = v[13];
   } else {
      v[1] = v[1];
   }
   if( v[36] > v[37] ) {
      v[1] = v[13];
   } else {
      v[1] = v[1];
   }
   if( v[24] > v[1] ) {
      v[24] = v[1];
   } else {
      v[24] = v[24];
   }
   if( fabs(v[24]) < 1e-09 ) {
      v[24] = 0;
   } else {
      v[24] = v[24] / v[1];
   }
   v[33] = v[26] - v[24] * v[33];
   v[27] = v[27] - v[24] * v[14];
   if( 0 - v[35] > 0.0196 ) {
      v[14] = v[37];
   } else {
      v[14] = 0 - v[35];
   }
   if( 0 - v[35] < 0 ) {
      v[14] = 0;
   } else {
      v[14] = v[14];
   }
   if( 0 - v[35] + v[10] > 0.0196 ) {
      v[26] = v[37];
   } else {
      v[26] = 0 - v[35] + v[10];
   }
   if( 0 - v[35] + v[10] < 0 ) {
      v[26] = 0;
   } else {
      v[26] = v[26];
   }
   if( v[36] < 0 ) {
      v[36] = 0;
   } else {
      v[36] = v[36];
   }
   if( v[36] > v[37] ) {
      v[36] = v[37];
   } else {
      v[36] = v[36];
   }
   if( v[38] > v[1] ) {
      v[36] = v[26];
   } else {
      v[36] = v[36];
   }
   if( v[38] < 0 ) {
      v[36] = v[14];
   } else {
      v[36] = v[36];
   }
   if( 0 - v[35] > 0.0196 ) {
      v[14] = v[37];
   } else {
      v[14] = 0.0196;
   }
   if( 0 - v[35] < 0 ) {
      v[14] = v[37];
   } else {
      v[14] = v[14];
   }
   if( 0 - v[35] + v[10] > 0.0196 ) {
      v[26] = v[37];
   } else {
      v[26] = 0.0196;
   }
   if( 0 - v[35] + v[10] < 0 ) {
      v[26] = v[37];
   } else {
      v[26] = v[26];
   }
   if( v[38] > v[1] ) {
      v[26] = v[26];
   } else {
      v[26] = v[37];
   }
   if( v[38] < 0 ) {
      v[26] = v[14];
   } else {
      v[26] = v[26];
   }
   if( fabs(v[36]) < 1e-09 ) {
      v[26] = 0;
   } else {
      v[26] = v[36] / v[26];
   }
   v[26] = v[17] + -0.14 * v[26] - v[24] * v[18];
   y[2] = sqrt(v[33] * v[33] + v[27] * v[27] + v[26] * v[26]) - 0.031;
   v[26] = v[29] * v[9] + v[32] * v[8] + v[25] * v[20];
   v[25] = (0 - v[29]) * v[23] + (0 - v[32]) * v[22] + (0 - v[25]) * v[34] + v[29] * v[21] + v[32] * v[12] + v[25] * v[7];
   v[32] = 0.07 * v[26] + v[25];
   v[29] = 0 - v[32];
   v[32] = -0.07 * v[26] + v[25] - v[32];
   v[25] = v[11] * v[8] + v[2] * v[20];
   v[26] = (0 - v[11]) * v[22] + (0 - v[2]) * v[34] + v[11] * v[12] + v[2] * v[7];
   v[27] = 0.07 * v[25] + v[26];
   v[26] = -0.07 * v[25] + v[26] - v[27];
   v[20] = v[31] * v[9] + v[28] * v[8] + v[30] * v[20];
   v[30] = (0 - v[31]) * v[23] + (0 - v[28]) * v[22] + (0 - v[30]) * v[34] + v[31] * v[21] + v[28] * v[12] + v[30] * v[7];
   v[28] = 0.07 * v[20] + v[30];
   v[30] = -0.07 * v[20] + v[30] - v[28];
   v[20] = v[32] * v[32] + v[26] * v[26] + v[30] * v[30];
   v[31] = -0.14 * v[30];
   v[34] = 0.0196 * v[20] - v[31] * v[31];
   v[27] = 0 - v[27];
   v[28] = 0.07 - v[28];
   v[22] = v[32] * v[29] + v[26] * v[27] + v[30] * v[28];
   v[23] = -0.14 * v[28];
   if( v[34] < 1e-09 ) {
      v[7] = 0;
   } else {
      v[7] = v[31] * v[22] - v[20] * v[23];
   }
   if( v[34] < 1e-09 ) {
      v[12] = 1;
   } else {
      v[12] = v[34];
   }
   if( v[34] < 1e-09 ) {
      v[21] = v[22];
   } else {
      v[21] = 0.0196 * v[22] - v[31] * v[23];
   }
   if( v[7] < 0 ) {
      v[21] = v[22];
   } else {
      v[21] = v[21];
   }
   if( v[7] > v[12] ) {
      v[21] = v[22] + v[31];
   } else {
      v[21] = v[21];
   }
   if( v[21] < 0 ) {
      v[22] = 0;
   } else {
      v[22] = v[21];
   }
   if( v[34] < 1e-09 ) {
      v[34] = v[20];
   } else {
      v[34] = v[34];
   }
   if( v[7] < 0 ) {
      v[34] = v[20];
   } else {
      v[34] = v[34];
   }
   if( v[7] > v[12] ) {
      v[34] = v[20];
   } else {
      v[34] = v[34];
   }
   if( v[22] > v[34] ) {
      v[22] = v[34];
   } else {
      v[22] = v[22];
   }
   if( fabs(v[22]) < 1e-09 ) {
      v[22] = 0;
   } else {
      v[22] = v[22] / v[34];
   }
   v[32] = v[29] - v[22] * v[32];
   v[27] = v[27] - v[22] * v[26];
   if( 0 - v[23] > 0.0196 ) {
      v[26] = v[12];
   } else {
      v[26] = 0 - v[23];
   }
   if( 0 - v[23] < 0 ) {
      v[26] = 0;
   } else {
      v[26] = v[26];
   }
   if( 0 - v[23] + v[31] > 0.0196 ) {
      v[29] = v[12];
   } else {
      v[29] = 0 - v[23] + v[31];
   }
   if( 0 - v[23] + v[31] < 0 ) {
      v[29] = 0;
   } else {
      v[29] = v[29];
   }
   if( v[7] < 0 ) {
      v[7] = 0;
   } else {
      v[7] = v[7];
   }
   if( v[7] > v[12] ) {
      v[7] = v[12];
   } else {
      v[7] = v[7];
   }
   if( v[21] > v[34] ) {
      v[7] = v[29];
   } else {
      v[7] = v[7];
   }
   if( v[21] < 0 ) {
      v[7] = v[26];
   } else {
      v[7] = v[7];
   }
   if( 0 - v[23] > 0.0196 ) {
      v[26] = v[12];
   } else {
      v[26] = 0.0196;
   }
   if( 0 - v[23] < 0 ) {
      v[26] = v[12];
   } else {
      v[26] = v[26];
   }
   if( 0 - v[23] + v[31] > 0.0196 ) {
      v[29] = v[12];
   } else {
      v[29] = 0.0196;
   }
   if( 0 - v[23] + v[31] < 0 ) {
      v[29] = v[12];
   } else {
      v[29] = v[29];
   }
   if( v[21] > v[34] ) {
      v[29] = v[29];
   } else {
      v[29] = v[12];
   }
   if( v[21] < 0 ) {
      v[29] = v[26];
   } else {
      v[29] = v[29];
   }
   if( fabs(v[7]) < 1e-09 ) {
      v[29] = 0;
   } else {
      v[29] = v[7] / v[29];
   }
   v[29] = v[28] + -0.14 * v[29] - v[22] * v[30];
   y[3] = sqrt(v[32] * v[32] + v[27] * v[27] + v[29] * v[29]) - 0.03;
   v[29] = sinf(x[7]);
   v[27] = sinf(x[6]);
   v[32] = 0 - v[27];
   v[22] = cosf(x[7]);
   v[28] = v[32] * v[22];
   v[30] = cosf(x[6]);
   v[7] = v[30] * v[22];
   v[26] = v[0] * v[29] + v[6] * v[28] + v[5] * v[7];
   v[21] = 0.0875 + 0.014 * v[30];
   v[34] = 0.014 * v[27];
   v[12] = 0.1946 * (0 - v[0]) + (0 - v[6]) * v[15] + (0 - v[5]) * v[16] + -0.1946 * v[0] + v[6] * v[21] + v[5] * v[34];
   v[23] = 0.1 * v[26] + v[12];
   v[31] = 0 - v[23];
   v[23] = -0.1 * v[26] + v[12] - v[23];
   v[12] = v[11] * v[28] + v[2] * v[7];
   v[26] = (0 - v[11]) * v[15] + (0 - v[2]) * v[16] + v[11] * v[21] + v[2] * v[34];
   v[20] = 0.1 * v[12] + v[26];
   v[26] = -0.1 * v[12] + v[26] - v[20];
   v[12] = v[4] * v[29] + v[3] * v[28] + v[19] * v[7];
   v[8] = 0.1946 * (0 - v[4]) + (0 - v[3]) * v[15] + (0 - v[19]) * v[16] + -0.1946 * v[4] + v[3] * v[21] + v[19] * v[34];
   v[9] = 0.1 * v[12] + v[8];
   v[8] = -0.1 * v[12] + v[8] - v[9];
   v[12] = v[23] * v[23] + v[26] * v[26] + v[8] * v[8];
   v[25] = -0.2 * v[8];
   v[33] = 0.04 * v[12] - v[25] * v[25];
   v[20] = 0 - v[20];
   v[9] = 0.1 - v[9];
   v[24] = v[23] * v[31] + v[26] * v[20] + v[8] * v[9];
   v[17] = -0.2 * v[9];
   if( v[33] < 1e-09 ) {
      v[18] = 0;
   } else {
      v[18] = v[25] * v[24] - v[12] * v[17];
   }
   if( v[33] < 1e-09 ) {
      v[36] = 1;
   } else {
      v[36] = v[33];
   }
   if( v[33] < 1e-09 ) {
      v[14] = v[24];
   } else {
      v[14] = 0.04 * v[24] - v[25] * v[17];
   }
   if( v[18] < 0 ) {
      v[14] = v[24];
   } else {
      v[14] = v[14];
   }
   if( v[18] > v[36] ) {
      v[14] = v[24] + v[25];
   } else {
      v[14] = v[14];
   }
   if( v[14] < 0 ) {
      v[24] = 0;
   } else {
      v[24] = v[14];
   }
   if( v[33] < 1e-09 ) {
      v[33] = v[12];
   } else {
      v[33] = v[33];
   }
   if( v[18] < 0 ) {
      v[33] = v[12];
   } else {
      v[33] = v[33];
   }
   if( v[18] > v[36] ) {
      v[33] = v[12];
   } else {
      v[33] = v[33];
   }
   if( v[24] > v[33] ) {
      v[24] = v[33];
   } else {
      v[24] = v[24];
   }
   if( fabs(v[24]) < 1e-09 ) {
      v[24] = 0;
   } else {
      v[24] = v[24] / v[33];
   }
   v[23] = v[31] - v[24] * v[23];
   v[20] = v[20] - v[24] * v[26];
   if( 0 - v[17] > 0.04 ) {
      v[26] = v[36];
   } else {
      v[26] = 0 - v[17];
   }
   if( 0 - v[17] < 0 ) {
      v[26] = 0;
   } else {
      v[26] = v[26];
   }
   if( 0 - v[17] + v[25] > 0.04 ) {
      v[31] = v[36];
   } else {
      v[31] = 0 - v[17] + v[25];
   }
   if( 0 - v[17] + v[25] < 0 ) {
      v[31] = 0;
   } else {
      v[31] = v[31];
   }
   if( v[18] < 0 ) {
      v[18] = 0;
   } else {
      v[18] = v[18];
   }
   if( v[18] > v[36] ) {
      v[18] = v[36];
   } else {
      v[18] = v[18];
   }
   if( v[14] > v[33] ) {
      v[18] = v[31];
   } else {
      v[18] = v[18];
   }
   if( v[14] < 0 ) {
      v[18] = v[26];
   } else {
      v[18] = v[18];
   }
   if( 0 - v[17] > 0.04 ) {
      v[26] = v[36];
   } else {
      v[26] = 0.04;
   }
   if( 0 - v[17] < 0 ) {
      v[26] = v[36];
   } else {
      v[26] = v[26];
   }
   if( 0 - v[17] + v[25] > 0.04 ) {
      v[31] = v[36];
   } else {
      v[31] = 0.04;
   }
   if( 0 - v[17] + v[25] < 0 ) {
      v[31] = v[36];
   } else {
      v[31] = v[31];
   }
   if( v[14] > v[33] ) {
      v[31] = v[31];
   } else {
      v[31] = v[36];
   }
   if( v[14] < 0 ) {
      v[31] = v[26];
   } else {
      v[31] = v[31];
   }
   if( fabs(v[18]) < 1e-09 ) {
      v[31] = 0;
   } else {
      v[31] = v[18] / v[31];
   }
   v[31] = v[9] + -0.2 * v[31] - v[24] * v[8];
   y[4] = sqrt(v[23] * v[23] + v[20] * v[20] + v[31] * v[31]) - 0.032;
   v[31] = sinf(x[8]);
   v[20] = cosf(x[8]);
   v[22] = v[22] * v[31] + v[29] * v[20];
   v[23] = 0 - v[29];
   v[32] = v[32] * v[23] * v[31] + v[28] * v[20];
   v[23] = v[30] * v[23] * v[31] + v[7] * v[20];
   v[20] = v[0] * v[22] + v[6] * v[32] + v[5] * v[23];
   v[29] = -0.1946 + -0.16 * v[29];
   v[21] = v[21] + 0.03745 * v[30] + -0.16 * v[28];
   v[34] = v[34] + 0.03745 * v[27] + -0.16 * v[7];
   v[5] = 0.1946 * (0 - v[0]) + (0 - v[6]) * v[15] + (0 - v[5]) * v[16] + v[0] * v[29] + v[6] * v[21] + v[5] * v[34];
   v[6] = 0.07 * v[20] + v[5];
   v[0] = 0 - v[6];
   v[6] = -0.07 * v[20] + v[5] - v[6];
   v[5] = v[11] * v[32] + v[2] * v[23];
   v[11] = (0 - v[11]) * v[15] + (0 - v[2]) * v[16] + v[11] * v[21] + v[2] * v[34];
   v[2] = 0.07 * v[5] + v[11];
   v[11] = -0.07 * v[5] + v[11] - v[2];
   v[23] = v[4] * v[22] + v[3] * v[32] + v[19] * v[23];
   v[34] = 0.1946 * (0 - v[4]) + (0 - v[3]) * v[15] + (0 - v[19]) * v[16] + v[4] * v[29] + v[3] * v[21] + v[19] * v[34];
   v[21] = 0.07 * v[23] + v[34];
   v[34] = -0.07 * v[23] + v[34] - v[21];
   v[23] = v[6] * v[6] + v[11] * v[11] + v[34] * v[34];
   v[29] = -0.2 * v[34];
   v[19] = 0.04 * v[23] - v[29] * v[29];
   v[2] = 0 - v[2];
   v[21] = 0.1 - v[21];
   v[3] = v[6] * v[0] + v[11] * v[2] + v[34] * v[21];
   v[16] = -0.2 * v[21];
   if( v[19] < 1e-09 ) {
      v[15] = 0;
   } else {
      v[15] = v[29] * v[3] - v[23] * v[16];
   }
   if( v[19] < 1e-09 ) {
      v[4] = 1;
   } else {
      v[4] = v[19];
   }
   if( v[19] < 1e-09 ) {
      v[32] = v[3];
   } else {
      v[32] = 0.04 * v[3] - v[29] * v[16];
   }
   if( v[15] < 0 ) {
      v[32] = v[3];
   } else {
      v[32] = v[32];
   }
   if( v[15] > v[4] ) {
      v[32] = v[3] + v[29];
   } else {
      v[32] = v[32];
   }
   if( v[32] < 0 ) {
      v[3] = 0;
   } else {
      v[3] = v[32];
   }
   if( v[19] < 1e-09 ) {
      v[19] = v[23];
   } else {
      v[19] = v[19];
   }
   if( v[15] < 0 ) {
      v[19] = v[23];
   } else {
      v[19] = v[19];
   }
   if( v[15] > v[4] ) {
      v[19] = v[23];
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
   if( 0 - v[16] + v[29] > 0.04 ) {
      v[0] = v[4];
   } else {
      v[0] = 0 - v[16] + v[29];
   }
   if( 0 - v[16] + v[29] < 0 ) {
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
   if( v[32] > v[19] ) {
      v[15] = v[0];
   } else {
      v[15] = v[15];
   }
   if( v[32] < 0 ) {
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
   if( 0 - v[16] + v[29] > 0.04 ) {
      v[0] = v[4];
   } else {
      v[0] = 0.04;
   }
   if( 0 - v[16] + v[29] < 0 ) {
      v[0] = v[4];
   } else {
      v[0] = v[0];
   }
   if( v[32] > v[19] ) {
      v[0] = v[0];
   } else {
      v[0] = v[4];
   }
   if( v[32] < 0 ) {
      v[0] = v[11];
   } else {
      v[0] = v[0];
   }
   if( fabs(v[15]) < 1e-09 ) {
      v[0] = 0;
   } else {
      v[0] = v[15] / v[0];
   }
   v[0] = v[21] + -0.2 * v[0] - v[3] * v[34];
   y[5] = sqrt(v[6] * v[6] + v[2] * v[2] + v[0] * v[0]) - 0.031;
}

