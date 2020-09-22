#include <math.h>
#include <stdio.h>

void solo_autocollision_legs_legs_forward_zero(double* x, double* y) {
   // auxiliary variables
   double v[34];

   v[0] = cos(x[0]);
   v[1] = sin(x[4]);
   v[2] = -0.19 + -0.16 * v[1];
   v[3] = sin(x[0]);
   v[4] = 0 - v[3];
   v[5] = cos(x[4]);
   v[6] = -0.16 * v[5];
   v[7] = 0.19 * (0 - v[0]) + v[0] * v[2] + v[4] * v[6];
   v[8] = sin(x[5]);
   v[9] = cos(x[5]);
   v[10] = v[5] * v[8] + v[1] * v[9];
   v[11] = 0 - v[1];
   v[12] = v[11] * v[8] + v[5] * v[9];
   v[13] = v[0] * v[10] + v[4] * v[12];
   v[14] = v[7] + -0.16 * v[13];
   v[15] = 0 - v[14];
   v[16] = v[7] - v[14];
   v[17] = 0.19 * (0 - v[3]) + v[3] * v[2] + v[0] * v[6];
   v[18] = v[3] * v[10] + v[0] * v[12];
   v[19] = v[17] + -0.16 * v[18];
   v[20] = v[17] - v[19];
   v[21] = v[16] * v[16] + v[20] * v[20];
   v[22] = 0.158 * v[20];
   v[23] = 0.024964 * v[21] - v[22] * v[22];
   v[24] = -0.158 - v[19];
   v[25] = v[16] * v[15] + v[20] * v[24];
   v[26] = 0.158 * v[24];
   if( v[23] < 1e-09 ) {
      v[27] = 0;
   } else {
      v[27] = v[22] * v[25] - v[21] * v[26];
   }
   if( v[23] < 1e-09 ) {
      v[28] = 1;
   } else {
      v[28] = v[23];
   }
   if( v[23] < 1e-09 ) {
      v[29] = v[25];
   } else {
      v[29] = 0.024964 * v[25] - v[22] * v[26];
   }
   if( v[27] < 0 ) {
      v[29] = v[25];
   } else {
      v[29] = v[29];
   }
   if( v[27] > v[28] ) {
      v[29] = v[25] + v[22];
   } else {
      v[29] = v[29];
   }
   if( v[29] < 0 ) {
      v[25] = 0;
   } else {
      v[25] = v[29];
   }
   if( v[23] < 1e-09 ) {
      v[23] = v[21];
   } else {
      v[23] = v[23];
   }
   if( v[27] < 0 ) {
      v[23] = v[21];
   } else {
      v[23] = v[23];
   }
   if( v[27] > v[28] ) {
      v[23] = v[21];
   } else {
      v[23] = v[23];
   }
   if( v[25] > v[23] ) {
      v[25] = v[23];
   } else {
      v[25] = v[25];
   }
   if( fabs(v[25]) < 1e-09 ) {
      v[25] = 0;
   } else {
      v[25] = v[25] / v[23];
   }
   v[15] = v[15] - v[25] * v[16];
   if( 0 - v[26] > 0.024964 ) {
      v[21] = v[28];
   } else {
      v[21] = 0 - v[26];
   }
   if( 0 - v[26] < 0 ) {
      v[21] = 0;
   } else {
      v[21] = v[21];
   }
   if( 0 - v[26] + v[22] > 0.024964 ) {
      v[30] = v[28];
   } else {
      v[30] = 0 - v[26] + v[22];
   }
   if( 0 - v[26] + v[22] < 0 ) {
      v[30] = 0;
   } else {
      v[30] = v[30];
   }
   if( v[27] < 0 ) {
      v[27] = 0;
   } else {
      v[27] = v[27];
   }
   if( v[27] > v[28] ) {
      v[27] = v[28];
   } else {
      v[27] = v[27];
   }
   if( v[29] > v[23] ) {
      v[27] = v[30];
   } else {
      v[27] = v[27];
   }
   if( v[29] < 0 ) {
      v[27] = v[21];
   } else {
      v[27] = v[27];
   }
   if( 0 - v[26] > 0.024964 ) {
      v[21] = v[28];
   } else {
      v[21] = 0.024964;
   }
   if( 0 - v[26] < 0 ) {
      v[21] = v[28];
   } else {
      v[21] = v[21];
   }
   if( 0 - v[26] + v[22] > 0.024964 ) {
      v[30] = v[28];
   } else {
      v[30] = 0.024964;
   }
   if( 0 - v[26] + v[22] < 0 ) {
      v[30] = v[28];
   } else {
      v[30] = v[30];
   }
   if( v[29] > v[23] ) {
      v[30] = v[30];
   } else {
      v[30] = v[28];
   }
   if( v[29] < 0 ) {
      v[30] = v[21];
   } else {
      v[30] = v[30];
   }
   if( fabs(v[27]) < 1e-09 ) {
      v[30] = 0;
   } else {
      v[30] = v[27] / v[30];
   }
   v[24] = v[24] + 0.158 * v[30] - v[25] * v[20];
   v[24] = v[15] * v[15] + 0.000838102500000001 + v[24] * v[24];
   y[0] = sqrt(v[24]) - 0.04;
   v[8] = 0 - v[8];
   v[15] = v[5] * v[9] + v[1] * v[8];
   v[8] = v[11] * v[9] + v[5] * v[8];
   v[9] = v[0] * v[15] + v[4] * v[8];
   v[27] = 0 - v[13];
   v[21] = v[3] * v[15] + v[0] * v[8];
   v[29] = 1 / (v[18] * v[9] + v[27] * v[21]);
   v[16] = v[14] + v[25] * v[16];
   v[24] = sqrt(v[24]);
   v[14] = v[16] / v[24];
   v[16] = v[16] - 0.015 * v[14];
   v[25] = v[19] + v[25] * v[20];
   v[30] = -0.158 + 0.158 * v[30];
   v[20] = (v[25] - v[30]) / v[24];
   v[25] = v[25] - 0.015 * v[20];
   v[27] = 0 - v[7] + v[18] * v[29] * v[16] + v[27] * v[29] * v[25];
   v[25] = 0 - v[17] + (0 - v[21]) * v[29] * v[16] + v[9] * v[29] * v[25];
   v[16] = v[2] + v[15] * v[27] + v[10] * v[25];
   v[25] = v[6] + v[8] * v[27] + v[12] * v[25];
   v[14] = 0.025 * v[14];
   v[20] = v[30] + 0.025 * v[20];
   v[30] = 0.19 + v[0] * v[14] + v[3] * v[20];
   v[20] = v[4] * v[14] + v[0] * v[20];
   v[14] = v[2] + v[15] * v[16] + v[10] * v[25] - (0.19 + v[0] * v[30] + v[3] * v[20]);
   v[27] = 0 - (0 - v[20]);
   v[17] = 0 - v[30];
   v[20] = v[6] + v[8] * v[16] + v[12] * v[25] - (v[4] * v[30] + v[0] * v[20]);
   v[24] = 0.02895 / v[24];
   v[24] = 0.14205 + 0.14205 + -0.03745 + (v[18] * v[9] - v[21] * v[13]) * v[29] * (0.04545 - 0.015 * v[24]) - (0.1046 + 0.1046 + 0.0165 + 0.025 * v[24]);
   v[24] = sqrt(v[14] * v[14] + v[24] * v[24] + v[20] * v[20]);
   y[1] = (v[14] * (0 - (v[0] * v[27] + v[3] * v[17])) + v[20] * (0 - (v[4] * v[27] + v[0] * v[17]))) / v[24];
   v[25] = 0 - v[25];
   v[17] = -0.16 * cos(x[5]) - v[25];
   v[27] = -0.16 * sin(x[5]) - v[16];
   y[5] = (v[14] * (v[15] * v[17] + v[10] * v[27]) + v[20] * (v[8] * v[17] + v[12] * v[27])) / v[24];
   v[25] = 0 - v[25];
   v[16] = 0 - v[16];
   y[6] = (v[14] * (v[15] * v[25] + v[10] * v[16]) + v[20] * (v[8] * v[25] + v[12] * v[16])) / v[24];
   v[16] = cos(x[1]);
   v[25] = sin(x[1]);
   v[24] = 0 - v[25];
   v[20] = v[0] * v[16] + v[3] * v[24];
   v[14] = 0.19 + -0.16 * v[3];
   v[24] = v[4] * v[16] + v[0] * v[24];
   v[27] = -0.16 * v[0];
   v[17] = (0 - v[20]) * v[14] + (0 - v[24]) * v[27] + -0.19 * v[20];
   v[29] = v[20] * v[1] + v[24] * v[5];
   v[21] = v[17] + -0.158 * v[29];
   v[9] = 0 - v[21];
   v[18] = v[17] - v[21];
   v[3] = v[0] * v[25] + v[3] * v[16];
   v[25] = v[4] * v[25] + v[0] * v[16];
   v[16] = (0 - v[3]) * v[14] + (0 - v[25]) * v[27] + -0.19 * v[3];
   v[4] = v[3] * v[1] + v[25] * v[5];
   v[0] = v[16] + -0.158 * v[4];
   v[13] = v[16] - v[0];
   v[30] = v[18] * v[18] + v[13] * v[13];
   v[7] = 0.16 * v[13];
   v[19] = 0.0256 * v[30] - v[7] * v[7];
   v[23] = -0.16 - v[0];
   v[28] = v[18] * v[9] + v[13] * v[23];
   v[26] = 0.16 * v[23];
   if( v[19] < 1e-09 ) {
      v[22] = 0;
   } else {
      v[22] = v[7] * v[28] - v[30] * v[26];
   }
   if( v[19] < 1e-09 ) {
      v[31] = 1;
   } else {
      v[31] = v[19];
   }
   if( v[19] < 1e-09 ) {
      v[32] = v[28];
   } else {
      v[32] = 0.0256 * v[28] - v[7] * v[26];
   }
   if( v[22] < 0 ) {
      v[32] = v[28];
   } else {
      v[32] = v[32];
   }
   if( v[22] > v[31] ) {
      v[32] = v[28] + v[7];
   } else {
      v[32] = v[32];
   }
   if( v[32] < 0 ) {
      v[28] = 0;
   } else {
      v[28] = v[32];
   }
   if( v[19] < 1e-09 ) {
      v[19] = v[30];
   } else {
      v[19] = v[19];
   }
   if( v[22] < 0 ) {
      v[19] = v[30];
   } else {
      v[19] = v[19];
   }
   if( v[22] > v[31] ) {
      v[19] = v[30];
   } else {
      v[19] = v[19];
   }
   if( v[28] > v[19] ) {
      v[28] = v[19];
   } else {
      v[28] = v[28];
   }
   if( fabs(v[28]) < 1e-09 ) {
      v[28] = 0;
   } else {
      v[28] = v[28] / v[19];
   }
   v[9] = v[9] - v[28] * v[18];
   if( 0 - v[26] > 0.0256 ) {
      v[30] = v[31];
   } else {
      v[30] = 0 - v[26];
   }
   if( 0 - v[26] < 0 ) {
      v[30] = 0;
   } else {
      v[30] = v[30];
   }
   if( 0 - v[26] + v[7] > 0.0256 ) {
      v[33] = v[31];
   } else {
      v[33] = 0 - v[26] + v[7];
   }
   if( 0 - v[26] + v[7] < 0 ) {
      v[33] = 0;
   } else {
      v[33] = v[33];
   }
   if( v[22] < 0 ) {
      v[22] = 0;
   } else {
      v[22] = v[22];
   }
   if( v[22] > v[31] ) {
      v[22] = v[31];
   } else {
      v[22] = v[22];
   }
   if( v[32] > v[19] ) {
      v[22] = v[33];
   } else {
      v[22] = v[22];
   }
   if( v[32] < 0 ) {
      v[22] = v[30];
   } else {
      v[22] = v[22];
   }
   if( 0 - v[26] > 0.0256 ) {
      v[30] = v[31];
   } else {
      v[30] = 0.0256;
   }
   if( 0 - v[26] < 0 ) {
      v[30] = v[31];
   } else {
      v[30] = v[30];
   }
   if( 0 - v[26] + v[7] > 0.0256 ) {
      v[33] = v[31];
   } else {
      v[33] = 0.0256;
   }
   if( 0 - v[26] + v[7] < 0 ) {
      v[33] = v[31];
   } else {
      v[33] = v[33];
   }
   if( v[32] > v[19] ) {
      v[33] = v[33];
   } else {
      v[33] = v[31];
   }
   if( v[32] < 0 ) {
      v[33] = v[30];
   } else {
      v[33] = v[33];
   }
   if( fabs(v[22]) < 1e-09 ) {
      v[33] = 0;
   } else {
      v[33] = v[22] / v[33];
   }
   v[23] = v[23] + 0.16 * v[33] - v[28] * v[13];
   v[23] = v[9] * v[9] + 0.000838102500000001 + v[23] * v[23];
   y[9] = sqrt(v[23]) - 0.04;
   v[9] = v[20] * v[5] + v[24] * v[11];
   v[22] = 0 - v[29];
   v[30] = v[3] * v[5] + v[25] * v[11];
   v[32] = 1 / (v[4] * v[9] + v[22] * v[30]);
   v[18] = v[21] + v[28] * v[18];
   v[23] = sqrt(v[23]);
   v[21] = v[18] / v[23];
   v[18] = v[18] - 0.025 * v[21];
   v[28] = v[0] + v[28] * v[13];
   v[33] = -0.16 + 0.16 * v[33];
   v[13] = (v[28] - v[33]) / v[23];
   v[28] = v[28] - 0.025 * v[13];
   v[22] = 0 - v[17] + v[4] * v[32] * v[18] + v[22] * v[32] * v[28];
   v[28] = 0 - v[16] + (0 - v[30]) * v[32] * v[18] + v[9] * v[32] * v[28];
   v[18] = -0.19 + v[5] * v[22] + v[1] * v[28];
   v[28] = v[11] * v[22] + v[5] * v[28];
   v[21] = 0.015 * v[21];
   v[13] = v[33] + 0.015 * v[13];
   v[33] = v[14] + v[20] * v[21] + v[3] * v[13];
   v[13] = v[27] + v[24] * v[21] + v[25] * v[13];
   v[21] = -0.19 + v[5] * v[18] + v[1] * v[28] - (v[14] + v[20] * v[33] + v[3] * v[13]);
   v[22] = 0 - v[13];
   v[16] = -0.16 * cos(x[1]) - v[22];
   v[17] = -0.16 * sin(x[1]) - v[33];
   v[13] = v[11] * v[18] + v[5] * v[28] - (v[27] + v[24] * v[33] + v[25] * v[13]);
   v[23] = -0.02895 / v[23];
   v[23] = 0.1046 + 0.1046 + 0.03745 + (v[4] * v[9] - v[30] * v[29]) * v[32] * (-0.02095 - 0.025 * v[23]) - (0.14205 + 0.14205 + 0.008 + 0.015 * v[23]);
   v[23] = sqrt(v[21] * v[21] + v[23] * v[23] + v[13] * v[13]);
   y[10] = (v[21] * (0 - (v[20] * v[16] + v[3] * v[17])) + v[13] * (0 - (v[24] * v[16] + v[25] * v[17]))) / v[23];
   v[22] = 0 - v[22];
   v[33] = 0 - v[33];
   y[11] = (v[21] * (0 - (v[20] * v[22] + v[3] * v[33])) + v[13] * (0 - (v[24] * v[22] + v[25] * v[33]))) / v[23];
   v[28] = 0 - (0 - v[28]);
   v[18] = 0 - v[18];
   y[14] = (v[21] * (v[5] * v[28] + v[1] * v[18]) + v[13] * (v[11] * v[28] + v[5] * v[18])) / v[23];
   v[18] = (0 - v[20]) * v[14] + (0 - v[24]) * v[27] + v[20] * v[2] + v[24] * v[6];
   v[28] = v[20] * v[10] + v[24] * v[12];
   v[23] = v[18] + -0.16 * v[28];
   v[13] = 0 - v[23];
   v[21] = v[18] - v[23];
   v[11] = (0 - v[3]) * v[14] + (0 - v[25]) * v[27] + v[3] * v[2] + v[25] * v[6];
   v[5] = v[3] * v[10] + v[25] * v[12];
   v[1] = v[11] + -0.16 * v[5];
   v[33] = v[11] - v[1];
   v[22] = v[21] * v[21] + v[33] * v[33];
   v[17] = 0.16 * v[33];
   v[16] = 0.0256 * v[22] - v[17] * v[17];
   v[32] = -0.16 - v[1];
   v[30] = v[21] * v[13] + v[33] * v[32];
   v[9] = 0.16 * v[32];
   if( v[16] < 1e-09 ) {
      v[4] = 0;
   } else {
      v[4] = v[17] * v[30] - v[22] * v[9];
   }
   if( v[16] < 1e-09 ) {
      v[29] = 1;
   } else {
      v[29] = v[16];
   }
   if( v[16] < 1e-09 ) {
      v[0] = v[30];
   } else {
      v[0] = 0.0256 * v[30] - v[17] * v[9];
   }
   if( v[4] < 0 ) {
      v[0] = v[30];
   } else {
      v[0] = v[0];
   }
   if( v[4] > v[29] ) {
      v[0] = v[30] + v[17];
   } else {
      v[0] = v[0];
   }
   if( v[0] < 0 ) {
      v[30] = 0;
   } else {
      v[30] = v[0];
   }
   if( v[16] < 1e-09 ) {
      v[16] = v[22];
   } else {
      v[16] = v[16];
   }
   if( v[4] < 0 ) {
      v[16] = v[22];
   } else {
      v[16] = v[16];
   }
   if( v[4] > v[29] ) {
      v[16] = v[22];
   } else {
      v[16] = v[16];
   }
   if( v[30] > v[16] ) {
      v[30] = v[16];
   } else {
      v[30] = v[30];
   }
   if( fabs(v[30]) < 1e-09 ) {
      v[30] = 0;
   } else {
      v[30] = v[30] / v[16];
   }
   v[13] = v[13] - v[30] * v[21];
   if( 0 - v[9] > 0.0256 ) {
      v[22] = v[29];
   } else {
      v[22] = 0 - v[9];
   }
   if( 0 - v[9] < 0 ) {
      v[22] = 0;
   } else {
      v[22] = v[22];
   }
   if( 0 - v[9] + v[17] > 0.0256 ) {
      v[19] = v[29];
   } else {
      v[19] = 0 - v[9] + v[17];
   }
   if( 0 - v[9] + v[17] < 0 ) {
      v[19] = 0;
   } else {
      v[19] = v[19];
   }
   if( v[4] < 0 ) {
      v[4] = 0;
   } else {
      v[4] = v[4];
   }
   if( v[4] > v[29] ) {
      v[4] = v[29];
   } else {
      v[4] = v[4];
   }
   if( v[0] > v[16] ) {
      v[4] = v[19];
   } else {
      v[4] = v[4];
   }
   if( v[0] < 0 ) {
      v[4] = v[22];
   } else {
      v[4] = v[4];
   }
   if( 0 - v[9] > 0.0256 ) {
      v[22] = v[29];
   } else {
      v[22] = 0.0256;
   }
   if( 0 - v[9] < 0 ) {
      v[22] = v[29];
   } else {
      v[22] = v[22];
   }
   if( 0 - v[9] + v[17] > 0.0256 ) {
      v[19] = v[29];
   } else {
      v[19] = 0.0256;
   }
   if( 0 - v[9] + v[17] < 0 ) {
      v[19] = v[29];
   } else {
      v[19] = v[19];
   }
   if( v[0] > v[16] ) {
      v[19] = v[19];
   } else {
      v[19] = v[29];
   }
   if( v[0] < 0 ) {
      v[19] = v[22];
   } else {
      v[19] = v[19];
   }
   if( fabs(v[4]) < 1e-09 ) {
      v[19] = 0;
   } else {
      v[19] = v[4] / v[19];
   }
   v[32] = v[32] + 0.16 * v[19] - v[30] * v[33];
   v[32] = v[13] * v[13] + v[32] * v[32];
   y[18] = sqrt(v[32]) - 0.03;
   v[13] = v[20] * v[15] + v[24] * v[8];
   v[4] = 0 - v[28];
   v[22] = v[3] * v[15] + v[25] * v[8];
   v[0] = 1 / (v[5] * v[13] + v[4] * v[22]);
   v[21] = v[23] + v[30] * v[21];
   v[32] = sqrt(v[32]);
   v[23] = v[21] / v[32];
   v[21] = v[21] - 0.015 * v[23];
   v[30] = v[1] + v[30] * v[33];
   v[19] = -0.16 + 0.16 * v[19];
   v[32] = (v[30] - v[19]) / v[32];
   v[30] = v[30] - 0.015 * v[32];
   v[4] = 0 - v[18] + v[5] * v[0] * v[21] + v[4] * v[0] * v[30];
   v[30] = 0 - v[11] + (0 - v[22]) * v[0] * v[21] + v[13] * v[0] * v[30];
   v[21] = v[2] + v[15] * v[4] + v[10] * v[30];
   v[30] = v[6] + v[8] * v[4] + v[12] * v[30];
   v[23] = 0.015 * v[23];
   v[32] = v[19] + 0.015 * v[32];
   v[19] = v[14] + v[20] * v[23] + v[3] * v[32];
   v[32] = v[27] + v[24] * v[23] + v[25] * v[32];
   v[14] = v[2] + v[15] * v[21] + v[10] * v[30] - (v[14] + v[20] * v[19] + v[3] * v[32]);
   v[2] = 0 - v[32];
   v[23] = -0.16 * cos(x[1]) - v[2];
   v[4] = -0.16 * sin(x[1]) - v[19];
   v[32] = v[6] + v[8] * v[21] + v[12] * v[30] - (v[27] + v[24] * v[19] + v[25] * v[32]);
   v[0] = 0.14205 + 0.14205 + 0.008 * (v[5] * v[13] - v[22] * v[28]) * v[0] - 0.2921;
   v[0] = sqrt(v[14] * v[14] + v[0] * v[0] + v[32] * v[32]);
   y[19] = (v[14] * (0 - (v[20] * v[23] + v[3] * v[4])) + v[32] * (0 - (v[24] * v[23] + v[25] * v[4]))) / v[0];
   v[2] = 0 - v[2];
   v[19] = 0 - v[19];
   y[20] = (v[14] * (0 - (v[20] * v[2] + v[3] * v[19])) + v[32] * (0 - (v[24] * v[2] + v[25] * v[19]))) / v[0];
   v[30] = 0 - v[30];
   v[19] = -0.16 * cos(x[5]) - v[30];
   v[2] = -0.16 * sin(x[5]) - v[21];
   y[23] = (v[14] * (v[15] * v[19] + v[10] * v[2]) + v[32] * (v[8] * v[19] + v[12] * v[2])) / v[0];
   v[30] = 0 - v[30];
   v[21] = 0 - v[21];
   y[24] = (v[14] * (v[15] * v[30] + v[10] * v[21]) + v[32] * (v[8] * v[30] + v[12] * v[21])) / v[0];
   v[21] = cos(x[2]);
   v[30] = sin(x[6]);
   v[0] = -0.19 + -0.16 * v[30];
   v[32] = sin(x[2]);
   v[14] = 0 - v[32];
   v[8] = cos(x[6]);
   v[15] = -0.16 * v[8];
   v[12] = 0.19 * (0 - v[21]) + v[21] * v[0] + v[14] * v[15];
   v[10] = sin(x[7]);
   v[2] = cos(x[7]);
   v[19] = v[8] * v[10] + v[30] * v[2];
   v[25] = 0 - v[30];
   v[3] = v[25] * v[10] + v[8] * v[2];
   v[24] = v[21] * v[19] + v[14] * v[3];
   v[20] = v[12] + -0.16 * v[24];
   v[4] = 0 - v[20];
   v[23] = v[12] - v[20];
   v[22] = 0.19 * (0 - v[32]) + v[32] * v[0] + v[21] * v[15];
   v[13] = v[32] * v[19] + v[21] * v[3];
   v[5] = v[22] + -0.16 * v[13];
   v[28] = v[22] - v[5];
   v[27] = v[23] * v[23] + v[28] * v[28];
   v[6] = 0.158 * v[28];
   v[11] = 0.024964 * v[27] - v[6] * v[6];
   v[18] = -0.158 - v[5];
   v[33] = v[23] * v[4] + v[28] * v[18];
   v[1] = 0.158 * v[18];
   if( v[11] < 1e-09 ) {
      v[16] = 0;
   } else {
      v[16] = v[6] * v[33] - v[27] * v[1];
   }
   if( v[11] < 1e-09 ) {
      v[29] = 1;
   } else {
      v[29] = v[11];
   }
   if( v[11] < 1e-09 ) {
      v[9] = v[33];
   } else {
      v[9] = 0.024964 * v[33] - v[6] * v[1];
   }
   if( v[16] < 0 ) {
      v[9] = v[33];
   } else {
      v[9] = v[9];
   }
   if( v[16] > v[29] ) {
      v[9] = v[33] + v[6];
   } else {
      v[9] = v[9];
   }
   if( v[9] < 0 ) {
      v[33] = 0;
   } else {
      v[33] = v[9];
   }
   if( v[11] < 1e-09 ) {
      v[11] = v[27];
   } else {
      v[11] = v[11];
   }
   if( v[16] < 0 ) {
      v[11] = v[27];
   } else {
      v[11] = v[11];
   }
   if( v[16] > v[29] ) {
      v[11] = v[27];
   } else {
      v[11] = v[11];
   }
   if( v[33] > v[11] ) {
      v[33] = v[11];
   } else {
      v[33] = v[33];
   }
   if( fabs(v[33]) < 1e-09 ) {
      v[33] = 0;
   } else {
      v[33] = v[33] / v[11];
   }
   v[4] = v[4] - v[33] * v[23];
   if( 0 - v[1] > 0.024964 ) {
      v[27] = v[29];
   } else {
      v[27] = 0 - v[1];
   }
   if( 0 - v[1] < 0 ) {
      v[27] = 0;
   } else {
      v[27] = v[27];
   }
   if( 0 - v[1] + v[6] > 0.024964 ) {
      v[17] = v[29];
   } else {
      v[17] = 0 - v[1] + v[6];
   }
   if( 0 - v[1] + v[6] < 0 ) {
      v[17] = 0;
   } else {
      v[17] = v[17];
   }
   if( v[16] < 0 ) {
      v[16] = 0;
   } else {
      v[16] = v[16];
   }
   if( v[16] > v[29] ) {
      v[16] = v[29];
   } else {
      v[16] = v[16];
   }
   if( v[9] > v[11] ) {
      v[16] = v[17];
   } else {
      v[16] = v[16];
   }
   if( v[9] < 0 ) {
      v[16] = v[27];
   } else {
      v[16] = v[16];
   }
   if( 0 - v[1] > 0.024964 ) {
      v[27] = v[29];
   } else {
      v[27] = 0.024964;
   }
   if( 0 - v[1] < 0 ) {
      v[27] = v[29];
   } else {
      v[27] = v[27];
   }
   if( 0 - v[1] + v[6] > 0.024964 ) {
      v[17] = v[29];
   } else {
      v[17] = 0.024964;
   }
   if( 0 - v[1] + v[6] < 0 ) {
      v[17] = v[29];
   } else {
      v[17] = v[17];
   }
   if( v[9] > v[11] ) {
      v[17] = v[17];
   } else {
      v[17] = v[29];
   }
   if( v[9] < 0 ) {
      v[17] = v[27];
   } else {
      v[17] = v[17];
   }
   if( fabs(v[16]) < 1e-09 ) {
      v[17] = 0;
   } else {
      v[17] = v[16] / v[17];
   }
   v[18] = v[18] + 0.158 * v[17] - v[33] * v[28];
   v[18] = v[4] * v[4] + 0.000838102500000001 + v[18] * v[18];
   y[27] = sqrt(v[18]) - 0.04;
   v[10] = 0 - v[10];
   v[4] = v[8] * v[2] + v[30] * v[10];
   v[10] = v[25] * v[2] + v[8] * v[10];
   v[2] = v[21] * v[4] + v[14] * v[10];
   v[16] = 0 - v[24];
   v[27] = v[32] * v[4] + v[21] * v[10];
   v[9] = 1 / (v[13] * v[2] + v[16] * v[27]);
   v[23] = v[20] + v[33] * v[23];
   v[18] = sqrt(v[18]);
   v[20] = v[23] / v[18];
   v[23] = v[23] - 0.015 * v[20];
   v[33] = v[5] + v[33] * v[28];
   v[17] = -0.158 + 0.158 * v[17];
   v[28] = (v[33] - v[17]) / v[18];
   v[33] = v[33] - 0.015 * v[28];
   v[16] = 0 - v[12] + v[13] * v[9] * v[23] + v[16] * v[9] * v[33];
   v[33] = 0 - v[22] + (0 - v[27]) * v[9] * v[23] + v[2] * v[9] * v[33];
   v[23] = v[0] + v[4] * v[16] + v[19] * v[33];
   v[33] = v[15] + v[10] * v[16] + v[3] * v[33];
   v[20] = 0.025 * v[20];
   v[28] = v[17] + 0.025 * v[28];
   v[17] = 0.19 + v[21] * v[20] + v[32] * v[28];
   v[28] = v[14] * v[20] + v[21] * v[28];
   v[20] = v[0] + v[4] * v[23] + v[19] * v[33] - (0.19 + v[21] * v[17] + v[32] * v[28]);
   v[16] = 0 - (0 - v[28]);
   v[22] = 0 - v[17];
   v[28] = v[15] + v[10] * v[23] + v[3] * v[33] - (v[14] * v[17] + v[21] * v[28]);
   v[18] = -0.02895 / v[18];
   v[18] = -0.14205 + -0.14205 + 0.03745 + (v[13] * v[2] - v[27] * v[24]) * v[9] * (-0.04545 - 0.015 * v[18]) - (-0.1046 + -0.1046 + -0.0165 + 0.025 * v[18]);
   v[18] = sqrt(v[20] * v[20] + v[18] * v[18] + v[28] * v[28]);
   y[30] = (v[20] * (0 - (v[21] * v[16] + v[32] * v[22])) + v[28] * (0 - (v[14] * v[16] + v[21] * v[22]))) / v[18];
   v[33] = 0 - v[33];
   v[22] = -0.16 * cos(x[7]) - v[33];
   v[16] = -0.16 * sin(x[7]) - v[23];
   y[34] = (v[20] * (v[4] * v[22] + v[19] * v[16]) + v[28] * (v[10] * v[22] + v[3] * v[16])) / v[18];
   v[33] = 0 - v[33];
   v[23] = 0 - v[23];
   y[35] = (v[20] * (v[4] * v[33] + v[19] * v[23]) + v[28] * (v[10] * v[33] + v[3] * v[23])) / v[18];
   v[23] = cos(x[3]);
   v[33] = sin(x[3]);
   v[18] = 0 - v[33];
   v[28] = v[21] * v[23] + v[32] * v[18];
   v[20] = 0.19 + -0.16 * v[32];
   v[18] = v[14] * v[23] + v[21] * v[18];
   v[16] = -0.16 * v[21];
   v[22] = (0 - v[28]) * v[20] + (0 - v[18]) * v[16] + -0.19 * v[28];
   v[9] = v[28] * v[30] + v[18] * v[8];
   v[27] = v[22] + -0.158 * v[9];
   v[2] = 0 - v[27];
   v[13] = v[22] - v[27];
   v[32] = v[21] * v[33] + v[32] * v[23];
   v[33] = v[14] * v[33] + v[21] * v[23];
   v[23] = (0 - v[32]) * v[20] + (0 - v[33]) * v[16] + -0.19 * v[32];
   v[14] = v[32] * v[30] + v[33] * v[8];
   v[21] = v[23] + -0.158 * v[14];
   v[24] = v[23] - v[21];
   v[17] = v[13] * v[13] + v[24] * v[24];
   v[12] = 0.16 * v[24];
   v[5] = 0.0256 * v[17] - v[12] * v[12];
   v[11] = -0.16 - v[21];
   v[29] = v[13] * v[2] + v[24] * v[11];
   v[1] = 0.16 * v[11];
   if( v[5] < 1e-09 ) {
      v[6] = 0;
   } else {
      v[6] = v[12] * v[29] - v[17] * v[1];
   }
   if( v[5] < 1e-09 ) {
      v[31] = 1;
   } else {
      v[31] = v[5];
   }
   if( v[5] < 1e-09 ) {
      v[26] = v[29];
   } else {
      v[26] = 0.0256 * v[29] - v[12] * v[1];
   }
   if( v[6] < 0 ) {
      v[26] = v[29];
   } else {
      v[26] = v[26];
   }
   if( v[6] > v[31] ) {
      v[26] = v[29] + v[12];
   } else {
      v[26] = v[26];
   }
   if( v[26] < 0 ) {
      v[29] = 0;
   } else {
      v[29] = v[26];
   }
   if( v[5] < 1e-09 ) {
      v[5] = v[17];
   } else {
      v[5] = v[5];
   }
   if( v[6] < 0 ) {
      v[5] = v[17];
   } else {
      v[5] = v[5];
   }
   if( v[6] > v[31] ) {
      v[5] = v[17];
   } else {
      v[5] = v[5];
   }
   if( v[29] > v[5] ) {
      v[29] = v[5];
   } else {
      v[29] = v[29];
   }
   if( fabs(v[29]) < 1e-09 ) {
      v[29] = 0;
   } else {
      v[29] = v[29] / v[5];
   }
   v[2] = v[2] - v[29] * v[13];
   if( 0 - v[1] > 0.0256 ) {
      v[17] = v[31];
   } else {
      v[17] = 0 - v[1];
   }
   if( 0 - v[1] < 0 ) {
      v[17] = 0;
   } else {
      v[17] = v[17];
   }
   if( 0 - v[1] + v[12] > 0.0256 ) {
      v[7] = v[31];
   } else {
      v[7] = 0 - v[1] + v[12];
   }
   if( 0 - v[1] + v[12] < 0 ) {
      v[7] = 0;
   } else {
      v[7] = v[7];
   }
   if( v[6] < 0 ) {
      v[6] = 0;
   } else {
      v[6] = v[6];
   }
   if( v[6] > v[31] ) {
      v[6] = v[31];
   } else {
      v[6] = v[6];
   }
   if( v[26] > v[5] ) {
      v[6] = v[7];
   } else {
      v[6] = v[6];
   }
   if( v[26] < 0 ) {
      v[6] = v[17];
   } else {
      v[6] = v[6];
   }
   if( 0 - v[1] > 0.0256 ) {
      v[17] = v[31];
   } else {
      v[17] = 0.0256;
   }
   if( 0 - v[1] < 0 ) {
      v[17] = v[31];
   } else {
      v[17] = v[17];
   }
   if( 0 - v[1] + v[12] > 0.0256 ) {
      v[7] = v[31];
   } else {
      v[7] = 0.0256;
   }
   if( 0 - v[1] + v[12] < 0 ) {
      v[7] = v[31];
   } else {
      v[7] = v[7];
   }
   if( v[26] > v[5] ) {
      v[7] = v[7];
   } else {
      v[7] = v[31];
   }
   if( v[26] < 0 ) {
      v[7] = v[17];
   } else {
      v[7] = v[7];
   }
   if( fabs(v[6]) < 1e-09 ) {
      v[7] = 0;
   } else {
      v[7] = v[6] / v[7];
   }
   v[11] = v[11] + 0.16 * v[7] - v[29] * v[24];
   v[11] = v[2] * v[2] + 0.000838102500000001 + v[11] * v[11];
   y[36] = sqrt(v[11]) - 0.04;
   v[2] = v[28] * v[8] + v[18] * v[25];
   v[6] = 0 - v[9];
   v[17] = v[32] * v[8] + v[33] * v[25];
   v[26] = 1 / (v[14] * v[2] + v[6] * v[17]);
   v[13] = v[27] + v[29] * v[13];
   v[11] = sqrt(v[11]);
   v[27] = v[13] / v[11];
   v[13] = v[13] - 0.025 * v[27];
   v[29] = v[21] + v[29] * v[24];
   v[7] = -0.16 + 0.16 * v[7];
   v[24] = (v[29] - v[7]) / v[11];
   v[29] = v[29] - 0.025 * v[24];
   v[6] = 0 - v[22] + v[14] * v[26] * v[13] + v[6] * v[26] * v[29];
   v[29] = 0 - v[23] + (0 - v[17]) * v[26] * v[13] + v[2] * v[26] * v[29];
   v[13] = -0.19 + v[8] * v[6] + v[30] * v[29];
   v[29] = v[25] * v[6] + v[8] * v[29];
   v[27] = 0.015 * v[27];
   v[24] = v[7] + 0.015 * v[24];
   v[7] = v[20] + v[28] * v[27] + v[32] * v[24];
   v[24] = v[16] + v[18] * v[27] + v[33] * v[24];
   v[27] = -0.19 + v[8] * v[13] + v[30] * v[29] - (v[20] + v[28] * v[7] + v[32] * v[24]);
   v[6] = 0 - v[24];
   v[23] = -0.16 * cos(x[3]) - v[6];
   v[22] = -0.16 * sin(x[3]) - v[7];
   v[24] = v[25] * v[13] + v[8] * v[29] - (v[16] + v[18] * v[7] + v[33] * v[24]);
   v[11] = 0.02895 / v[11];
   v[11] = -0.1046 + -0.1046 + -0.03745 + (v[14] * v[2] - v[17] * v[9]) * v[26] * (0.02095 - 0.025 * v[11]) - (-0.14205 + -0.14205 + -0.008 + 0.015 * v[11]);
   v[11] = sqrt(v[27] * v[27] + v[11] * v[11] + v[24] * v[24]);
   y[39] = (v[27] * (0 - (v[28] * v[23] + v[32] * v[22])) + v[24] * (0 - (v[18] * v[23] + v[33] * v[22]))) / v[11];
   v[6] = 0 - v[6];
   v[7] = 0 - v[7];
   y[40] = (v[27] * (0 - (v[28] * v[6] + v[32] * v[7])) + v[24] * (0 - (v[18] * v[6] + v[33] * v[7]))) / v[11];
   v[29] = 0 - (0 - v[29]);
   v[13] = 0 - v[13];
   y[43] = (v[27] * (v[8] * v[29] + v[30] * v[13]) + v[24] * (v[25] * v[29] + v[8] * v[13])) / v[11];
   v[13] = (0 - v[28]) * v[20] + (0 - v[18]) * v[16] + v[28] * v[0] + v[18] * v[15];
   v[29] = v[28] * v[19] + v[18] * v[3];
   v[11] = v[13] + -0.16 * v[29];
   v[24] = 0 - v[11];
   v[27] = v[13] - v[11];
   v[25] = (0 - v[32]) * v[20] + (0 - v[33]) * v[16] + v[32] * v[0] + v[33] * v[15];
   v[8] = v[32] * v[19] + v[33] * v[3];
   v[30] = v[25] + -0.16 * v[8];
   v[7] = v[25] - v[30];
   v[6] = v[27] * v[27] + v[7] * v[7];
   v[22] = 0.16 * v[7];
   v[23] = 0.0256 * v[6] - v[22] * v[22];
   v[26] = -0.16 - v[30];
   v[17] = v[27] * v[24] + v[7] * v[26];
   v[2] = 0.16 * v[26];
   if( v[23] < 1e-09 ) {
      v[14] = 0;
   } else {
      v[14] = v[22] * v[17] - v[6] * v[2];
   }
   if( v[23] < 1e-09 ) {
      v[9] = 1;
   } else {
      v[9] = v[23];
   }
   if( v[23] < 1e-09 ) {
      v[21] = v[17];
   } else {
      v[21] = 0.0256 * v[17] - v[22] * v[2];
   }
   if( v[14] < 0 ) {
      v[21] = v[17];
   } else {
      v[21] = v[21];
   }
   if( v[14] > v[9] ) {
      v[21] = v[17] + v[22];
   } else {
      v[21] = v[21];
   }
   if( v[21] < 0 ) {
      v[17] = 0;
   } else {
      v[17] = v[21];
   }
   if( v[23] < 1e-09 ) {
      v[23] = v[6];
   } else {
      v[23] = v[23];
   }
   if( v[14] < 0 ) {
      v[23] = v[6];
   } else {
      v[23] = v[23];
   }
   if( v[14] > v[9] ) {
      v[23] = v[6];
   } else {
      v[23] = v[23];
   }
   if( v[17] > v[23] ) {
      v[17] = v[23];
   } else {
      v[17] = v[17];
   }
   if( fabs(v[17]) < 1e-09 ) {
      v[17] = 0;
   } else {
      v[17] = v[17] / v[23];
   }
   v[24] = v[24] - v[17] * v[27];
   if( 0 - v[2] > 0.0256 ) {
      v[6] = v[9];
   } else {
      v[6] = 0 - v[2];
   }
   if( 0 - v[2] < 0 ) {
      v[6] = 0;
   } else {
      v[6] = v[6];
   }
   if( 0 - v[2] + v[22] > 0.0256 ) {
      v[5] = v[9];
   } else {
      v[5] = 0 - v[2] + v[22];
   }
   if( 0 - v[2] + v[22] < 0 ) {
      v[5] = 0;
   } else {
      v[5] = v[5];
   }
   if( v[14] < 0 ) {
      v[14] = 0;
   } else {
      v[14] = v[14];
   }
   if( v[14] > v[9] ) {
      v[14] = v[9];
   } else {
      v[14] = v[14];
   }
   if( v[21] > v[23] ) {
      v[14] = v[5];
   } else {
      v[14] = v[14];
   }
   if( v[21] < 0 ) {
      v[14] = v[6];
   } else {
      v[14] = v[14];
   }
   if( 0 - v[2] > 0.0256 ) {
      v[6] = v[9];
   } else {
      v[6] = 0.0256;
   }
   if( 0 - v[2] < 0 ) {
      v[6] = v[9];
   } else {
      v[6] = v[6];
   }
   if( 0 - v[2] + v[22] > 0.0256 ) {
      v[5] = v[9];
   } else {
      v[5] = 0.0256;
   }
   if( 0 - v[2] + v[22] < 0 ) {
      v[5] = v[9];
   } else {
      v[5] = v[5];
   }
   if( v[21] > v[23] ) {
      v[5] = v[5];
   } else {
      v[5] = v[9];
   }
   if( v[21] < 0 ) {
      v[5] = v[6];
   } else {
      v[5] = v[5];
   }
   if( fabs(v[14]) < 1e-09 ) {
      v[5] = 0;
   } else {
      v[5] = v[14] / v[5];
   }
   v[26] = v[26] + 0.16 * v[5] - v[17] * v[7];
   v[26] = v[24] * v[24] + v[26] * v[26];
   y[45] = sqrt(v[26]) - 0.03;
   v[24] = v[28] * v[4] + v[18] * v[10];
   v[14] = 0 - v[29];
   v[6] = v[32] * v[4] + v[33] * v[10];
   v[21] = 1 / (v[8] * v[24] + v[14] * v[6]);
   v[27] = v[11] + v[17] * v[27];
   v[26] = sqrt(v[26]);
   v[11] = v[27] / v[26];
   v[27] = v[27] - 0.015 * v[11];
   v[17] = v[30] + v[17] * v[7];
   v[5] = -0.16 + 0.16 * v[5];
   v[26] = (v[17] - v[5]) / v[26];
   v[17] = v[17] - 0.015 * v[26];
   v[14] = 0 - v[13] + v[8] * v[21] * v[27] + v[14] * v[21] * v[17];
   v[17] = 0 - v[25] + (0 - v[6]) * v[21] * v[27] + v[24] * v[21] * v[17];
   v[27] = v[0] + v[4] * v[14] + v[19] * v[17];
   v[17] = v[15] + v[10] * v[14] + v[3] * v[17];
   v[11] = 0.015 * v[11];
   v[26] = v[5] + 0.015 * v[26];
   v[5] = v[20] + v[28] * v[11] + v[32] * v[26];
   v[26] = v[16] + v[18] * v[11] + v[33] * v[26];
   v[20] = v[0] + v[4] * v[27] + v[19] * v[17] - (v[20] + v[28] * v[5] + v[32] * v[26]);
   v[0] = 0 - v[26];
   v[11] = -0.16 * cos(x[3]) - v[0];
   v[14] = -0.16 * sin(x[3]) - v[5];
   v[26] = v[15] + v[10] * v[27] + v[3] * v[17] - (v[16] + v[18] * v[5] + v[33] * v[26]);
   v[21] = -0.14205 + -0.14205 + -0.008 * (v[8] * v[24] - v[6] * v[29]) * v[21] + 0.2921;
   v[21] = sqrt(v[20] * v[20] + v[21] * v[21] + v[26] * v[26]);
   y[48] = (v[20] * (0 - (v[28] * v[11] + v[32] * v[14])) + v[26] * (0 - (v[18] * v[11] + v[33] * v[14]))) / v[21];
   v[0] = 0 - v[0];
   v[5] = 0 - v[5];
   y[49] = (v[20] * (0 - (v[28] * v[0] + v[32] * v[5])) + v[26] * (0 - (v[18] * v[0] + v[33] * v[5]))) / v[21];
   v[17] = 0 - v[17];
   v[5] = -0.16 * cos(x[7]) - v[17];
   v[0] = -0.16 * sin(x[7]) - v[27];
   y[52] = (v[20] * (v[4] * v[5] + v[19] * v[0]) + v[26] * (v[10] * v[5] + v[3] * v[0])) / v[21];
   v[17] = 0 - v[17];
   v[27] = 0 - v[27];
   y[53] = (v[20] * (v[4] * v[17] + v[19] * v[27]) + v[26] * (v[10] * v[17] + v[3] * v[27])) / v[21];
   // dependent variables without operations
   y[2] = 0;
   y[3] = 0;
   y[4] = 0;
   y[7] = 0;
   y[8] = 0;
   y[12] = 0;
   y[13] = 0;
   y[15] = 0;
   y[16] = 0;
   y[17] = 0;
   y[21] = 0;
   y[22] = 0;
   y[25] = 0;
   y[26] = 0;
   y[28] = -0;
   y[29] = -0;
   y[31] = -0;
   y[32] = -0;
   y[33] = -0;
   y[37] = 0;
   y[38] = 0;
   y[41] = 0;
   y[42] = 0;
   y[44] = 0;
   y[46] = 0;
   y[47] = 0;
   y[50] = 0;
   y[51] = 0;
}
