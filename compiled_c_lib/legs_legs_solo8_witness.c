#include <math.h>
#include <stdio.h>

void solo_autocollision_legs_legs_forward_zero(double* x, double* y) {
   // auxiliary variables
   double v[30];

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
   v[13] = v[7] + -0.16 * (v[0] * v[10] + v[4] * v[12]);
   v[14] = 0 - v[13];
   v[7] = v[7] - v[13];
   v[15] = 0.19 * (0 - v[3]) + v[3] * v[2] + v[0] * v[6];
   v[16] = v[15] + -0.16 * (v[3] * v[10] + v[0] * v[12]);
   v[15] = v[15] - v[16];
   v[17] = v[7] * v[7] + v[15] * v[15];
   v[18] = 0.158 * v[15];
   v[19] = 0.024964 * v[17] - v[18] * v[18];
   v[20] = -0.158 - v[16];
   v[21] = v[7] * v[14] + v[15] * v[20];
   v[22] = 0.158 * v[20];
   if( v[19] < 1e-09 ) {
      v[23] = 0;
   } else {
      v[23] = v[18] * v[21] - v[17] * v[22];
   }
   if( v[19] < 1e-09 ) {
      v[24] = 1;
   } else {
      v[24] = v[19];
   }
   if( v[19] < 1e-09 ) {
      v[25] = v[21];
   } else {
      v[25] = 0.024964 * v[21] - v[18] * v[22];
   }
   if( v[23] < 0 ) {
      v[25] = v[21];
   } else {
      v[25] = v[25];
   }
   if( v[23] > v[24] ) {
      v[25] = v[21] + v[18];
   } else {
      v[25] = v[25];
   }
   if( v[25] < 0 ) {
      v[21] = 0;
   } else {
      v[21] = v[25];
   }
   if( v[19] < 1e-09 ) {
      v[19] = v[17];
   } else {
      v[19] = v[19];
   }
   if( v[23] < 0 ) {
      v[19] = v[17];
   } else {
      v[19] = v[19];
   }
   if( v[23] > v[24] ) {
      v[19] = v[17];
   } else {
      v[19] = v[19];
   }
   if( v[21] > v[19] ) {
      v[21] = v[19];
   } else {
      v[21] = v[21];
   }
   if( fabs(v[21]) < 1e-09 ) {
      v[21] = 0;
   } else {
      v[21] = v[21] / v[19];
   }
   v[14] = v[14] - v[21] * v[7];
   if( 0 - v[22] > 0.024964 ) {
      v[17] = v[24];
   } else {
      v[17] = 0 - v[22];
   }
   if( 0 - v[22] < 0 ) {
      v[17] = 0;
   } else {
      v[17] = v[17];
   }
   if( 0 - v[22] + v[18] > 0.024964 ) {
      v[26] = v[24];
   } else {
      v[26] = 0 - v[22] + v[18];
   }
   if( 0 - v[22] + v[18] < 0 ) {
      v[26] = 0;
   } else {
      v[26] = v[26];
   }
   if( v[23] < 0 ) {
      v[23] = 0;
   } else {
      v[23] = v[23];
   }
   if( v[23] > v[24] ) {
      v[23] = v[24];
   } else {
      v[23] = v[23];
   }
   if( v[25] > v[19] ) {
      v[23] = v[26];
   } else {
      v[23] = v[23];
   }
   if( v[25] < 0 ) {
      v[23] = v[17];
   } else {
      v[23] = v[23];
   }
   if( 0 - v[22] > 0.024964 ) {
      v[17] = v[24];
   } else {
      v[17] = 0.024964;
   }
   if( 0 - v[22] < 0 ) {
      v[17] = v[24];
   } else {
      v[17] = v[17];
   }
   if( 0 - v[22] + v[18] > 0.024964 ) {
      v[26] = v[24];
   } else {
      v[26] = 0.024964;
   }
   if( 0 - v[22] + v[18] < 0 ) {
      v[26] = v[24];
   } else {
      v[26] = v[26];
   }
   if( v[25] > v[19] ) {
      v[26] = v[26];
   } else {
      v[26] = v[24];
   }
   if( v[25] < 0 ) {
      v[26] = v[17];
   } else {
      v[26] = v[26];
   }
   if( fabs(v[23]) < 1e-09 ) {
      v[26] = 0;
   } else {
      v[26] = v[23] / v[26];
   }
   v[20] = v[20] + 0.158 * v[26] - v[21] * v[15];
   v[20] = v[14] * v[14] + 0.000838102500000001 + v[20] * v[20];
   y[0] = sqrt(v[20]) - 0.04;
   v[8] = 0 - v[8];
   v[14] = v[5] * v[9] + v[1] * v[8];
   v[8] = v[11] * v[9] + v[5] * v[8];
   v[7] = v[13] + v[21] * v[7];
   v[20] = sqrt(v[20]);
   v[13] = v[7] / v[20];
   v[7] = v[7] - 0.015 * v[13];
   v[21] = v[16] + v[21] * v[15];
   v[26] = -0.158 + 0.158 * v[26];
   v[15] = (v[21] - v[26]) / v[20];
   v[21] = v[21] - 0.015 * v[15];
   y[12] = (0 - v[14]) * v[2] + (0 - v[8]) * v[6] + 0.19 * v[14] + (v[14] * v[0] + v[8] * v[4]) * v[7] + (v[14] * v[3] + v[8] * v[0]) * v[21];
   y[14] = (0 - v[10]) * v[2] + (0 - v[12]) * v[6] + 0.19 * v[10] + (v[10] * v[0] + v[12] * v[4]) * v[7] + (v[10] * v[3] + v[12] * v[0]) * v[21];
   y[9] = 0.025 * v[13];
   y[11] = v[26] + 0.025 * v[15];
   v[15] = v[2] + v[14] * y[12] + v[10] * y[14] - (0.19 + v[0] * y[9] + v[3] * y[11]);
   v[26] = 0 - (0 - y[11]);
   v[13] = 0 - y[9];
   v[21] = v[6] + v[8] * y[12] + v[12] * y[14] - (v[4] * y[9] + v[0] * y[11]);
   v[20] = 0.02895 / v[20];
   y[13] = -0.03745 + 0.04545 - 0.015 * v[20];
   y[10] = 0.0165 + 0.025 * v[20];
   v[20] = 0.14205 + y[13] - (0.1046 + y[10]);
   v[20] = sqrt(v[15] * v[15] + v[20] * v[20] + v[21] * v[21]);
   y[1] = (v[15] * (0 - (v[0] * v[26] + v[3] * v[13])) + v[21] * (0 - (v[4] * v[26] + v[0] * v[13]))) / v[20];
   v[13] = 0 - y[14];
   v[26] = -0.16 * cos(x[5]) - v[13];
   v[7] = -0.16 * sin(x[5]) - y[12];
   y[5] = (v[15] * (v[14] * v[26] + v[10] * v[7]) + v[21] * (v[8] * v[26] + v[12] * v[7])) / v[20];
   v[13] = 0 - v[13];
   v[7] = 0 - y[12];
   y[6] = (v[15] * (v[14] * v[13] + v[10] * v[7]) + v[21] * (v[8] * v[13] + v[12] * v[7])) / v[20];
   v[7] = cos(x[1]);
   v[13] = sin(x[1]);
   v[20] = 0 - v[13];
   v[21] = v[0] * v[7] + v[3] * v[20];
   v[15] = 0.19 + -0.16 * v[3];
   v[20] = v[4] * v[7] + v[0] * v[20];
   v[26] = -0.16 * v[0];
   v[16] = (0 - v[21]) * v[15] + (0 - v[20]) * v[26] + -0.19 * v[21];
   v[9] = v[16] + -0.158 * (v[21] * v[1] + v[20] * v[5]);
   v[23] = 0 - v[9];
   v[16] = v[16] - v[9];
   v[3] = v[0] * v[13] + v[3] * v[7];
   v[13] = v[4] * v[13] + v[0] * v[7];
   v[7] = (0 - v[3]) * v[15] + (0 - v[13]) * v[26] + -0.19 * v[3];
   v[4] = v[7] + -0.158 * (v[3] * v[1] + v[13] * v[5]);
   v[7] = v[7] - v[4];
   v[0] = v[16] * v[16] + v[7] * v[7];
   v[17] = 0.16 * v[7];
   v[25] = 0.0256 * v[0] - v[17] * v[17];
   v[19] = -0.16 - v[4];
   v[24] = v[16] * v[23] + v[7] * v[19];
   v[22] = 0.16 * v[19];
   if( v[25] < 1e-09 ) {
      v[18] = 0;
   } else {
      v[18] = v[17] * v[24] - v[0] * v[22];
   }
   if( v[25] < 1e-09 ) {
      v[27] = 1;
   } else {
      v[27] = v[25];
   }
   if( v[25] < 1e-09 ) {
      v[28] = v[24];
   } else {
      v[28] = 0.0256 * v[24] - v[17] * v[22];
   }
   if( v[18] < 0 ) {
      v[28] = v[24];
   } else {
      v[28] = v[28];
   }
   if( v[18] > v[27] ) {
      v[28] = v[24] + v[17];
   } else {
      v[28] = v[28];
   }
   if( v[28] < 0 ) {
      v[24] = 0;
   } else {
      v[24] = v[28];
   }
   if( v[25] < 1e-09 ) {
      v[25] = v[0];
   } else {
      v[25] = v[25];
   }
   if( v[18] < 0 ) {
      v[25] = v[0];
   } else {
      v[25] = v[25];
   }
   if( v[18] > v[27] ) {
      v[25] = v[0];
   } else {
      v[25] = v[25];
   }
   if( v[24] > v[25] ) {
      v[24] = v[25];
   } else {
      v[24] = v[24];
   }
   if( fabs(v[24]) < 1e-09 ) {
      v[24] = 0;
   } else {
      v[24] = v[24] / v[25];
   }
   v[23] = v[23] - v[24] * v[16];
   if( 0 - v[22] > 0.0256 ) {
      v[0] = v[27];
   } else {
      v[0] = 0 - v[22];
   }
   if( 0 - v[22] < 0 ) {
      v[0] = 0;
   } else {
      v[0] = v[0];
   }
   if( 0 - v[22] + v[17] > 0.0256 ) {
      v[29] = v[27];
   } else {
      v[29] = 0 - v[22] + v[17];
   }
   if( 0 - v[22] + v[17] < 0 ) {
      v[29] = 0;
   } else {
      v[29] = v[29];
   }
   if( v[18] < 0 ) {
      v[18] = 0;
   } else {
      v[18] = v[18];
   }
   if( v[18] > v[27] ) {
      v[18] = v[27];
   } else {
      v[18] = v[18];
   }
   if( v[28] > v[25] ) {
      v[18] = v[29];
   } else {
      v[18] = v[18];
   }
   if( v[28] < 0 ) {
      v[18] = v[0];
   } else {
      v[18] = v[18];
   }
   if( 0 - v[22] > 0.0256 ) {
      v[0] = v[27];
   } else {
      v[0] = 0.0256;
   }
   if( 0 - v[22] < 0 ) {
      v[0] = v[27];
   } else {
      v[0] = v[0];
   }
   if( 0 - v[22] + v[17] > 0.0256 ) {
      v[29] = v[27];
   } else {
      v[29] = 0.0256;
   }
   if( 0 - v[22] + v[17] < 0 ) {
      v[29] = v[27];
   } else {
      v[29] = v[29];
   }
   if( v[28] > v[25] ) {
      v[29] = v[29];
   } else {
      v[29] = v[27];
   }
   if( v[28] < 0 ) {
      v[29] = v[0];
   } else {
      v[29] = v[29];
   }
   if( fabs(v[18]) < 1e-09 ) {
      v[29] = 0;
   } else {
      v[29] = v[18] / v[29];
   }
   v[19] = v[19] + 0.16 * v[29] - v[24] * v[7];
   v[19] = v[23] * v[23] + 0.000838102500000001 + v[19] * v[19];
   y[15] = sqrt(v[19]) - 0.04;
   v[16] = v[9] + v[24] * v[16];
   v[19] = sqrt(v[19]);
   v[9] = v[16] / v[19];
   v[16] = v[16] - 0.025 * v[9];
   v[24] = v[4] + v[24] * v[7];
   v[29] = -0.16 + 0.16 * v[29];
   v[7] = (v[24] - v[29]) / v[19];
   v[24] = v[24] - 0.025 * v[7];
   y[27] = -0.19 * (0 - v[5]) + v[5] * v[15] + v[11] * v[26] + (v[5] * v[21] + v[11] * v[20]) * v[16] + (v[5] * v[3] + v[11] * v[13]) * v[24];
   y[29] = -0.19 * (0 - v[1]) + v[1] * v[15] + v[5] * v[26] + (v[1] * v[21] + v[5] * v[20]) * v[16] + (v[1] * v[3] + v[5] * v[13]) * v[24];
   y[24] = 0.015 * v[9];
   y[26] = v[29] + 0.015 * v[7];
   v[7] = -0.19 + v[5] * y[27] + v[1] * y[29] - (v[15] + v[21] * y[24] + v[3] * y[26]);
   v[29] = 0 - y[26];
   v[9] = -0.16 * cos(x[1]) - v[29];
   v[24] = -0.16 * sin(x[1]) - y[24];
   v[16] = v[11] * y[27] + v[5] * y[29] - (v[26] + v[20] * y[24] + v[13] * y[26]);
   v[19] = -0.02895 / v[19];
   y[28] = 0.03745 + -0.02095 - 0.025 * v[19];
   y[25] = 0.008 + 0.015 * v[19];
   v[19] = 0.1046 + y[28] - (0.14205 + y[25]);
   v[19] = sqrt(v[7] * v[7] + v[19] * v[19] + v[16] * v[16]);
   y[16] = (v[7] * (0 - (v[21] * v[9] + v[3] * v[24])) + v[16] * (0 - (v[20] * v[9] + v[13] * v[24]))) / v[19];
   v[29] = 0 - v[29];
   v[24] = 0 - y[24];
   y[17] = (v[7] * (0 - (v[21] * v[29] + v[3] * v[24])) + v[16] * (0 - (v[20] * v[29] + v[13] * v[24]))) / v[19];
   v[24] = 0 - (0 - y[29]);
   v[29] = 0 - y[27];
   y[20] = (v[7] * (v[5] * v[24] + v[1] * v[29]) + v[16] * (v[11] * v[24] + v[5] * v[29])) / v[19];
   v[29] = (0 - v[21]) * v[15] + (0 - v[20]) * v[26] + v[21] * v[2] + v[20] * v[6];
   v[24] = v[29] + -0.16 * (v[21] * v[10] + v[20] * v[12]);
   v[19] = 0 - v[24];
   v[29] = v[29] - v[24];
   v[16] = (0 - v[3]) * v[15] + (0 - v[13]) * v[26] + v[3] * v[2] + v[13] * v[6];
   v[7] = v[16] + -0.16 * (v[3] * v[10] + v[13] * v[12]);
   v[16] = v[16] - v[7];
   v[11] = v[29] * v[29] + v[16] * v[16];
   v[5] = 0.16 * v[16];
   v[1] = 0.0256 * v[11] - v[5] * v[5];
   v[9] = -0.16 - v[7];
   v[4] = v[29] * v[19] + v[16] * v[9];
   v[23] = 0.16 * v[9];
   if( v[1] < 1e-09 ) {
      v[18] = 0;
   } else {
      v[18] = v[5] * v[4] - v[11] * v[23];
   }
   if( v[1] < 1e-09 ) {
      v[0] = 1;
   } else {
      v[0] = v[1];
   }
   if( v[1] < 1e-09 ) {
      v[28] = v[4];
   } else {
      v[28] = 0.0256 * v[4] - v[5] * v[23];
   }
   if( v[18] < 0 ) {
      v[28] = v[4];
   } else {
      v[28] = v[28];
   }
   if( v[18] > v[0] ) {
      v[28] = v[4] + v[5];
   } else {
      v[28] = v[28];
   }
   if( v[28] < 0 ) {
      v[4] = 0;
   } else {
      v[4] = v[28];
   }
   if( v[1] < 1e-09 ) {
      v[1] = v[11];
   } else {
      v[1] = v[1];
   }
   if( v[18] < 0 ) {
      v[1] = v[11];
   } else {
      v[1] = v[1];
   }
   if( v[18] > v[0] ) {
      v[1] = v[11];
   } else {
      v[1] = v[1];
   }
   if( v[4] > v[1] ) {
      v[4] = v[1];
   } else {
      v[4] = v[4];
   }
   if( fabs(v[4]) < 1e-09 ) {
      v[4] = 0;
   } else {
      v[4] = v[4] / v[1];
   }
   v[19] = v[19] - v[4] * v[29];
   if( 0 - v[23] > 0.0256 ) {
      v[11] = v[0];
   } else {
      v[11] = 0 - v[23];
   }
   if( 0 - v[23] < 0 ) {
      v[11] = 0;
   } else {
      v[11] = v[11];
   }
   if( 0 - v[23] + v[5] > 0.0256 ) {
      v[25] = v[0];
   } else {
      v[25] = 0 - v[23] + v[5];
   }
   if( 0 - v[23] + v[5] < 0 ) {
      v[25] = 0;
   } else {
      v[25] = v[25];
   }
   if( v[18] < 0 ) {
      v[18] = 0;
   } else {
      v[18] = v[18];
   }
   if( v[18] > v[0] ) {
      v[18] = v[0];
   } else {
      v[18] = v[18];
   }
   if( v[28] > v[1] ) {
      v[18] = v[25];
   } else {
      v[18] = v[18];
   }
   if( v[28] < 0 ) {
      v[18] = v[11];
   } else {
      v[18] = v[18];
   }
   if( 0 - v[23] > 0.0256 ) {
      v[11] = v[0];
   } else {
      v[11] = 0.0256;
   }
   if( 0 - v[23] < 0 ) {
      v[11] = v[0];
   } else {
      v[11] = v[11];
   }
   if( 0 - v[23] + v[5] > 0.0256 ) {
      v[25] = v[0];
   } else {
      v[25] = 0.0256;
   }
   if( 0 - v[23] + v[5] < 0 ) {
      v[25] = v[0];
   } else {
      v[25] = v[25];
   }
   if( v[28] > v[1] ) {
      v[25] = v[25];
   } else {
      v[25] = v[0];
   }
   if( v[28] < 0 ) {
      v[25] = v[11];
   } else {
      v[25] = v[25];
   }
   if( fabs(v[18]) < 1e-09 ) {
      v[25] = 0;
   } else {
      v[25] = v[18] / v[25];
   }
   v[9] = v[9] + 0.16 * v[25] - v[4] * v[16];
   v[9] = v[19] * v[19] + v[9] * v[9];
   y[30] = sqrt(v[9]) - 0.03;
   v[29] = v[24] + v[4] * v[29];
   v[9] = sqrt(v[9]);
   v[24] = v[29] / v[9];
   v[29] = v[29] - 0.015 * v[24];
   v[4] = v[7] + v[4] * v[16];
   v[25] = -0.16 + 0.16 * v[25];
   v[9] = (v[4] - v[25]) / v[9];
   v[4] = v[4] - 0.015 * v[9];
   y[42] = (0 - v[14]) * v[2] + (0 - v[8]) * v[6] + v[14] * v[15] + v[8] * v[26] + (v[14] * v[21] + v[8] * v[20]) * v[29] + (v[14] * v[3] + v[8] * v[13]) * v[4];
   y[44] = (0 - v[10]) * v[2] + (0 - v[12]) * v[6] + v[10] * v[15] + v[12] * v[26] + (v[10] * v[21] + v[12] * v[20]) * v[29] + (v[10] * v[3] + v[12] * v[13]) * v[4];
   y[39] = 0.015 * v[24];
   y[41] = v[25] + 0.015 * v[9];
   v[15] = v[2] + v[14] * y[42] + v[10] * y[44] - (v[15] + v[21] * y[39] + v[3] * y[41]);
   v[2] = 0 - y[41];
   v[9] = -0.16 * cos(x[1]) - v[2];
   v[25] = -0.16 * sin(x[1]) - y[39];
   v[26] = v[6] + v[8] * y[42] + v[12] * y[44] - (v[26] + v[20] * y[39] + v[13] * y[41]);
   v[6] = sqrt(v[15] * v[15] + v[26] * v[26]);
   y[31] = (v[15] * (0 - (v[21] * v[9] + v[3] * v[25])) + v[26] * (0 - (v[20] * v[9] + v[13] * v[25]))) / v[6];
   v[2] = 0 - v[2];
   v[25] = 0 - y[39];
   y[32] = (v[15] * (0 - (v[21] * v[2] + v[3] * v[25])) + v[26] * (0 - (v[20] * v[2] + v[13] * v[25]))) / v[6];
   v[25] = 0 - y[44];
   v[2] = -0.16 * cos(x[5]) - v[25];
   v[13] = -0.16 * sin(x[5]) - y[42];
   y[35] = (v[15] * (v[14] * v[2] + v[10] * v[13]) + v[26] * (v[8] * v[2] + v[12] * v[13])) / v[6];
   v[25] = 0 - v[25];
   v[13] = 0 - y[42];
   y[36] = (v[15] * (v[14] * v[25] + v[10] * v[13]) + v[26] * (v[8] * v[25] + v[12] * v[13])) / v[6];
   v[13] = cos(x[2]);
   v[25] = sin(x[6]);
   v[6] = -0.19 + -0.16 * v[25];
   v[26] = sin(x[2]);
   v[15] = 0 - v[26];
   v[8] = cos(x[6]);
   v[14] = -0.16 * v[8];
   v[12] = 0.19 * (0 - v[13]) + v[13] * v[6] + v[15] * v[14];
   v[10] = sin(x[7]);
   v[2] = cos(x[7]);
   v[3] = v[8] * v[10] + v[25] * v[2];
   v[20] = 0 - v[25];
   v[21] = v[20] * v[10] + v[8] * v[2];
   v[9] = v[12] + -0.16 * (v[13] * v[3] + v[15] * v[21]);
   v[24] = 0 - v[9];
   v[12] = v[12] - v[9];
   v[4] = 0.19 * (0 - v[26]) + v[26] * v[6] + v[13] * v[14];
   v[29] = v[4] + -0.16 * (v[26] * v[3] + v[13] * v[21]);
   v[4] = v[4] - v[29];
   v[16] = v[12] * v[12] + v[4] * v[4];
   v[7] = 0.158 * v[4];
   v[19] = 0.024964 * v[16] - v[7] * v[7];
   v[18] = -0.158 - v[29];
   v[11] = v[12] * v[24] + v[4] * v[18];
   v[28] = 0.158 * v[18];
   if( v[19] < 1e-09 ) {
      v[1] = 0;
   } else {
      v[1] = v[7] * v[11] - v[16] * v[28];
   }
   if( v[19] < 1e-09 ) {
      v[0] = 1;
   } else {
      v[0] = v[19];
   }
   if( v[19] < 1e-09 ) {
      v[23] = v[11];
   } else {
      v[23] = 0.024964 * v[11] - v[7] * v[28];
   }
   if( v[1] < 0 ) {
      v[23] = v[11];
   } else {
      v[23] = v[23];
   }
   if( v[1] > v[0] ) {
      v[23] = v[11] + v[7];
   } else {
      v[23] = v[23];
   }
   if( v[23] < 0 ) {
      v[11] = 0;
   } else {
      v[11] = v[23];
   }
   if( v[19] < 1e-09 ) {
      v[19] = v[16];
   } else {
      v[19] = v[19];
   }
   if( v[1] < 0 ) {
      v[19] = v[16];
   } else {
      v[19] = v[19];
   }
   if( v[1] > v[0] ) {
      v[19] = v[16];
   } else {
      v[19] = v[19];
   }
   if( v[11] > v[19] ) {
      v[11] = v[19];
   } else {
      v[11] = v[11];
   }
   if( fabs(v[11]) < 1e-09 ) {
      v[11] = 0;
   } else {
      v[11] = v[11] / v[19];
   }
   v[24] = v[24] - v[11] * v[12];
   if( 0 - v[28] > 0.024964 ) {
      v[16] = v[0];
   } else {
      v[16] = 0 - v[28];
   }
   if( 0 - v[28] < 0 ) {
      v[16] = 0;
   } else {
      v[16] = v[16];
   }
   if( 0 - v[28] + v[7] > 0.024964 ) {
      v[5] = v[0];
   } else {
      v[5] = 0 - v[28] + v[7];
   }
   if( 0 - v[28] + v[7] < 0 ) {
      v[5] = 0;
   } else {
      v[5] = v[5];
   }
   if( v[1] < 0 ) {
      v[1] = 0;
   } else {
      v[1] = v[1];
   }
   if( v[1] > v[0] ) {
      v[1] = v[0];
   } else {
      v[1] = v[1];
   }
   if( v[23] > v[19] ) {
      v[1] = v[5];
   } else {
      v[1] = v[1];
   }
   if( v[23] < 0 ) {
      v[1] = v[16];
   } else {
      v[1] = v[1];
   }
   if( 0 - v[28] > 0.024964 ) {
      v[16] = v[0];
   } else {
      v[16] = 0.024964;
   }
   if( 0 - v[28] < 0 ) {
      v[16] = v[0];
   } else {
      v[16] = v[16];
   }
   if( 0 - v[28] + v[7] > 0.024964 ) {
      v[5] = v[0];
   } else {
      v[5] = 0.024964;
   }
   if( 0 - v[28] + v[7] < 0 ) {
      v[5] = v[0];
   } else {
      v[5] = v[5];
   }
   if( v[23] > v[19] ) {
      v[5] = v[5];
   } else {
      v[5] = v[0];
   }
   if( v[23] < 0 ) {
      v[5] = v[16];
   } else {
      v[5] = v[5];
   }
   if( fabs(v[1]) < 1e-09 ) {
      v[5] = 0;
   } else {
      v[5] = v[1] / v[5];
   }
   v[18] = v[18] + 0.158 * v[5] - v[11] * v[4];
   v[18] = v[24] * v[24] + 0.000838102500000001 + v[18] * v[18];
   y[45] = sqrt(v[18]) - 0.04;
   v[10] = 0 - v[10];
   v[24] = v[8] * v[2] + v[25] * v[10];
   v[10] = v[20] * v[2] + v[8] * v[10];
   v[12] = v[9] + v[11] * v[12];
   v[18] = sqrt(v[18]);
   v[9] = v[12] / v[18];
   v[12] = v[12] - 0.015 * v[9];
   v[11] = v[29] + v[11] * v[4];
   v[5] = -0.158 + 0.158 * v[5];
   v[4] = (v[11] - v[5]) / v[18];
   v[11] = v[11] - 0.015 * v[4];
   y[57] = (0 - v[24]) * v[6] + (0 - v[10]) * v[14] + 0.19 * v[24] + (v[24] * v[13] + v[10] * v[15]) * v[12] + (v[24] * v[26] + v[10] * v[13]) * v[11];
   y[59] = (0 - v[3]) * v[6] + (0 - v[21]) * v[14] + 0.19 * v[3] + (v[3] * v[13] + v[21] * v[15]) * v[12] + (v[3] * v[26] + v[21] * v[13]) * v[11];
   y[54] = 0.025 * v[9];
   y[56] = v[5] + 0.025 * v[4];
   v[4] = v[6] + v[24] * y[57] + v[3] * y[59] - (0.19 + v[13] * y[54] + v[26] * y[56]);
   v[5] = 0 - (0 - y[56]);
   v[9] = 0 - y[54];
   v[11] = v[14] + v[10] * y[57] + v[21] * y[59] - (v[15] * y[54] + v[13] * y[56]);
   v[18] = -0.02895 / v[18];
   y[58] = 0.03745 + -0.04545 - 0.015 * v[18];
   y[55] = -0.0165 + 0.025 * v[18];
   v[18] = -0.14205 + y[58] - (-0.1046 + y[55]);
   v[18] = sqrt(v[4] * v[4] + v[18] * v[18] + v[11] * v[11]);
   y[48] = (v[4] * (0 - (v[13] * v[5] + v[26] * v[9])) + v[11] * (0 - (v[15] * v[5] + v[13] * v[9]))) / v[18];
   v[9] = 0 - y[59];
   v[5] = -0.16 * cos(x[7]) - v[9];
   v[12] = -0.16 * sin(x[7]) - y[57];
   y[52] = (v[4] * (v[24] * v[5] + v[3] * v[12]) + v[11] * (v[10] * v[5] + v[21] * v[12])) / v[18];
   v[9] = 0 - v[9];
   v[12] = 0 - y[57];
   y[53] = (v[4] * (v[24] * v[9] + v[3] * v[12]) + v[11] * (v[10] * v[9] + v[21] * v[12])) / v[18];
   v[12] = cos(x[3]);
   v[9] = sin(x[3]);
   v[18] = 0 - v[9];
   v[11] = v[13] * v[12] + v[26] * v[18];
   v[4] = 0.19 + -0.16 * v[26];
   v[18] = v[15] * v[12] + v[13] * v[18];
   v[5] = -0.16 * v[13];
   v[29] = (0 - v[11]) * v[4] + (0 - v[18]) * v[5] + -0.19 * v[11];
   v[2] = v[29] + -0.158 * (v[11] * v[25] + v[18] * v[8]);
   v[1] = 0 - v[2];
   v[29] = v[29] - v[2];
   v[26] = v[13] * v[9] + v[26] * v[12];
   v[9] = v[15] * v[9] + v[13] * v[12];
   v[12] = (0 - v[26]) * v[4] + (0 - v[9]) * v[5] + -0.19 * v[26];
   v[15] = v[12] + -0.158 * (v[26] * v[25] + v[9] * v[8]);
   v[12] = v[12] - v[15];
   v[13] = v[29] * v[29] + v[12] * v[12];
   v[16] = 0.16 * v[12];
   v[23] = 0.0256 * v[13] - v[16] * v[16];
   v[19] = -0.16 - v[15];
   v[0] = v[29] * v[1] + v[12] * v[19];
   v[28] = 0.16 * v[19];
   if( v[23] < 1e-09 ) {
      v[7] = 0;
   } else {
      v[7] = v[16] * v[0] - v[13] * v[28];
   }
   if( v[23] < 1e-09 ) {
      v[27] = 1;
   } else {
      v[27] = v[23];
   }
   if( v[23] < 1e-09 ) {
      v[22] = v[0];
   } else {
      v[22] = 0.0256 * v[0] - v[16] * v[28];
   }
   if( v[7] < 0 ) {
      v[22] = v[0];
   } else {
      v[22] = v[22];
   }
   if( v[7] > v[27] ) {
      v[22] = v[0] + v[16];
   } else {
      v[22] = v[22];
   }
   if( v[22] < 0 ) {
      v[0] = 0;
   } else {
      v[0] = v[22];
   }
   if( v[23] < 1e-09 ) {
      v[23] = v[13];
   } else {
      v[23] = v[23];
   }
   if( v[7] < 0 ) {
      v[23] = v[13];
   } else {
      v[23] = v[23];
   }
   if( v[7] > v[27] ) {
      v[23] = v[13];
   } else {
      v[23] = v[23];
   }
   if( v[0] > v[23] ) {
      v[0] = v[23];
   } else {
      v[0] = v[0];
   }
   if( fabs(v[0]) < 1e-09 ) {
      v[0] = 0;
   } else {
      v[0] = v[0] / v[23];
   }
   v[1] = v[1] - v[0] * v[29];
   if( 0 - v[28] > 0.0256 ) {
      v[13] = v[27];
   } else {
      v[13] = 0 - v[28];
   }
   if( 0 - v[28] < 0 ) {
      v[13] = 0;
   } else {
      v[13] = v[13];
   }
   if( 0 - v[28] + v[16] > 0.0256 ) {
      v[17] = v[27];
   } else {
      v[17] = 0 - v[28] + v[16];
   }
   if( 0 - v[28] + v[16] < 0 ) {
      v[17] = 0;
   } else {
      v[17] = v[17];
   }
   if( v[7] < 0 ) {
      v[7] = 0;
   } else {
      v[7] = v[7];
   }
   if( v[7] > v[27] ) {
      v[7] = v[27];
   } else {
      v[7] = v[7];
   }
   if( v[22] > v[23] ) {
      v[7] = v[17];
   } else {
      v[7] = v[7];
   }
   if( v[22] < 0 ) {
      v[7] = v[13];
   } else {
      v[7] = v[7];
   }
   if( 0 - v[28] > 0.0256 ) {
      v[13] = v[27];
   } else {
      v[13] = 0.0256;
   }
   if( 0 - v[28] < 0 ) {
      v[13] = v[27];
   } else {
      v[13] = v[13];
   }
   if( 0 - v[28] + v[16] > 0.0256 ) {
      v[17] = v[27];
   } else {
      v[17] = 0.0256;
   }
   if( 0 - v[28] + v[16] < 0 ) {
      v[17] = v[27];
   } else {
      v[17] = v[17];
   }
   if( v[22] > v[23] ) {
      v[17] = v[17];
   } else {
      v[17] = v[27];
   }
   if( v[22] < 0 ) {
      v[17] = v[13];
   } else {
      v[17] = v[17];
   }
   if( fabs(v[7]) < 1e-09 ) {
      v[17] = 0;
   } else {
      v[17] = v[7] / v[17];
   }
   v[19] = v[19] + 0.16 * v[17] - v[0] * v[12];
   v[19] = v[1] * v[1] + 0.000838102500000001 + v[19] * v[19];
   y[60] = sqrt(v[19]) - 0.04;
   v[29] = v[2] + v[0] * v[29];
   v[19] = sqrt(v[19]);
   v[2] = v[29] / v[19];
   v[29] = v[29] - 0.025 * v[2];
   v[0] = v[15] + v[0] * v[12];
   v[17] = -0.16 + 0.16 * v[17];
   v[12] = (v[0] - v[17]) / v[19];
   v[0] = v[0] - 0.025 * v[12];
   y[72] = -0.19 * (0 - v[8]) + v[8] * v[4] + v[20] * v[5] + (v[8] * v[11] + v[20] * v[18]) * v[29] + (v[8] * v[26] + v[20] * v[9]) * v[0];
   y[74] = -0.19 * (0 - v[25]) + v[25] * v[4] + v[8] * v[5] + (v[25] * v[11] + v[8] * v[18]) * v[29] + (v[25] * v[26] + v[8] * v[9]) * v[0];
   y[69] = 0.015 * v[2];
   y[71] = v[17] + 0.015 * v[12];
   v[12] = -0.19 + v[8] * y[72] + v[25] * y[74] - (v[4] + v[11] * y[69] + v[26] * y[71]);
   v[17] = 0 - y[71];
   v[2] = -0.16 * cos(x[3]) - v[17];
   v[0] = -0.16 * sin(x[3]) - y[69];
   v[29] = v[20] * y[72] + v[8] * y[74] - (v[5] + v[18] * y[69] + v[9] * y[71]);
   v[19] = 0.02895 / v[19];
   y[73] = -0.03745 + 0.02095 - 0.025 * v[19];
   y[70] = -0.008 + 0.015 * v[19];
   v[19] = -0.1046 + y[73] - (-0.14205 + y[70]);
   v[19] = sqrt(v[12] * v[12] + v[19] * v[19] + v[29] * v[29]);
   y[63] = (v[12] * (0 - (v[11] * v[2] + v[26] * v[0])) + v[29] * (0 - (v[18] * v[2] + v[9] * v[0]))) / v[19];
   v[17] = 0 - v[17];
   v[0] = 0 - y[69];
   y[64] = (v[12] * (0 - (v[11] * v[17] + v[26] * v[0])) + v[29] * (0 - (v[18] * v[17] + v[9] * v[0]))) / v[19];
   v[0] = 0 - (0 - y[74]);
   v[17] = 0 - y[72];
   y[67] = (v[12] * (v[8] * v[0] + v[25] * v[17]) + v[29] * (v[20] * v[0] + v[8] * v[17])) / v[19];
   v[17] = (0 - v[11]) * v[4] + (0 - v[18]) * v[5] + v[11] * v[6] + v[18] * v[14];
   v[0] = v[17] + -0.16 * (v[11] * v[3] + v[18] * v[21]);
   v[19] = 0 - v[0];
   v[17] = v[17] - v[0];
   v[29] = (0 - v[26]) * v[4] + (0 - v[9]) * v[5] + v[26] * v[6] + v[9] * v[14];
   v[12] = v[29] + -0.16 * (v[26] * v[3] + v[9] * v[21]);
   v[29] = v[29] - v[12];
   v[20] = v[17] * v[17] + v[29] * v[29];
   v[8] = 0.16 * v[29];
   v[25] = 0.0256 * v[20] - v[8] * v[8];
   v[2] = -0.16 - v[12];
   v[15] = v[17] * v[19] + v[29] * v[2];
   v[1] = 0.16 * v[2];
   if( v[25] < 1e-09 ) {
      v[7] = 0;
   } else {
      v[7] = v[8] * v[15] - v[20] * v[1];
   }
   if( v[25] < 1e-09 ) {
      v[13] = 1;
   } else {
      v[13] = v[25];
   }
   if( v[25] < 1e-09 ) {
      v[22] = v[15];
   } else {
      v[22] = 0.0256 * v[15] - v[8] * v[1];
   }
   if( v[7] < 0 ) {
      v[22] = v[15];
   } else {
      v[22] = v[22];
   }
   if( v[7] > v[13] ) {
      v[22] = v[15] + v[8];
   } else {
      v[22] = v[22];
   }
   if( v[22] < 0 ) {
      v[15] = 0;
   } else {
      v[15] = v[22];
   }
   if( v[25] < 1e-09 ) {
      v[25] = v[20];
   } else {
      v[25] = v[25];
   }
   if( v[7] < 0 ) {
      v[25] = v[20];
   } else {
      v[25] = v[25];
   }
   if( v[7] > v[13] ) {
      v[25] = v[20];
   } else {
      v[25] = v[25];
   }
   if( v[15] > v[25] ) {
      v[15] = v[25];
   } else {
      v[15] = v[15];
   }
   if( fabs(v[15]) < 1e-09 ) {
      v[15] = 0;
   } else {
      v[15] = v[15] / v[25];
   }
   v[19] = v[19] - v[15] * v[17];
   if( 0 - v[1] > 0.0256 ) {
      v[20] = v[13];
   } else {
      v[20] = 0 - v[1];
   }
   if( 0 - v[1] < 0 ) {
      v[20] = 0;
   } else {
      v[20] = v[20];
   }
   if( 0 - v[1] + v[8] > 0.0256 ) {
      v[23] = v[13];
   } else {
      v[23] = 0 - v[1] + v[8];
   }
   if( 0 - v[1] + v[8] < 0 ) {
      v[23] = 0;
   } else {
      v[23] = v[23];
   }
   if( v[7] < 0 ) {
      v[7] = 0;
   } else {
      v[7] = v[7];
   }
   if( v[7] > v[13] ) {
      v[7] = v[13];
   } else {
      v[7] = v[7];
   }
   if( v[22] > v[25] ) {
      v[7] = v[23];
   } else {
      v[7] = v[7];
   }
   if( v[22] < 0 ) {
      v[7] = v[20];
   } else {
      v[7] = v[7];
   }
   if( 0 - v[1] > 0.0256 ) {
      v[20] = v[13];
   } else {
      v[20] = 0.0256;
   }
   if( 0 - v[1] < 0 ) {
      v[20] = v[13];
   } else {
      v[20] = v[20];
   }
   if( 0 - v[1] + v[8] > 0.0256 ) {
      v[23] = v[13];
   } else {
      v[23] = 0.0256;
   }
   if( 0 - v[1] + v[8] < 0 ) {
      v[23] = v[13];
   } else {
      v[23] = v[23];
   }
   if( v[22] > v[25] ) {
      v[23] = v[23];
   } else {
      v[23] = v[13];
   }
   if( v[22] < 0 ) {
      v[23] = v[20];
   } else {
      v[23] = v[23];
   }
   if( fabs(v[7]) < 1e-09 ) {
      v[23] = 0;
   } else {
      v[23] = v[7] / v[23];
   }
   v[2] = v[2] + 0.16 * v[23] - v[15] * v[29];
   v[2] = v[19] * v[19] + v[2] * v[2];
   y[75] = sqrt(v[2]) - 0.03;
   v[17] = v[0] + v[15] * v[17];
   v[2] = sqrt(v[2]);
   v[0] = v[17] / v[2];
   v[17] = v[17] - 0.015 * v[0];
   v[15] = v[12] + v[15] * v[29];
   v[23] = -0.16 + 0.16 * v[23];
   v[2] = (v[15] - v[23]) / v[2];
   v[15] = v[15] - 0.015 * v[2];
   y[87] = (0 - v[24]) * v[6] + (0 - v[10]) * v[14] + v[24] * v[4] + v[10] * v[5] + (v[24] * v[11] + v[10] * v[18]) * v[17] + (v[24] * v[26] + v[10] * v[9]) * v[15];
   y[89] = (0 - v[3]) * v[6] + (0 - v[21]) * v[14] + v[3] * v[4] + v[21] * v[5] + (v[3] * v[11] + v[21] * v[18]) * v[17] + (v[3] * v[26] + v[21] * v[9]) * v[15];
   y[84] = 0.015 * v[0];
   y[86] = v[23] + 0.015 * v[2];
   v[4] = v[6] + v[24] * y[87] + v[3] * y[89] - (v[4] + v[11] * y[84] + v[26] * y[86]);
   v[6] = 0 - y[86];
   v[2] = -0.16 * cos(x[3]) - v[6];
   v[23] = -0.16 * sin(x[3]) - y[84];
   v[5] = v[14] + v[10] * y[87] + v[21] * y[89] - (v[5] + v[18] * y[84] + v[9] * y[86]);
   v[14] = sqrt(v[4] * v[4] + v[5] * v[5]);
   y[78] = (v[4] * (0 - (v[11] * v[2] + v[26] * v[23])) + v[5] * (0 - (v[18] * v[2] + v[9] * v[23]))) / v[14];
   v[6] = 0 - v[6];
   v[23] = 0 - y[84];
   y[79] = (v[4] * (0 - (v[11] * v[6] + v[26] * v[23])) + v[5] * (0 - (v[18] * v[6] + v[9] * v[23]))) / v[14];
   v[23] = 0 - y[89];
   v[6] = -0.16 * cos(x[7]) - v[23];
   v[9] = -0.16 * sin(x[7]) - y[87];
   y[82] = (v[4] * (v[24] * v[6] + v[3] * v[9]) + v[5] * (v[10] * v[6] + v[21] * v[9])) / v[14];
   v[23] = 0 - v[23];
   v[9] = 0 - y[87];
   y[83] = (v[4] * (v[24] * v[23] + v[3] * v[9]) + v[5] * (v[10] * v[23] + v[21] * v[9])) / v[14];
   // dependent variables without operations
   y[2] = 0;
   y[3] = 0;
   y[4] = 0;
   y[7] = 0;
   y[8] = 0;
   y[18] = 0;
   y[19] = 0;
   y[21] = 0;
   y[22] = 0;
   y[23] = 0;
   y[33] = 0;
   y[34] = 0;
   y[37] = 0;
   y[38] = 0;
   y[40] = 0.008;
   y[43] = 0.008;
   y[46] = -0;
   y[47] = -0;
   y[49] = -0;
   y[50] = -0;
   y[51] = -0;
   y[61] = 0;
   y[62] = 0;
   y[65] = 0;
   y[66] = 0;
   y[68] = 0;
   y[76] = 0;
   y[77] = 0;
   y[80] = 0;
   y[81] = 0;
   y[85] = -0.008;
   y[88] = -0.008;
}

