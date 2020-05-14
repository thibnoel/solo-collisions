#include <iosfwd>
#include <iostream>
#include <array>
#include <stdlib.h>

double generatedFun(double x[] , double y[])
{
    double v[18] ;

    // Generated dist(x) :
    v[0] = x[4] - x[1];
    v[1] = x[3] - x[0];
    v[2] = x[5] - x[2];
    v[3] = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    v[4] = x[10] - x[7];
    v[5] = x[9] - x[6];
    v[6] = x[11] - x[8];
    v[7] = v[4] * v[4] + v[5] * v[5] + v[6] * v[6];
    v[8] = v[0] * v[4] + v[1] * v[5] + v[2] * v[6];
    v[9] = v[3] * v[7] - v[8] * v[8];
    v[10] = x[1] - x[7];
    v[11] = x[0] - x[6];
    v[12] = x[2] - x[8];
    v[13] = v[4] * v[10] + v[5] * v[11] + v[6] * v[12];
    v[14] = v[0] * v[10] + v[1] * v[11] + v[2] * v[12];
    if( v[9] < 1e-09 ) {
        v[15] = 0;
    } else {
        v[15] = v[8] * v[13] - v[7] * v[14];
    }
    if( v[9] < 1e-09 ) {
        v[16] = 1;
    } else {
        v[16] = v[9];
    }
    if( v[9] < 1e-09 ) {
        v[17] = v[13];
    } else {
        v[17] = v[3] * v[13] - v[8] * v[14];
    }
    if( v[15] < 0 ) {
        v[17] = v[13];
    } else {
        v[17] = v[17];
    }
    if( v[15] > v[16] ) {
        v[17] = v[13] + v[8];
    } else {
        v[17] = v[17];
    }
    v[13] = 0 - v[14];
    if( 0 - v[14] > v[3] ) {
        v[18] = v[16];
    } else {
        v[18] = v[13];
    }
    if( v[13] < 0 ) {
        v[18] = 0;
    } else {
        v[18] = v[18];
    }
    if( v[9] < 1e-09 ) {
        v[9] = v[7];
    } else {
        v[9] = v[9];
    }
    if( v[15] > v[16] ) {
        v[9] = v[7];
    } else {
        v[9] = v[9];
    }
    if( v[8] - v[14] > v[3] ) {
        v[7] = v[16];
    } else {
        v[7] = v[8] - v[14];
    }
    if( v[8] - v[14] < 0 ) {
        v[7] = 0;
    } else {
        v[7] = v[7];
    }
    if( v[15] < 0 ) {
        v[15] = 0;
    } else {
        v[15] = v[15];
    }
    if( v[15] > v[16] ) {
        v[15] = v[16];
    } else {
        v[15] = v[15];
    }
    if( v[17] > v[9] ) {
        v[15] = v[7];
    } else {
        v[15] = v[15];
    }
    if( v[17] < 0 ) {
        v[15] = v[18];
    } else {
        v[15] = v[15];
    }
    if( 0 - v[14] > v[3] ) {
        v[18] = v[16];
    } else {
        v[18] = v[3];
    }
    if( 0 - v[14] < 0 ) {
        v[18] = v[16];
    } else {
        v[18] = v[18];
    }
    if( v[8] - v[14] > v[3] ) {
        v[3] = v[16];
    } else {
        v[3] = v[3];
    }
    if( v[8] - v[14] < 0 ) {
        v[3] = v[16];
    } else {
        v[3] = v[3];
    }
    if( v[17] > v[9] ) {
        v[3] = v[3];
    } else {
        v[3] = v[16];
    }
    if( v[17] < 0 ) {
        v[3] = v[18];
    } else {
        v[3] = v[3];
    }
    if( abs(v[15]) < 1e-09 ) {
        v[3] = 0;
    } else {
        v[3] = v[15] / v[3];
    }
    if( v[17] < 0 ) {
        v[17] = 0;
    } else {
        v[17] = v[17];
    }
    if( v[17] > v[9] ) {
        v[17] = v[9];
    } else {
        v[17] = v[17];
    }
    if( abs(v[17]) < 1e-09 ) {
        v[17] = 0;
    } else {
        v[17] = v[17] / v[9];
    }
    v[10] = v[3] * v[0] + v[10] - v[17] * v[4];
    v[11] = v[3] * v[1] + v[11] - v[17] * v[5];
    v[17] = v[3] * v[2] + v[12] - v[17] * v[6];
    y[0] = v[10] * v[10] + v[11] * v[11] + v[17] * v[17];



    std::cout << v[0] << std::endl;
    std::cout << v[1] << std::endl;
    std::cout << v[2] << std::endl;
    std::cout << v[3] << std::endl;
    std::cout << v[4] << std::endl;
    std::cout << v[10] << std::endl;
    std::cout << v[11] << std::endl;
    std::cout << v[17] << std::endl;


    return y[0];
}

int main(){
    double x[12] = {0,0,0,1,1,1,0,0,2,0,0,3};
    double y[1] = {0.};
    double a = generatedFun(x,y);
    std::cout << "Result : " << a << std::endl;
}



