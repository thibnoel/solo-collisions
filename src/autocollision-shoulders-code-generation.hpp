#include <stdio.h>
#include <stdlib.h>
#include <math.h>  
#include <iostream>
#include <fstream>
#include <istream> 
#include <vector>
//#include <Eigen/Core>
//#include <Eigen/Dense>
#include "pinocchio/algorithm/geometry.hpp"
#include <complex>

#include "codegen_helper.hpp"

#define PI 3.14159265

// Read a CSV file and dump it to an Eigen Matrix
Eigen::Matrix<double, Eigen::Dynamic, 1> readCSV(std::istream &input, int matSize)
{
    int a = 0;
    int b = 0;

    std::string csvLine;
    Eigen::Matrix<double, Eigen::Dynamic, 1> out;
    out.resize(matSize*matSize);
    // read every line from the stream
    while( std::getline(input, csvLine) )
    {

        std::istringstream csvStream(csvLine);
        std::vector<std::string> csvColumn;
        std::string csvElement;
        // read every element from the line that is seperated by commas
        // and put it into the vector or strings
        while( getline(csvStream, csvElement, ',') )
        {
            csvColumn.push_back(csvElement);
            out[b] = std::stod(csvElement);
            b++;
        }       
        a++;
    }
    std::cout << "a : " << a << " b : " << b << std::endl;  
    return out;
}

// Build a complex Eigen::Matrix representing the 2D FFT coeffs from 2 files representing the real and imaginary 2D FFT matrices
template <typename Scalar>
Eigen::Matrix<std::complex<Scalar>, Eigen::Dynamic, 1> CSVtoFTcoeff(std::istream &inputReal, std::istream &inputImag, int matSize)
{
    std::complex<Scalar> consti;
    consti = std::complex<double>{0,1};

    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> outReal;
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> outImag;
    Eigen::Matrix<std::complex<Scalar>, Eigen::Dynamic, 1> out;

    outReal = readCSV(inputReal, matSize);
    outImag = readCSV(inputImag, matSize);

    out = outReal + consti*outImag;

    return out;
}

// Evaluate a 2D function from its FT coefficients 
template <typename Scalar>
std::complex<Scalar> evaluateFromFT(Eigen::Matrix<std::complex<Scalar>, Eigen::Dynamic, Eigen::Dynamic> FTcoeffs, Scalar x, Scalar y)
{
    Scalar zero;
    zero = 0;
    
    std::complex<Scalar> eval;
    eval = std::complex<double>{0,0};
    
    std::complex<Scalar> consti;
    consti = std::complex<double>{0,1};

    int m = (int)FTcoeffs.rows();
    int n = (int)FTcoeffs.cols();

    Scalar coeffThreshold;
    coeffThreshold = 1e-6;

    for(int i=0; i<n; i++)
    {
        for(int j=0; j<m; j++)
        {   
            Scalar coeffReal;
            Scalar coeffImag;

            coeffReal = CppAD::CondExpLt(std::abs(FTcoeffs(i,j)), coeffThreshold, zero, FTcoeffs(i,j).real());
            coeffImag = CppAD::CondExpLt(std::abs(FTcoeffs(i,j)), coeffThreshold, zero, FTcoeffs(i,j).imag());
            std::complex<Scalar> coeff(coeffReal, coeffImag);

            eval += coeff*(cos(2*PI*(y*(i-n/2)/n + x*(j-m/2)/m)) + consti*sin(2*PI*(y*(i-n/2)/n + x*(j-m/2)/m)));
        }
    }
    std::complex<Scalar> scale;
    scale = std::complex<double>{(double)n*m,0};
    eval/=scale;
    return eval;
}
// NOT TESTED
template <typename Scalar> 
Eigen::Matrix<std::complex<Scalar>, Eigen::Dynamic, 1> evaluateJacobianFromFFT(Eigen::Matrix<std::complex<Scalar>, Eigen::Dynamic, Eigen::Dynamic> FTcoeffs, Scalar x, Scalar y)
{
    Scalar zero;
    zero = 0;

    int m = (int)FTcoeffs.rows();
    int n = (int)FTcoeffs.cols();

    std::complex<Scalar> const_i;
    const_i = std::complex<double>{0,1};
    std::complex<Scalar> Jx;
    std::complex<Scalar> Jy;

    Scalar coeffThreshold;
    coeffThreshold = 1e-6;

    for(int i=0; i<n; i++)
    {
        for(int j=0; j<m; j++)
        {   
            Scalar coeffReal;
            Scalar coeffImag;

            coeffReal = CppAD::CondExpLt(std::abs(FTcoeffs(i,j)), coeffThreshold, zero, FTcoeffs(i,j).real());
            coeffImag = CppAD::CondExpLt(std::abs(FTcoeffs(i,j)), coeffThreshold, zero, FTcoeffs(i,j).imag());
            std::complex<Scalar> coeff(coeffReal, coeffImag);

            Jx += 2*PI*(x*j/m)*const_i*coeff*(cos(2*PI*(y*(i-n/2)/n + x*(j-m/2)/m)) + const_i*sin(2*PI*(y*(i-n/2)/n + x*(j-m/2)/m)));
            Jy += 2*PI*(y*i/n)*const_i*coeff*(cos(2*PI*(y*(i-n/2)/n + x*(j-m/2)/m)) + const_i*sin(2*PI*(y*(i-n/2)/n + x*(j-m/2)/m)));
        }
    }
    std::complex<Scalar> scale;
    scale = std::complex<double>{(double)n*m,0};

    Eigen::Matrix<std::complex<Scalar>, Eigen::Dynamic, 1> J;
    J.resize(2);
    J << Jx/scale, Jy/scale;
    return J;  
}

// Generates the model for the function f(q, pair) = dist. between frames of given pair (defined by the FT coeffs!)
ADFun tapeADShoulderDistanceCheck(Eigen::Matrix<std::complex<ADScalar>, Eigen::Dynamic, Eigen::Dynamic> FTcoeffs)
{   
    // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(2);
    ad_Y.resize(1);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;

    // Tape the function
    std::complex<ADScalar> d = evaluateFromFT<ADScalar>(FTcoeffs, ad_X[0], ad_X[1]);
    ad_Y[0] = CppAD::sqrt(d.real()*d.real() - d.imag()*d.imag());
    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}

// TO CHECK : return the magnitudes of d∕dx, d∕dy : is it correct ?
ADFun tapeADShoulderJacobian(Eigen::Matrix<std::complex<ADScalar>, Eigen::Dynamic, Eigen::Dynamic> FTcoeffs)
{
    // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(2);
    ad_Y.resize(2);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;

    // Tape the function
    Eigen::Matrix<std::complex<ADScalar>, 2, 1> jac;
    jac = evaluateJacobianFromFFT<ADScalar>(FTcoeffs, ad_X[0], ad_X[1]);
    ad_Y[0] = CppAD::sqrt(jac[0].real()*jac[0].real() - jac[0].imag()*jac[0].imag());
    ad_Y[1] = CppAD::sqrt(jac[1].real()*jac[1].real() - jac[1].imag()*jac[1].imag());
    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}

// Same for all shoulders
// The collision map (FTcoeffs) is the same for all shoulders (symetry), we use the Front Left shoulder as reference 
ADFun tapeAD4ShouldersDistanceCheck(Eigen::Matrix<std::complex<ADScalar>, Eigen::Dynamic, Eigen::Dynamic> FL_FTcoeffs)
{
    // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(8);
    ad_Y.resize(4);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;

    std::complex<ADScalar> dFL, dFR, dHL, dHR;
    // Tape the function
    dFL = evaluateFromFT<ADScalar>(FL_FTcoeffs, ad_X[0], ad_X[1]);
    dFR = evaluateFromFT<ADScalar>(FL_FTcoeffs, -ad_X[2], ad_X[3]); // coeff : opposite on x dir
    dHL = evaluateFromFT<ADScalar>(FL_FTcoeffs, ad_X[4], -ad_X[5]); // coeff : opposite on y dir
    dHR = evaluateFromFT<ADScalar>(FL_FTcoeffs, -ad_X[6], -ad_X[7]); // coeff : opposite on both dirs
    ad_Y[0] = CppAD::sqrt(dFL.real()*dFL.real() - dFL.imag()*dFL.imag());
    ad_Y[1] = CppAD::sqrt(dFR.real()*dFR.real() - dFR.imag()*dFR.imag());
    ad_Y[2] = CppAD::sqrt(dHL.real()*dHL.real() - dHL.imag()*dHL.imag());
    ad_Y[3] = CppAD::sqrt(dHR.real()*dHR.real() - dHR.imag()*dHR.imag());
    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}


