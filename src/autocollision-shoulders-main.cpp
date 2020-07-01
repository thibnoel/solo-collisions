#include "autocollision-shoulders-code-generation.hpp"

int main()
{

    int size = 500;
    // Files path
    //std::string csvRealPath = "/home/tnoel/stage/solo-collisions/src/python/fft_estim_real.csv";
    //std::string csvImagPath = "/home/tnoel/stage/solo-collisions/src/python/fft_estim_imag.csv";
    std::string csvRealPath = "/home/thibault/stage_LAAS_042020_102020/git_workspace/solo-collisions/src/python/fft_estim_real.csv";
    std::string csvImagPath = "/home/thibault/stage_LAAS_042020_102020/git_workspace/solo-collisions/src/python/fft_estim_imag.csv";
    std::fstream fileReal(csvRealPath, std::ios::in);
    std::fstream fileImag(csvImagPath, std::ios::in);

    if(!fileReal.is_open() || !fileImag.is_open())
    {
        std::cout << "File not found!\n";
        return 1;
    }
    // Initialize Fourier coeff array
    //Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> out;
    Eigen::Matrix<std::complex<ADScalar>, Eigen::Dynamic, Eigen::Dynamic> ad_FT_coeffs;
    // Fill the values from .csv file 
    ad_FT_coeffs = CSVtoFTcoeff<ADScalar>(fileReal, fileImag, size);
    ad_FT_coeffs.resize(size,size);

    // Generate the code for the specified shoulder, based on an FT coeffs matrix
    //ADFun genFun = tapeAD4ShouldersDistanceCheck(ad_FT_coeffs);
    //generateCompileCLib("solo_shoulder_autocollision", genFun);
    // Generate the jacobian for the specified shoulder
    ADFun genJac = tapeADShoulderJacobian(ad_FT_coeffs);
    generateCompileCLib("solo_shoulder_autocollision_jac", genJac);

    std::cout << "Size : " << size << "x" << size << std::endl;
    ADScalar x_eval,y_eval;
    x_eval = 250;
    y_eval = 250; 
    std::cout << "Fun. eval. : " << evaluateFromFT<ADScalar>(ad_FT_coeffs, x_eval, y_eval) << std::endl;
    std::cout << "Jac. eval. : " << evaluateJacobianFromFFT<ADScalar>(ad_FT_coeffs, x_eval, y_eval) << std::endl;

}