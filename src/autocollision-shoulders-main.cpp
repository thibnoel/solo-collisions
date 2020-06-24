#include "autocollision-shoulders-code-generation.hpp"

int main()
{

    int size = 500;
    std::string csvRealPath = "/home/tnoel/stage/solo-collisions/src/python/fft_estim_real.csv";
    std::string csvImagPath = "/home/tnoel/stage/solo-collisions/src/python/fft_estim_imag.csv";
    std::fstream fileReal(csvRealPath, std::ios::in);
    std::fstream fileImag(csvImagPath, std::ios::in);

    if(!fileReal.is_open() || !fileImag.is_open())
    {
        std::cout << "File not found!\n";
        return 1;
    }

    //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> outReal;
    //Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> outImag;
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> out;
    Eigen::Matrix<std::complex<ADScalar>, Eigen::Dynamic, Eigen::Dynamic> ad_out;

    /*outReal = readCSV(fileReal);
    outImag = readCSV(fileImag);
    outReal.resize(size,size);
    outImag.resize(size,size);
    

    std::complex<double> consti(0,1);*/

    //out = CSVtoFTcoeff<double>(fileReal, fileImag, size);
    //out.resize(size,size);

    ad_out = CSVtoFTcoeff<ADScalar>(fileReal, fileImag, size);
    ad_out.resize(size,size);

    // Generate the code for the specified shoulder, based on an FT coeffs matrix
    ADFun genFun = tapeAD4ShouldersDistanceCheck(ad_out);
    generateCompileCLib("solo_shoulder_autocollision", genFun);

    std::cout << "Size : " << size << "x" << size << std::endl;
    ADScalar x_eval,y_eval;
    x_eval = 250;
    y_eval = 250; 
    std::cout << evaluateFromFT<ADScalar>(ad_out, x_eval, y_eval) << std::endl;
}