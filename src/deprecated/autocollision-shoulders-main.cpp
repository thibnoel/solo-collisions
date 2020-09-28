#include "autocollision-shoulders-code-generation.hpp"
#include <chrono> 

using namespace std::chrono; 

int main()
{

    int size = 500;
    // Files path
    std::string csvRealPath = "/home/tnoel/stage/solo-collisions/src/python/fft_estim_real.csv";
    std::string csvImagPath = "/home/tnoel/stage/solo-collisions/src/python/fft_estim_imag.csv";
    //std::string csvRealPath = "/home/thibault/stage_LAAS_042020_102020/git_workspace/solo-collisions/src/python/fft_estim_real.csv";
    //std::string csvImagPath = "/home/thibault/stage_LAAS_042020_102020/git_workspace/solo-collisions/src/python/fft_estim_imag.csv";
    std::fstream fileReal(csvRealPath, std::ios::in);
    std::fstream fileImag(csvImagPath, std::ios::in);

    if(!fileReal.is_open() || !fileImag.is_open())
    {
        std::cout << "File not found!\n";
        return 1;
    }
    // Initialize Fourier coeff array
    Eigen::Matrix<std::complex<double>, Eigen::Dynamic, Eigen::Dynamic> FT_coeffs;
    Eigen::Matrix<std::complex<ADScalar>, Eigen::Dynamic, Eigen::Dynamic> ad_FT_coeffs; 
    std::pair<Eigen::Matrix<std::complex<ADScalar>, Eigen::Dynamic, Eigen::Dynamic>,
                Eigen::Matrix<std::complex<ADScalar>, Eigen::Dynamic, Eigen::Dynamic>> ad_Jac_FT_coeffs;
    // Fill the values from .csv file 
    FT_coeffs = CSVtoFTcoeff<double>(fileReal, fileImag, size);
    FT_coeffs.resize(size,size);

    fileReal.close();
    fileReal.clear();
    fileReal.open(csvRealPath);
    fileImag.close();
    fileImag.clear();
    fileImag.open(csvImagPath);

    ad_FT_coeffs = CSVtoFTcoeff<ADScalar>(fileReal, fileImag, size);
    ad_FT_coeffs.resize(size,size);
    ad_Jac_FT_coeffs = computeJacFTcoeff<ADScalar>(ad_FT_coeffs.transpose());

    /***************************************************************************
    *                      Code generation
    ***************************************************************************/
    // Generate the code for the specified shoulder, based on an FT coeffs matrix
    ADFun genFun = tapeAD4ShouldersDistanceCheck(ad_FT_coeffs.transpose());
    generateCompileCLib("solo_shoulder_autocollision", genFun);
    // Generate the jacobian for the specified shoulder
    //ADFun genJac = tapeADShoulderJacobian(ad_FT_coeffs);
    //generateCompileCLib("solo_shoulder_autocollision_jac", genJac);

    /***************************************************************************
    *                      Generated code evaluation
    ***************************************************************************/
    const std::string LIBRARY_NAME = "./libCGsolo_shoulder_autocollision";
    const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("solo_shoulder_autocollision");

    // Generated code evaluation
        // Input : Get a random config.
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_test;
    X_test = 3.1415*Eigen::Matrix<double,8,1>::Random(8,1);
    //X_test = Eigen::Matrix<double,8,1>::Zero(8,1);
        // Output : distance
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_test;
    Y_test.resize(4);

    // Function evaluation with start and stop timestamps
    auto start_cg = high_resolution_clock::now();
    model->ForwardZero(X_test, Y_test);
    auto stop_cg = high_resolution_clock::now(); 
    auto duration_cg = duration_cast<nanoseconds>(stop_cg - start_cg);

    /*ADScalar x_eval,y_eval;
    x_eval = PI;
    y_eval = 1.5*PI; */
    std::cout << "Fun. eval. : \n";
    std::cout << evaluateFromFT<double>(FT_coeffs.transpose(), X_test[0], X_test[1]).real() << std::endl;
    //std::cout << "Jac. x eval. : " << evaluateFromFT<ADScalar>(ad_Jac_FT_coeffs.first, x_eval, y_eval).real() << std::endl;
    //std::cout << "Jac. y eval. : " << evaluateFromFT<ADScalar>(ad_Jac_FT_coeffs.second, x_eval, y_eval).real() << std::endl;
    std::cout << "\nCodegen eval : \n" << Y_test  << std::endl;
    std::cout << "Duration : " << ((int)duration_cg.count())*0.001 << " microseconds" << std::endl; 
}