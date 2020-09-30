#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "inference_neural_net.hpp"
#include <chrono> 
#include <sstream>

using namespace std::chrono; 

// MAIN
int main()
{
    Eigen::Matrix<double, 8, 4> w0;
    Eigen::Matrix<double, 8, 1> b0;
    Eigen::Matrix<double, 1, 8> w1;
    Eigen::Matrix<double, 1, 1> b1;

    w0 << 0.16665615, -0.57635957, -0.9273581,  -0.0443992, 
          0.27460676, 0.39665112,-1.4758316  , 0.01899619,
          -0.27149975,-0.02114667,-0.03340376 ,-1.0156049 ,
          -0.2222562 , 0.92098653,-0.8633715  ,-0.0436529 ,
          1.3515042 , 1.7989769 ,-3.7476184  ,-0.0526121 ,
          -0.4920758 ,-0.03281262,-0.01200571 , 0.76729655,
          0.65630466,-1.1160649 ,-1.2035916  ,-0.07412633,
          0.15867329, 0.7818634 , 1.0530293  , 1.6800112 ;
    
    b0 << 0.6493005, 0.8563075, -0.32729325, -0.5499291, -0.3899357, 0.07156015, 0.25592574, -0.15024726;

    w1 << -2.137344, 0.94646454, -1.1316826, -0.7469052, 0.20912892, -0.98117703, 0.8960185, -0.23441358;

    b1 << 0.20489123;

    Layer<ADScalar> layer0 = {w0, b0};
    Layer<ADScalar> layer1 = {w1, b1};

    Eigen::Matrix<ADScalar, 4, 1> test_x;
    test_x << (ADScalar)0,(ADScalar)1,(ADScalar)0,(ADScalar)2;
    std::cout << test_x << std::endl;

    InferenceNeuralNetwork<ADScalar> nn;
    nn.layers.push_back(layer0);
    nn.layers.push_back(layer1);

    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> out  = nn.feedForward(test_x);
    std::cout << "Result :\n" << out << std::endl;

    Eigen::Matrix<ADScalar, Eigen::Dynamic,  Eigen::Dynamic> grad = nn.outInGradient(test_x); 
    std::cout << "Jac :\n" << grad << "\nsize : " << grad.rows() << grad.cols() << std::endl;

    Eigen::Matrix<ADScalar, Eigen::Dynamic,  Eigen::Dynamic> grad_in = nn.inQGradient(test_x); 
    std::cout << "Jac_in :\n" << grad_in << "\nsize : " << grad_in.rows() << grad_in.cols() <<  std::endl;

    std::cout << "Jac_prod :\n" << grad*grad_in << std::endl;

    ADFun genFun = tapeADNeuralNetInference(nn, 2);
    generateCompileCLib("solo_autocollision_nn_shoulder", genFun);


    /***************************************************************************
    *                      Generated code evaluation
    ***************************************************************************/
    // Load the model from the compiled library
    const std::string LIBRARY_NAME = "./libCGsolo_autocollision_nn_shoulder";
    const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("solo_autocollision_nn_shoulder");

    // Input : Get a random config.
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_test;
    X_test = 3.1415*Eigen::Matrix<double,4,1>::Random(4,1);
    //X_test = Eigen::Matrix<double,12,1>::Zero(12,1);
    
    // Output : distance
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_test;
    //Y_test.resize(20*(1+3*12));
    Y_test.resize(1);
    
    // Function evaluation with start and stop timestamps
    auto start_cg = high_resolution_clock::now();
    for (int k = 0; k<1e6; k++)
    {
        X_test = 3.1415*Eigen::Matrix<double,4,1>::Random(4,1);
        model->ForwardZero(X_test, Y_test);
    }
    auto stop_cg = high_resolution_clock::now(); 
    auto duration_cg = duration_cast<microseconds>(stop_cg - start_cg); 

    std::cout << "\nTime taken by function (1e6 executions): " << ((int)duration_cg.count()) << " microseconds" << std::endl; 

}