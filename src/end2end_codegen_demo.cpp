#include <stdlib.h>
#include <iostream>
#include "end2end_legs_collision_check.hpp"
#include <chrono> 
using namespace std::chrono; 
using namespace pinocchio;

int main(int argc, char* argv[])
{

    // Check the number of parameters
    if (argc < 3) {
        // Tell the user how to run the program
        std::cerr << "Usage: " << argv[0] << " Frame1_NAME Frame2_NAME" << std::endl;
        /* "Usage messages" are a conventional way of telling the user
         * how to run a program if they enter the command incorrectly.
         */
        return 1;
    }

    std::string frame1Name = argv[1];
    std::string frame2Name = argv[2];

    // Load Solo 12 model
        // Setup URDF path
    const std::string urdf_filename = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf";

        // Load and build the URDF model
    Model rmodel;
    pinocchio::urdf::buildModel(urdf_filename,rmodel);
        // Generate model data
    Data rdata(rmodel); 

    // Get frames indices from the model
    int fl_upper_leg = (int)rmodel.getFrameId(frame1Name);
    int hr_lower_leg = (int)rmodel.getFrameId(frame2Name);

    // Define capsule parameters
        // Geometry
    ADScalar capsLength;
    ADScalar capsRadius;
    capsLength = 0.2;
    capsRadius = 0.02;
        // Capsule frame : offset between leg frame and 1 end of the capsule
        // Translation
    Eigen::Matrix<ADScalar, 3, 1> capsPosOffset;
    capsPosOffset[0] = 0;
    capsPosOffset[1] = 0;
    capsPosOffset[2] = 0;
        // Rotation
    Eigen::Quaternion<ADScalar> capsRotOffset;
    capsRotOffset.setIdentity();
        // SE3 object
    pinocchio::SE3Tpl<ADScalar> capsFrame(capsRotOffset, capsPosOffset);
    

    // Generate the code for the specified frames and capsule parameters, and compile it as library
    ADFun genFun = tapeADFunEnd2End(rmodel, hr_lower_leg, fl_upper_leg, capsLength, capsRadius, capsFrame);
    generateCompileCLib("end2end_" + frame1Name + frame2Name,genFun);
    // Print the C code to the console
    std::cout << "---- CODE GENERATION ----" << std::endl;
    std::cout << "// Generated end2end(q) :\n";
    std::cout << generateCSourceCode(genFun, rmodel.nq) << std::endl;

    std::cout << "---- CODE EVALUATION ----" << std::endl;

    const std::string LIBRARY_NAME = "./libCGend2end_" + frame1Name + frame2Name;
    const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    /***************************************************************************
     *                       Use the dynamic library
     **************************************************************************/
    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("end2end_" + frame1Name + frame2Name);

    // Generated code evaluation
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_test;
    X_test.resize(rmodel.nq);
        // Input : robot configuration
    X_test[0] = 0;
    X_test[1] = 0;
    X_test[2] = 0;
    X_test[3] = 0;
    X_test[4] = 0;
    X_test[5] = 0;
    X_test[6] = 0;
    X_test[7] = 0;
    X_test[8] = 0;
    X_test[9] = 0;
    X_test[10] = 0;
    X_test[11] = 0;
        // Output : distance
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_test;
    Y_test.resize(1);
    
    // Function evaluation with start and stop timestamps
    auto start = high_resolution_clock::now();
    model->ForwardZero(X_test, Y_test);
    auto stop = high_resolution_clock::now(); 
    auto duration = duration_cast<nanoseconds>(stop - start); 

    // Print output
    std::cout << "X = \n" << X_test << std::endl;
    std::cout << "\tDist. result : " << Y_test << std::endl;
    std::cout << "Time taken by function: " << duration.count() << " nanoseconds" << std::endl; 

}