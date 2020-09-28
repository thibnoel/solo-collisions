#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "end2end_legs_collision_check.hpp"
#include <chrono> 
using namespace std::chrono; 
using namespace pinocchio;

int main(int argc, char* argv[])
{
    // Initialize random seed 
    srand((unsigned int) time(0));

    // Check the number of parameters
    if (argc < 3) {
        // Provide usage feedback
        std::cerr << "Usage: " << argv[0] << " Frame1_NAME Frame2_NAME" << std::endl;
        return 1;
    }

    /***************************************************************************
     *                      Parameters initialization
     **************************************************************************/
    // Initialize frame names
    const std::string frame1Name = argv[1];
    const std::string frame2Name = argv[2];

    // Capsule geometry
    const double UPPER_CAPSULE_LENGTH = 0.2;
    const double UPPER_CAPSULE_RADIUS = 0.02;

    const double LOWER_CAPSULE_LENGTH = 0.165;
    const double LOWER_CAPSULE_RADIUS = 0.016;

    // Initialize capsules placements :
    // Each of the capsule has to be placed and oriented in its leg frame
        // Translations
    Eigen::Matrix<double, 3, 1> FRAME1_TO_CAPS1_TRANSLATION;
    FRAME1_TO_CAPS1_TRANSLATION[0] = 0.0;
    FRAME1_TO_CAPS1_TRANSLATION[1] = 0.0;
    FRAME1_TO_CAPS1_TRANSLATION[2] = 0.0;
    // Test with random capsule translation : FRAME1_TO_CAPS1_TRANSLATION = Eigen::Matrix<double,3,1>::Random(3,1);

    Eigen::Matrix<double, 3, 1> FRAME2_TO_CAPS2_TRANSLATION;
    FRAME2_TO_CAPS2_TRANSLATION[0] = 0.0;
    FRAME2_TO_CAPS2_TRANSLATION[1] = 0.0;
    FRAME2_TO_CAPS2_TRANSLATION[2] = 0.0;
    // Test with random capsule translation : FRAME2_TO_CAPS2_TRANSLATION = Eigen::Matrix<double,3,1>::Random(3,1);
        // Rotations
    Eigen::Quaternion<double> FRAME1_TO_CAPS1_ROTATION;
    FRAME1_TO_CAPS1_ROTATION.setIdentity();

    Eigen::Quaternion<double> FRAME2_TO_CAPS2_ROTATION;
    FRAME2_TO_CAPS2_ROTATION.setIdentity();
        // Initialize SE3 objects
    const pinocchio::SE3 f1Mcaps1(FRAME1_TO_CAPS1_ROTATION, FRAME1_TO_CAPS1_TRANSLATION);
    const pinocchio::SE3 f2Mcaps2(FRAME2_TO_CAPS2_ROTATION, FRAME2_TO_CAPS2_TRANSLATION);

    // Load Solo 12 model
        // Setup URDF path
    const std::string urdf_filename = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf";
    const std::string robots_model_path = "/opt/openrobots/share/example-robot-data/robots";// PINOCCHIO_MODEL_DIR + std::string("/others/robots/"); 

        // Load and build the Model and GeometryModel from URDF 
    Model rmodel;
    GeometryModel gmodel;
    pinocchio::urdf::buildModel(urdf_filename,rmodel);
    pinocchio::urdf::buildGeom(rmodel, urdf_filename, pinocchio::COLLISION, gmodel, robots_model_path);
        // Generate model data
    Data rdata(rmodel); 

    // Get frames indices from the model
    const int frameInd1 = (int)rmodel.getFrameId(frame1Name);
    const int frameInd2 = (int)rmodel.getFrameId(frame2Name);

    /***************************************************************************
     *                               Code generation
     **************************************************************************/
    // Define AD parameters
        // Geometry
    ADScalar ad_upperCapsLength;
    ADScalar ad_upperCapsRadius;
    ADScalar ad_lowerCapsLength;
    ADScalar ad_lowerCapsRadius;

    ad_upperCapsLength = UPPER_CAPSULE_LENGTH;
    ad_upperCapsRadius = UPPER_CAPSULE_RADIUS;
    ad_lowerCapsLength = LOWER_CAPSULE_LENGTH;
    ad_lowerCapsRadius = LOWER_CAPSULE_RADIUS;

        // SE3 objects
    pinocchio::SE3Tpl<ADScalar> ad_f1Mcaps1(FRAME1_TO_CAPS1_ROTATION, FRAME1_TO_CAPS1_TRANSLATION);
    pinocchio::SE3Tpl<ADScalar> ad_f2Mcaps2(FRAME2_TO_CAPS2_ROTATION, FRAME2_TO_CAPS2_TRANSLATION);
    
    // Generate the code for the specified frames and capsule parameters, and compile it as library
    ADFun genFun = tapeADFunEnd2End(rmodel, frameInd1, frameInd2, ad_upperCapsLength, ad_upperCapsRadius, ad_lowerCapsLength, ad_lowerCapsRadius, ad_f1Mcaps1, ad_f2Mcaps2);
    generateCompileCLib("end2end_" + frame1Name + "_" + frame2Name,genFun);
    // Print the C code to the console
    
    /*
    std::cout << "---- CODE GENERATION ----" << std::endl;
    std::cout << "// Generated end2end(q) :\n";
    std::cout << generateCSourceCode(genFun, rmodel.nq) << std::endl;
    */

    /***************************************************************************
     *                       Code generation - full legs
     **************************************************************************/
    std::string leg1 = "FR";
    std::string leg2 = "HR";

    ADFun genL2LFun = tapeADFunFullLeg(rmodel, leg1, leg2, ad_upperCapsLength, ad_upperCapsRadius, ad_lowerCapsLength, ad_lowerCapsRadius, ad_f1Mcaps1, ad_f2Mcaps2);
    generateCompileCLib("full_leg_" + leg1 + "_" + leg2,genL2LFun);
    
    /*
    std::cout << "---- CODE GENERATION ----" << std::endl;
    std::cout << "// Generated end2end FULL LEG(q) :\n";
    std::cout << generateCSourceCode(genL2LFun, rmodel.nq) << std::endl;
    */

    /***************************************************************************
     *                       Code generation - ALL legs
     **************************************************************************/
    ADFun genAllLegsFun = tapeADFunAllLegs(rmodel, ad_upperCapsLength, ad_upperCapsRadius, ad_lowerCapsLength, ad_lowerCapsRadius, ad_f1Mcaps1, ad_f2Mcaps2, 6);
    generateCompileCLib("allLegs",genAllLegsFun);

    /***************************************************************************
     *               Use the dynamic library and check the result
     *                              against FCL
     **************************************************************************/
    //const std::string LIBRARY_NAME = "./libCGend2end_" + frame1Name + "_" + frame2Name;
    //const std::string LIBRARY_NAME = "./libCGfull_leg_" + leg1 + "_" + leg2;
    const std::string LIBRARY_NAME = "./libCGallLegs";
    const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;

    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    //std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("end2end_" + frame1Name + "_" + frame2Name);
    //std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("full_leg_" + leg1 + "_" + leg2);
    std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("allLegs");

    // Generated code evaluation
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_test;
    X_test.resize(rmodel.nq);
        // Input : robot configuration
    /*X_test[0] = 0;
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
    X_test[11] = 0;*/

        // Input : Get a random config.
    X_test = 3.1415*Eigen::Matrix<double,12,1>::Random(12,1);

        // Output : distance
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_test;
    //Y_test.resize(1);
    //Y_test.resize(4);
    Y_test.resize(64);

    // Function evaluation with start and stop timestamps
    auto start_cg = high_resolution_clock::now();
    model->ForwardZero(X_test, Y_test);
    auto stop_cg = high_resolution_clock::now(); 
    auto duration_cg = duration_cast<nanoseconds>(stop_cg - start_cg); 

        // FCL check 
    auto start_fcl = high_resolution_clock::now();
    double fcl_dist = getPairFCLResult(rmodel,gmodel,rdata,X_test,frame1Name, frame2Name, UPPER_CAPSULE_LENGTH, UPPER_CAPSULE_RADIUS, LOWER_CAPSULE_LENGTH, LOWER_CAPSULE_RADIUS, f1Mcaps1, f2Mcaps2);
    auto stop_fcl = high_resolution_clock::now(); 
    auto duration_fcl = duration_cast<microseconds>(stop_fcl - start_fcl);

    // Print output
    std::cout << "---- CODE EVALUATION ----" << std::endl;
    std::cout << "X = \n" << X_test << std::endl;
    //std::cout << "\tDist. result (codegen): " << Y_test << std::endl;
    std::cout << "\tDist. result (codegen):\n" << Eigen::Map<const Eigen::Matrix<Scalar, 8, 8> >(Y_test.data()) << std::endl;
    std::cout << "Time taken by function: " << ((int)duration_cg.count())*0.001 << " microseconds" << std::endl; 
    std::cout << "\tDist. result (FCL): " << fcl_dist << std::endl;
    std::cout << "Time taken by function: " << duration_fcl.count() << " microseconds" << std::endl;
    std::cout << "\n" << std::endl;
    std::cout << "Dist. error = " << std::abs(fcl_dist - Y_test[0]) << std::endl;

    /***************************************************************************
     *                       TO MOVE IN UNIT TEST
     **************************************************************************/
        // Test on n random configurations
    int n = 100;
    double max_dist_err = 0;
    double dist_err;
    for(int k=0; k<n; k++)
    {
        X_test = 3.1415*Eigen::Matrix<double,12,1>::Random(12,1);

        model->ForwardZero(X_test, Y_test);
        fcl_dist = getPairFCLResult(rmodel,gmodel,rdata,X_test,frame1Name, frame2Name, UPPER_CAPSULE_LENGTH, UPPER_CAPSULE_RADIUS, LOWER_CAPSULE_LENGTH, LOWER_CAPSULE_RADIUS, f1Mcaps1, f2Mcaps2);

        dist_err = std::abs(fcl_dist - Y_test[0]);
        if(dist_err > max_dist_err)
        {
            max_dist_err = dist_err;
        }
    }
    std::cout << "Max error from " << n << " random config. evaluations : " << max_dist_err << std::endl;

}