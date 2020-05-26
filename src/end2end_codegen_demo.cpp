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

    // Initialize frame names
    const std::string frame1Name = argv[1];
    const std::string frame2Name = argv[2];

    // Capsule geometry
    const double CAPSULE_LENGTH = 0.2;
    const double CAPSULE_RADIUS = 0.02;

    // Initialize capsules placements :
        // Each of the capsule has to be placed and oriented in its leg frame
        // Translations
    Eigen::Matrix<double, 3, 1> FRAME1_TO_CAPS1_TRANSLATION;
    FRAME1_TO_CAPS1_TRANSLATION[0] = 0;
    FRAME1_TO_CAPS1_TRANSLATION[1] = 0;
    FRAME1_TO_CAPS1_TRANSLATION[2] = 0;

    Eigen::Matrix<double, 3, 1> FRAME2_TO_CAPS2_TRANSLATION;
    FRAME2_TO_CAPS2_TRANSLATION[0] = 0;
    FRAME2_TO_CAPS2_TRANSLATION[1] = 0;
    FRAME2_TO_CAPS2_TRANSLATION[2] = 0;
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

    // Define AD parameters
        // Geometry
    ADScalar ad_capsLength;
    ADScalar ad_capsRadius;
    ad_capsLength = CAPSULE_LENGTH;
    ad_capsRadius = CAPSULE_RADIUS;    
        // SE3 objects
    pinocchio::SE3Tpl<ADScalar> ad_f1Mcaps1(FRAME1_TO_CAPS1_ROTATION, FRAME1_TO_CAPS1_TRANSLATION);
    pinocchio::SE3Tpl<ADScalar> ad_f2Mcaps2(FRAME2_TO_CAPS2_ROTATION, FRAME2_TO_CAPS2_TRANSLATION);
    
    // Generate the code for the specified frames and capsule parameters, and compile it as library
    ADFun genFun = tapeADFunEnd2End(rmodel, frameInd1, frameInd2, ad_capsLength, ad_capsRadius, ad_f1Mcaps1, ad_f2Mcaps2);
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

        // Get a random config.
    srand((unsigned int) time(0));
    //X_test = 3.1415*Eigen::Matrix<double,12,1>::Random(12,1);

        // Output : distance
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_test;
    Y_test.resize(1);

    // Function evaluation with start and stop timestamps
    auto start_cg = high_resolution_clock::now();
    model->ForwardZero(X_test, Y_test);
    auto stop_cg = high_resolution_clock::now(); 
    auto duration_cg = duration_cast<nanoseconds>(stop_cg - start_cg); 

        // FCL check 
    auto start_fcl = high_resolution_clock::now();
    double fcl_dist = getFCLResult(rmodel,gmodel,rdata,X_test,frame1Name, frame2Name, CAPSULE_LENGTH, CAPSULE_RADIUS, f1Mcaps1, f2Mcaps2);
    auto stop_fcl = high_resolution_clock::now(); 
    auto duration_fcl = duration_cast<microseconds>(stop_fcl - start_fcl);

    // Print output
    std::cout << "X = \n" << X_test << std::endl;
    std::cout << "\tDist. result (codegen): " << Y_test << std::endl;
    std::cout << "Time taken by function: " << ((int)duration_cg.count())*0.001 << " microseconds" << std::endl; 
    std::cout << "\tDist. result (FCL): " << fcl_dist << std::endl;
    std::cout << "Time taken by function: " << duration_fcl.count() << " microseconds" << std::endl;
    std::cout << "\n" << std::endl;
    std::cout << "Dist. error = " << std::abs(fcl_dist - Y_test[0]) << std::endl;

    // TO MOVE IN UNIT TEST

        // Test on n random configurations
    int n = 100;
    double max_dist_err = 0;
    double dist_err;
    for(int k=0; k<n; k++)
    {
        X_test = 3.1415*Eigen::Matrix<double,12,1>::Random(12,1);

        model->ForwardZero(X_test, Y_test);
        fcl_dist = getFCLResult(rmodel,gmodel,rdata,X_test,frame1Name, frame2Name, CAPSULE_LENGTH, CAPSULE_RADIUS, f1Mcaps1, f2Mcaps2);

        dist_err = std::abs(fcl_dist - Y_test[0]);
        if(dist_err > max_dist_err)
        {
            max_dist_err = dist_err;
        }
    }
    std::cout << "Max error from " << n << " random config. evaluations : " << max_dist_err << std::endl;

}