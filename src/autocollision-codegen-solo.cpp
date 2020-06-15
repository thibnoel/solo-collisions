#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "autocollision-code-generation.hpp"
#include <chrono> 

using namespace std::chrono; 
using namespace pinocchio;

int main()
{
    // Initialize random seed 
    srand((unsigned int) time(0));

    /***************************************************************************
    *                      Parameters initialization
    ***************************************************************************/
    // Load Solo 12 model
        // Setup URDF path
    const std::string urdf_filename = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf";
    const std::string robots_model_path = "/opt/openrobots/share/example-robot-data/robots";

        // Load and build the Model and GeometryModel from URDF 
    Model rmodel;
    GeometryModel gmodel;
    pinocchio::urdf::buildModel(urdf_filename,rmodel);
    pinocchio::urdf::buildGeom(rmodel, urdf_filename, pinocchio::COLLISION, gmodel, robots_model_path);
        // Generate model data
    Data rdata(rmodel); 

    // Capsules geometries
    const double UPPER_CAPSULE_RADIUS = 0.02;
    Eigen::Matrix<double, 3, 1> UPPER_CAPSULE_A;
    Eigen::Matrix<double, 3, 1> UPPER_CAPSULE_B;
    UPPER_CAPSULE_A << 0.0, 0.0, 0.1;
    UPPER_CAPSULE_B << 0.0, 0.0, -0.1;
    
    const double LOWER_CAPSULE_RADIUS = 0.016;
    Eigen::Matrix<double, 3, 1> LOWER_CAPSULE_A;
    Eigen::Matrix<double, 3, 1> LOWER_CAPSULE_B;
    LOWER_CAPSULE_A << 0.0, 0.0, 0.08;
    LOWER_CAPSULE_B << 0.0, 0.0, -0.08;
    
   
    /***************************************************************************
    *                               Code generation
    ***************************************************************************/

     // Frames pairs
    std::pair<int,int> framesPairs[4] = {getFramesPair("FL_UPPER_LEG","FR_UPPER_LEG", rmodel),
                                          getFramesPair("FL_UPPER_LEG","FR_LOWER_LEG", rmodel),
                                          getFramesPair("FL_LOWER_LEG","FR_UPPER_LEG", rmodel),
                                          getFramesPair("FL_LOWER_LEG","FR_LOWER_LEG", rmodel)};

    // Predefine frames and capsules pairs (nb of pairs evaluated chosen in tapeADFun)
    // Predefined double capsules
    struct Capsule<double> LeftUpperCaps = {UPPER_CAPSULE_A, UPPER_CAPSULE_B, UPPER_CAPSULE_RADIUS};
    struct Capsule<double> RightUpperCaps = {UPPER_CAPSULE_A, UPPER_CAPSULE_B, UPPER_CAPSULE_RADIUS};
    struct Capsule<double> LeftLowerCaps = {LOWER_CAPSULE_A, LOWER_CAPSULE_B, LOWER_CAPSULE_RADIUS};
    struct Capsule<double> RightLowerCaps = {LOWER_CAPSULE_A, LOWER_CAPSULE_B, LOWER_CAPSULE_RADIUS};

        // Predefined AD capsules
    struct Capsule<ADScalar> ADLeftUpperCaps = LeftUpperCaps.cast<ADScalar>(); //{ad_upperCapsLength, ad_upperCapsRadius, ad_f1Mcaps1};
    struct Capsule<ADScalar> ADRightUpperCaps = RightUpperCaps.cast<ADScalar>();//{ad_upperCapsLength, ad_upperCapsRadius, ad_f2Mcaps2};
    struct Capsule<ADScalar> ADLeftLowerCaps = LeftLowerCaps.cast<ADScalar>();//{ad_lowerCapsLength, ad_lowerCapsRadius, ad_f1Mcaps1};
    struct Capsule<ADScalar> ADRightLowerCaps = RightLowerCaps.cast<ADScalar>();//{ad_lowerCapsLength, ad_lowerCapsRadius, ad_f2Mcaps2};
       
        // AD Capsules pairs (code gen arg.)
    std::pair<Capsule<ADScalar>,Capsule<ADScalar>> ADCapsPairs[4] = {std::make_pair(ADLeftUpperCaps, ADRightUpperCaps),
                                                                    std::make_pair(ADLeftUpperCaps, ADRightLowerCaps),
                                                                    std::make_pair(ADLeftLowerCaps, ADRightUpperCaps),
                                                                    std::make_pair(ADLeftLowerCaps, ADRightLowerCaps)};
    
       
        // double Capsules pairs (FCL check arg.)
    std::pair<Capsule<double>,Capsule<double>> doubleCapsPairs[4] = {std::make_pair(LeftUpperCaps, RightUpperCaps),
                                                                    std::make_pair(LeftUpperCaps, RightLowerCaps),
                                                                    std::make_pair(LeftLowerCaps, RightUpperCaps),
                                                                    std::make_pair(LeftLowerCaps, RightLowerCaps)};

    // Generate the code for the specified frames and capsule parameters, and compile it as library
    ADFun genFun = tapeADCapsulesDistanceCheck(rmodel, framesPairs, ADCapsPairs, 4);
    //std::cout << generateCSourceCode(genFun, rmodel.nq) << std::endl;
    generateCompileCLib("autocollision_solo",genFun);

    /***************************************************************************
    *                      Generated code evaluation
    ***************************************************************************/
    const std::string LIBRARY_NAME = "./libCGautocollision_solo";
    const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("autocollision_solo");

    // Generated code evaluation
        // Input : Get a random config.
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_test;
    X_test.resize(rmodel.nq);
    X_test = 3.1415*Eigen::Matrix<double,12,1>::Random(12,1);
    //X_test = Eigen::Matrix<double,12,1>::Zero(12,1);
        // Output : distance
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_test;
    Y_test.resize(4);

    // Function evaluation with start and stop timestamps
    auto start_cg = high_resolution_clock::now();
    model->ForwardZero(X_test, Y_test);
    auto stop_cg = high_resolution_clock::now(); 
    auto duration_cg = duration_cast<nanoseconds>(stop_cg - start_cg); 

    /***************************************************************************
    *                              FCL evaluation
    ***************************************************************************/
    double fcl_dist0;
    fcl_dist0 = getPairFCLResult(rmodel,gmodel,rdata,X_test, framesPairs[0], doubleCapsPairs[0]);
    // Print output
    std::cout << "---- CODE EVALUATION ----" << std::endl;
    std::cout << "X = \n" << X_test << std::endl;
    std::cout << "\tDist. result (codegen): \n" << Y_test << std::endl;
    std::cout << "Time taken by function: " << ((int)duration_cg.count())*0.001 << " microseconds" << std::endl; 
    std::cout << "\tDist. result (FCL): \n" << fcl_dist0 << std::endl;
    std::cout << "\n" << std::endl;    

}