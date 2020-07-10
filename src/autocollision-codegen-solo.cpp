#include <stdlib.h>
#include <iostream>
#include <fstream>
//#include "autocollision-code-generation.hpp"
#include "autocollision-fcl-check.hpp"
#include <chrono> 
#include <sstream>

using namespace std::chrono; 
using namespace pinocchio;

std::pair<int,int>* getSoloLegsFramesPairs(pinocchio::Model rmodel)
{
    // Frames pairs
    static std::pair<int,int> framesPairs[20] = {// FL - FR
                                          getFramesPair("FL_UPPER_LEG","FR_UPPER_LEG", rmodel),
                                          getFramesPair("FL_UPPER_LEG","FR_LOWER_LEG", rmodel),
                                          getFramesPair("FL_LOWER_LEG","FR_UPPER_LEG", rmodel),
                                          getFramesPair("FL_LOWER_LEG","FR_LOWER_LEG", rmodel),
                                          // FL - HL
                                          //getFramesPair("FL_UPPER_LEG","HL_UPPER_LEG", rmodel),
                                          getFramesPair("FL_UPPER_LEG","HL_LOWER_LEG", rmodel),
                                          getFramesPair("FL_LOWER_LEG","HL_UPPER_LEG", rmodel),
                                          getFramesPair("FL_LOWER_LEG","HL_LOWER_LEG", rmodel),
                                          // FL - HR
                                          //getFramesPair("FL_UPPER_LEG","HR_UPPER_LEG", rmodel),
                                          getFramesPair("FL_UPPER_LEG","HR_LOWER_LEG", rmodel),
                                          getFramesPair("FL_LOWER_LEG","HR_UPPER_LEG", rmodel),
                                          getFramesPair("FL_LOWER_LEG","HR_LOWER_LEG", rmodel),
                                          // FR - HL
                                          //getFramesPair("FR_UPPER_LEG","HL_UPPER_LEG", rmodel),
                                          getFramesPair("FR_UPPER_LEG","HL_LOWER_LEG", rmodel),
                                          getFramesPair("FR_LOWER_LEG","HL_UPPER_LEG", rmodel),
                                          getFramesPair("FR_LOWER_LEG","HL_LOWER_LEG", rmodel),
                                          // FR - HR
                                          //getFramesPair("FR_UPPER_LEG","HR_UPPER_LEG", rmodel),
                                          getFramesPair("FR_UPPER_LEG","HR_LOWER_LEG", rmodel),
                                          getFramesPair("FR_LOWER_LEG","HR_UPPER_LEG", rmodel),
                                          getFramesPair("FR_LOWER_LEG","HR_LOWER_LEG", rmodel),
                                          // HL - HR
                                          getFramesPair("HL_UPPER_LEG","HR_UPPER_LEG", rmodel),
                                          getFramesPair("HL_UPPER_LEG","HR_LOWER_LEG", rmodel),
                                          getFramesPair("HL_LOWER_LEG","HR_UPPER_LEG", rmodel),
                                          getFramesPair("HL_LOWER_LEG","HR_LOWER_LEG", rmodel)
                                          };
    return framesPairs;
}

std::pair<Capsule<ADScalar>,Capsule<ADScalar>>* getSoloLegsADCapsPairs(Capsule<ADScalar>* ADCapsApprox)
{
    // AD capsules pairs (code gen. arg.)
    static std::pair<Capsule<ADScalar>,Capsule<ADScalar>> ADCapsPairs[20] = {std::make_pair(ADCapsApprox[0], ADCapsApprox[2]),
                                                                    std::make_pair(ADCapsApprox[0], ADCapsApprox[3]),
                                                                    std::make_pair(ADCapsApprox[1], ADCapsApprox[2]),
                                                                    std::make_pair(ADCapsApprox[1], ADCapsApprox[3]),

                                                                    //std::make_pair(ADCapsApprox[0], ADCapsApprox[0]),
                                                                    std::make_pair(ADCapsApprox[0], ADCapsApprox[1]),
                                                                    std::make_pair(ADCapsApprox[1], ADCapsApprox[0]),
                                                                    std::make_pair(ADCapsApprox[1], ADCapsApprox[1]),

                                                                    //std::make_pair(ADCapsApprox[0], ADCapsApprox[2]),
                                                                    std::make_pair(ADCapsApprox[0], ADCapsApprox[3]),
                                                                    std::make_pair(ADCapsApprox[1], ADCapsApprox[2]),
                                                                    std::make_pair(ADCapsApprox[1], ADCapsApprox[3]),

                                                                    //std::make_pair(ADCapsApprox[2], ADCapsApprox[0]),
                                                                    std::make_pair(ADCapsApprox[2], ADCapsApprox[1]),
                                                                    std::make_pair(ADCapsApprox[3], ADCapsApprox[0]),
                                                                    std::make_pair(ADCapsApprox[3], ADCapsApprox[1]),

                                                                    //std::make_pair(ADCapsApprox[2], ADCapsApprox[2]),
                                                                    std::make_pair(ADCapsApprox[2], ADCapsApprox[3]),
                                                                    std::make_pair(ADCapsApprox[3], ADCapsApprox[2]),
                                                                    std::make_pair(ADCapsApprox[3], ADCapsApprox[3]),

                                                                    std::make_pair(ADCapsApprox[0], ADCapsApprox[2]),
                                                                    std::make_pair(ADCapsApprox[0], ADCapsApprox[3]),
                                                                    std::make_pair(ADCapsApprox[1], ADCapsApprox[2]),
                                                                    std::make_pair(ADCapsApprox[1], ADCapsApprox[3])
                                                                    };
    return ADCapsPairs;
}

Eigen::Matrix<double, 8, 8> reshapeCodegenResult(Eigen::Matrix<double, Eigen::Dynamic, 1> y)
{
    Eigen::Matrix<double, 8, 8> result;
    result << 0.0 , 0.0 , y[0], y[1], y[4] , y[5] , y[8] , y[9],
              0.0 , 0.0 , y[2], y[3], y[6] , y[7] , y[10], y[11],
              y[0], y[2], 0.0 , 0.0 , y[12], y[13], y[16], y[17],
              y[1], y[3], 0.0 , 0.0 , y[14], y[15], y[18], y[19],
              y[4], y[6], y[12], y[14], 0.0, 0.0 , y[20] , y[21],
              y[5], y[7], y[13], y[15], 0.0, 0.0 , y[22] , y[23],
              y[8],y[10], y[16], y[18], y[20], y[22], 0.0, 0.0 ,  
              y[9],y[11], y[17], y[19], y[21], y[23], 0.0, 0.0 ;
    return result;

}

int main(int argc, char *argv[])
{
    // Check the number of parameters
    if (argc < 2) {
        // Provide usage feedback
        std::cerr << "Usage: " << argv[0] << " nb_pairs" << std::endl;
        return 1;
    }
    std::istringstream ss(argv[1]);
    int nb_pairs;
    if (!(ss >> nb_pairs)) {
        std::cerr << "Invalid number: " << argv[1] << '\n';
    } else if (!ss.eof()) {
        std::cerr << "Trailing characters after number: " << argv[1] << '\n';
    }

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
    const double LEFT_UPPER_CAPSULE_RADIUS = 0.016;
    Eigen::Matrix<double, 3, 1> LEFT_UPPER_CAPSULE_A;
    Eigen::Matrix<double, 3, 1> LEFT_UPPER_CAPSULE_B;
    LEFT_UPPER_CAPSULE_A << 0.0, 0.0, 0.1;
    LEFT_UPPER_CAPSULE_B << 0.0, 0.0, -0.1;
    
    const double LEFT_LOWER_CAPSULE_RADIUS = 0.015;
    Eigen::Matrix<double, 3, 1> LEFT_LOWER_CAPSULE_A;
    Eigen::Matrix<double, 3, 1> LEFT_LOWER_CAPSULE_B;
    LEFT_LOWER_CAPSULE_A << 0.0, 0.0, 0.07;
    LEFT_LOWER_CAPSULE_B << 0.0, 0.0, -0.07;

    const double RIGHT_UPPER_CAPSULE_RADIUS = LEFT_UPPER_CAPSULE_RADIUS;
    Eigen::Matrix<double, 3, 1> RIGHT_UPPER_CAPSULE_A;
    Eigen::Matrix<double, 3, 1> RIGHT_UPPER_CAPSULE_B;
    RIGHT_UPPER_CAPSULE_A << 0.0, 0.0, 0.1;
    RIGHT_UPPER_CAPSULE_B << 0.0, 0.0, -0.1;
    
    const double RIGHT_LOWER_CAPSULE_RADIUS = LEFT_LOWER_CAPSULE_RADIUS;
    Eigen::Matrix<double, 3, 1> RIGHT_LOWER_CAPSULE_A;
    Eigen::Matrix<double, 3, 1> RIGHT_LOWER_CAPSULE_B;
    RIGHT_LOWER_CAPSULE_A << 0.0, 0.0, 0.07;
    RIGHT_LOWER_CAPSULE_B << 0.0, 0.0, -0.07;

    // Lower legs spheres (base-lower legs pairs)
    const double LOW_LEGS_SPHERE_RADIUS = 0.015;
    Eigen::Matrix<double, 3, 1> LOW_LEGS_SPHERE_CENTER;
    LOW_LEGS_SPHERE_CENTER << 0.0, 0.0, -0.07;

    // Base link rectangleSweptShere dimensions
    const double BASE_LENGTH = 0.3;
    const double BASE_WIDTH = 0.16;
    const double BASE_RADIUS = 0.02;
    Eigen::Matrix<double, 3, 1> BASE_RECT_TOPLEFT;
    BASE_RECT_TOPLEFT << -BASE_LENGTH*0.5, -BASE_WIDTH*0.5, 0.0;
    
   
    /***************************************************************************
    *                               Code generation
    ***************************************************************************/
    // Predefine frames and capsules pairs (nb of pairs evaluated chosen in tapeADFun)
        // Predefined frames pairs
    std::pair<int,int>* framesPairs = getSoloLegsFramesPairs(rmodel);
        // Predefined double capsules
    struct Capsule<double> LeftUpperCaps = {LEFT_UPPER_CAPSULE_A, LEFT_UPPER_CAPSULE_B, LEFT_UPPER_CAPSULE_RADIUS};
    struct Capsule<double> LeftLowerCaps = {LEFT_LOWER_CAPSULE_A, LEFT_LOWER_CAPSULE_B, LEFT_LOWER_CAPSULE_RADIUS};
    struct Capsule<double> RightUpperCaps = {RIGHT_UPPER_CAPSULE_A, RIGHT_UPPER_CAPSULE_B, RIGHT_UPPER_CAPSULE_RADIUS};
    struct Capsule<double> RightLowerCaps = {RIGHT_LOWER_CAPSULE_A, RIGHT_LOWER_CAPSULE_B, RIGHT_LOWER_CAPSULE_RADIUS};

    struct Capsule<double> capsulesApprox[4] = {LeftUpperCaps, LeftLowerCaps, RightUpperCaps, RightLowerCaps};
        // Predefined AD capsules
    struct Capsule<ADScalar> ADLeftUpperCaps = LeftUpperCaps.cast<ADScalar>(); 
    struct Capsule<ADScalar> ADRightUpperCaps = RightUpperCaps.cast<ADScalar>();
    struct Capsule<ADScalar> ADLeftLowerCaps = LeftLowerCaps.cast<ADScalar>();
    struct Capsule<ADScalar> ADRightLowerCaps = RightLowerCaps.cast<ADScalar>();

    struct Capsule<ADScalar> ADCapsulesApprox[4] = {ADLeftUpperCaps, ADLeftLowerCaps, ADRightUpperCaps, ADRightLowerCaps};
       
        // AD Capsules pairs (code gen. arg.)
    std::pair<Capsule<ADScalar>,Capsule<ADScalar>>* ADCapsPairs = getSoloLegsADCapsPairs(ADCapsulesApprox);       


    /*------- TEST : sphere-RSS lower legs collision -----*/      

    /*std::pair<int,int> framesBaseLL[4] = {getFramesPair("base_link","FL_LOWER_LEG", rmodel), 
                                          getFramesPair("base_link","FR_LOWER_LEG", rmodel),
                                          getFramesPair("base_link","HL_LOWER_LEG", rmodel),
                                          getFramesPair("base_link","HR_LOWER_LEG", rmodel)};*/


    struct RectSweptSph<double> base_rss = {BASE_RECT_TOPLEFT, BASE_LENGTH, BASE_WIDTH, BASE_RADIUS};
    struct Sphere<double> fl_lower_sph = {LOW_LEGS_SPHERE_CENTER, LOW_LEGS_SPHERE_RADIUS};

    struct RectSweptSph<ADScalar> AD_base_rss = base_rss.cast<ADScalar>();
    struct Sphere<ADScalar> AD_fl_lower_sph = fl_lower_sph.cast<ADScalar>();                                               

    // Generate the code for the specified frames and capsule parameters, and compile it as library
    ADFun genFun = tapeADCapsulesDistanceCheck(rmodel, framesPairs, ADCapsPairs, nb_pairs);
    generateCompileCLib("solo_autocollision", genFun);

    //ADFun genFun = tapeADPointRSSMultDistanceCheck(rmodel, framesBaseLL, AD_base_rss, AD_fl_lower_sph, 4);
    //generateCompileCLib("solo_autocollision_LL_BASE", genFun);

    // Test Jacobian
    // TODO
    //ADFun genFun = tapeADJacobianDistanceCheck(rmodel, framesPairs[0], std::make_pair(LEFT_UPPER_CAPSULE_A, RIGHT_UPPER_CAPSULE_A));
    //generateCompileCLib("solo_autocollision_Jacobian", genFun);

    /***************************************************************************
    *                      Generated code evaluation
    ***************************************************************************/
    const std::string LIBRARY_NAME = "./libCGsolo_autocollision";
    const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("solo_autocollision");

    /*const std::string LIBRARY_NAME = "./libCGsolo_autocollision_LL_BASE";
    const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("solo_autocollision_LL_BASE");*/

    /*const std::string LIBRARY_NAME = "./libCGsolo_autocollision_Jacobian";
    const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("solo_autocollision_Jacobian");*/

    // Generated code evaluation
        // Input : Get a random config.
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_test;
    //X_test.resize(rmodel.nq);
    X_test = 3.1415*Eigen::Matrix<double,12,1>::Random(12,1);
    //X_test = Eigen::Matrix<double,12,1>::Zero(12,1);
        // Output : distance
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_test;
    Y_test.resize(20);
    //Y_test.resize(36);

    // Function evaluation with start and stop timestamps
    auto start_cg = high_resolution_clock::now();
    model->ForwardZero(X_test, Y_test);
    auto stop_cg = high_resolution_clock::now(); 
    auto duration_cg = duration_cast<nanoseconds>(stop_cg - start_cg); 


    /***************************************************************************
    *                              FCL evaluation
    ***************************************************************************/
    Eigen::Matrix<double, 8, 8> fcl_result;
    fcl_result = getSoloFCLResult(rmodel,gmodel,rdata,X_test,capsulesApprox);
    // Print output
    std::cout << "---- CODE EVALUATION ----" << std::endl;
    std::cout << "X = \n" << X_test << std::endl;
    //std::cout << "\tDist. result (codegen): \n" << reshapeCodegenResult(Y_test) << std::endl;
    std::cout << "\tDist. result (codegen): \n" << Y_test<< std::endl;
    std::cout << "\nTime taken by function: " << ((int)duration_cg.count())*0.001 << " microseconds" << std::endl; 
    std::cout << "\n\tDist. result (FCL): \n" << fcl_result << std::endl;
    std::cout << "\n" << std::endl;    

}