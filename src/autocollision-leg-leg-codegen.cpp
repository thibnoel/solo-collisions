#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "autocollision-leg-leg-fcl-check.hpp"
#include <chrono> 
#include <sstream>

using namespace std::chrono; 
using namespace pinocchio;

/*
// Create a static list of the frames pairs for SOLO-12 collisions
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
*/

// SOLO12

// Create a static list of the geometries (capsules) pairs for SOLO-12 collisions
std::pair<int,int>* getSoloLegsGeomPairs(pinocchio::GeometryModel gmodel)
{
    // Frames pairs
    static std::pair<int,int> geomPairs[20] = {// FL - FR
                                            getGeomPair("FL_UPPER_LEG_0","FR_UPPER_LEG_0", gmodel),
                                            getGeomPair("FL_UPPER_LEG_0","FR_LOWER_LEG_0", gmodel),
                                            getGeomPair("FL_LOWER_LEG_0","FR_UPPER_LEG_0", gmodel),
                                            getGeomPair("FL_LOWER_LEG_0","FR_LOWER_LEG_0", gmodel),
                                            // FL - HL
                                            //getFramesPair("FL_UPPER_LEG","HL_UPPER_LEG", rmodel),
                                            getGeomPair("FL_UPPER_LEG_0","HL_LOWER_LEG_0", gmodel),
                                            getGeomPair("FL_LOWER_LEG_0","HL_UPPER_LEG_0", gmodel),
                                            getGeomPair("FL_LOWER_LEG_0","HL_LOWER_LEG_0", gmodel),
                                            // FL - HR
                                            //getFramesPair("FL_UPPER_LEG","HR_UPPER_LEG", rmodel),
                                            getGeomPair("FL_UPPER_LEG_0","HR_LOWER_LEG_0", gmodel),
                                            getGeomPair("FL_LOWER_LEG_0","HR_UPPER_LEG_0", gmodel),
                                            getGeomPair("FL_LOWER_LEG_0","HR_LOWER_LEG_0", gmodel),
                                            // FR - HL
                                            //getFramesPair("FR_UPPER_LEG","HL_UPPER_LEG", rmodel),
                                            getGeomPair("FR_UPPER_LEG_0","HL_LOWER_LEG_0", gmodel),
                                            getGeomPair("FR_LOWER_LEG_0","HL_UPPER_LEG_0", gmodel),
                                            getGeomPair("FR_LOWER_LEG_0","HL_LOWER_LEG_0", gmodel),
                                            // FR - HR
                                            //getFramesPair("FR_UPPER_LEG","HR_UPPER_LEG", rmodel),
                                            getGeomPair("FR_UPPER_LEG_0","HR_LOWER_LEG_0", gmodel),
                                            getGeomPair("FR_LOWER_LEG_0","HR_UPPER_LEG_0", gmodel),
                                            getGeomPair("FR_LOWER_LEG_0","HR_LOWER_LEG_0", gmodel),
                                            // HL - HR
                                            getGeomPair("HL_UPPER_LEG_0","HR_UPPER_LEG_0", gmodel),
                                            getGeomPair("HL_UPPER_LEG_0","HR_LOWER_LEG_0", gmodel),
                                            getGeomPair("HL_LOWER_LEG_0","HR_UPPER_LEG_0", gmodel),
                                            getGeomPair("HL_LOWER_LEG_0","HR_LOWER_LEG_0", gmodel)
                                            };
    return geomPairs;
}

/*
// Create a static list of the geometries (capsules) pairs for SOLO-12 collisions
std::pair<int,int>* getSoloLegsGeomPairs(pinocchio::GeometryModel gmodel)
{
    // Frames pairs
    static std::pair<int,int> geomPairs[6] = {// FL - FR
                                            //getGeomPair("FL_UPPER_LEG_0","FR_UPPER_LEG_0", gmodel),
                                            //getGeomPair("FL_UPPER_LEG_0","FR_LOWER_LEG_0", gmodel),
                                            //getGeomPair("FL_LOWER_LEG_0","FR_UPPER_LEG_0", gmodel),
                                            //getGeomPair("FL_LOWER_LEG_0","FR_LOWER_LEG_0", gmodel),
                                            // FL - HL
                                            //getFramesPair("FL_UPPER_LEG","HL_UPPER_LEG", rmodel),
                                            getGeomPair("FL_UPPER_LEG_0","HL_LOWER_LEG_0", gmodel),
                                            getGeomPair("FL_LOWER_LEG_0","HL_UPPER_LEG_0", gmodel),
                                            getGeomPair("FL_LOWER_LEG_0","HL_LOWER_LEG_0", gmodel),
                                            // FL - HR
                                            //getFramesPair("FL_UPPER_LEG","HR_UPPER_LEG", rmodel),
                                            //getGeomPair("FL_UPPER_LEG_0","HR_LOWER_LEG_0", gmodel),
                                            //getGeomPair("FL_LOWER_LEG_0","HR_UPPER_LEG_0", gmodel),
                                            //getGeomPair("FL_LOWER_LEG_0","HR_LOWER_LEG_0", gmodel),
                                            // FR - HL
                                            //getFramesPair("FR_UPPER_LEG","HL_UPPER_LEG", rmodel),
                                            //getGeomPair("FR_UPPER_LEG_0","HL_LOWER_LEG_0", gmodel),
                                            //getGeomPair("FR_LOWER_LEG_0","HL_UPPER_LEG_0", gmodel),
                                            //getGeomPair("FR_LOWER_LEG_0","HL_LOWER_LEG_0", gmodel),
                                            // FR - HR
                                            //getFramesPair("FR_UPPER_LEG","HR_UPPER_LEG", rmodel),
                                            getGeomPair("FR_UPPER_LEG_0","HR_LOWER_LEG_0", gmodel),
                                            getGeomPair("FR_LOWER_LEG_0","HR_UPPER_LEG_0", gmodel),
                                            getGeomPair("FR_LOWER_LEG_0","HR_LOWER_LEG_0", gmodel),
                                            // HL - HR
                                            //getGeomPair("HL_UPPER_LEG_0","HR_UPPER_LEG_0", gmodel),
                                            //getGeomPair("HL_UPPER_LEG_0","HR_LOWER_LEG_0", gmodel),
                                            //getGeomPair("HL_LOWER_LEG_0","HR_UPPER_LEG_0", gmodel),
                                            //getGeomPair("HL_LOWER_LEG_0","HR_LOWER_LEG_0", gmodel)
                                            };
    return geomPairs;
}
*/

// MAIN
int main(int argc, char *argv[])
{
    // Check the number of parameters
    // Parameter : number of pairs out of 20 to be generated (performance tests)
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

        // Original model
    //const std::string urdf_filename = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf";
    //const std::string robots_model_path = "/opt/openrobots/share/example-robot-data/robots";

        // Simplified model
        // The capsules geometries needed for the coe generation are contained in the simplified 
        // URDF file provided here ! It does not work with the original meshes-described URDF
    const std::string urdf_filename = "/home/tnoel/stage/solo-collisions/urdf/solo12_simplified.urdf";
    //const std::string urdf_filename = "/home/tnoel/stage/solo-collisions/urdf/solo8_simplified.urdf";
    const std::string robots_model_path = "/opt/openrobots/share/example-robot-data/robots";

        // Load and build the Model and GeometryModel from URDF 
    Model rmodel;
    GeometryModel gmodel;
    pinocchio::urdf::buildModel(urdf_filename,rmodel);
    pinocchio::urdf::buildGeom(rmodel, urdf_filename, pinocchio::COLLISION, gmodel, robots_model_path);
        // Generate model data
    Data rdata(rmodel); 
    GeometryData gdata(gmodel);

    // Predefine frames and capsules pairs (nb of pairs evaluated chosen in tapeADFun)
        // Predefined frames pairs
    //std::pair<int,int>* framesPairs = getSoloLegsFramesPairs(rmodel);
        // Predefined geometries pairs
    std::pair<int,int>* geomPairs = getSoloLegsGeomPairs(gmodel);

    /***************************************************************************
    *                               Code generation
    ***************************************************************************/
    // Tape the model to generate
    //ADFun genFun = tapeADCapsulesCollisionComputation(rmodel, gmodel, framesPairs, geomPairs, nb_pairs);
    ADFun genFun = tapeADCapsulesCollisionComputation(rmodel, gmodel, geomPairs, nb_pairs);
    generateCompileCLib("solo_autocollision_legs_legs", genFun);

    /***************************************************************************
    *                      Generated code evaluation
    ***************************************************************************/
    // Load the model from the compiled library
    const std::string LIBRARY_NAME = "./libCGsolo_autocollision_legs_legs";
    const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;
    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > model = dynamicLib.model("solo_autocollision_legs_legs");

    // Input : Get a random config.
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_test;
    X_test = 3.1415*Eigen::Matrix<double,12,1>::Random(12,1);
    //X_test = 3.1415*Eigen::Matrix<double,8,1>::Random(8,1);
    //X_test = Eigen::Matrix<double,12,1>::Zero(12,1);
    
    // Output : distance
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_test;
    //Y_test.resize(20*(1+3*12));
    Y_test.resize(20*(1+12+6));
    //Y_test.resize(6*(1+8+6));
    
    // Function evaluation with start and stop timestamps
    auto start_cg = high_resolution_clock::now();
    for (int k = 0; k<1e6; k++)
    {
        X_test = 3.1415*Eigen::Matrix<double,12,1>::Random(12,1);
        //X_test = 3.1415*Eigen::Matrix<double,8,1>::Random(8,1);
        model->ForwardZero(X_test, Y_test);
    }
    auto stop_cg = high_resolution_clock::now(); 
    auto duration_cg = duration_cast<microseconds>(stop_cg - start_cg); 

    /***************************************************************************
    *                       FCL evaluation and comparison
    ***************************************************************************/
    Eigen::Matrix<double, 8, 8> fcl_result;
    // Compute groundtruth result for the distance with FCL
    fcl_result = getSoloFCLResult(rmodel,gmodel,rdata,X_test);

    // Print output
    std::cout << "---- CODE EVALUATION ----" << std::endl;
    std::cout << "X = \n" << X_test << std::endl;

    std::cout << "\nDist. result (codegen): \n" << std::endl;

    std::string sep = "\n######################################\n";
    for(int k = 0; k<nb_pairs; k++)
    {
        std::cout << "Pair " << k << std::endl; 
        //std::cout << "Dist : \n" << Y_test(k*37,0) << "\n" << std::endl; 
        //std::cout << "Jacobian : \n" << Eigen::Map<const Eigen::Matrix<Scalar, 3, 12> >(Y_test.block(1+k*37, 0, 36, 1).data()) << "\n" << std::endl; 
        //std::cout << "Dist : \n" << Y_test(k*13,0) << "\n" << std::endl; 
        std::cout << "Dist : \n" << Y_test(k*19,0) << "\n" << std::endl; 
        //std::cout << "Jacobian : \n" << Eigen::Map<const Eigen::Matrix<Scalar, 1, 12> >(Y_test.block(1+k*13, 0, 12, 1).data()) << "\n" << std::endl;
        std::cout << "Jacobian : \n" << Eigen::Map<const Eigen::Matrix<Scalar, 1, 8> >(Y_test.block(1+k*19, 0, 12, 1).data()) << "\n" << std::endl;
        std::cout << "Witness point 1 : \n" << Eigen::Map<const Eigen::Matrix<Scalar, 1, 3> >(Y_test.block(1+k*19+12, 0, 3, 1).data()) << "\n" << std::endl;
        std::cout << "Witness point 2 : \n" << Eigen::Map<const Eigen::Matrix<Scalar, 1, 3> >(Y_test.block(1+k*19+12+3, 0, 3, 1).data()) << "\n" << std::endl;

        std::cout << sep << std::endl; 
    }

    std::cout << "\n\tDist. result (FCL): \n" << fcl_result << std::endl;
    std::cout << "\n" << std::endl; 

    std::cout << "\nTime taken by function (1e6 executions): " << ((int)duration_cg.count()) << " microseconds" << std::endl; 
      
}