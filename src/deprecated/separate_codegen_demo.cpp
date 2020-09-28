#include <iostream>
#include "relative-placement-codegen.hpp"
#include "segment-segment-distance-codegen.hpp"
#include <chrono> 

using namespace std::chrono; 
using namespace pinocchio;

// For now, this main only generates the relative placement code as a string and prints it to the console
int main()
{
    /*************************** Forward kinematics code generation *****************************/
    // Load Solo 12 model
        // Setup URDF path
    const std::string urdf_filename = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf";

        // Load and build the URDF model
    Model rmodel;
    pinocchio::urdf::buildModel(urdf_filename,rmodel);
        // Generate model data
    Data rdata(rmodel); 

    // Get frames indices from the model
    int fl_lower_leg = (int)rmodel.getFrameId("FL_LOWER_LEG");
    int hl_lower_leg = (int)rmodel.getFrameId("HL_LOWER_LEG");
    //int base_link = (int)rmodel.getFrameId("base_link");

    /*// Generate the code for the specified frames and compile it as library
    ADFun relPlacementFun1 = tapeADFunRelativePlacement(rmodel, base_link, fl_upper_leg);
    generateCompileCLib("rel_placement",relPlacementFun1);
    // Print the C code to the console
    std::cout << "// Generated rel_placement(q) for frames BASE_LINK and FL_UPPER_LEG :\n";
    std::cout << generateCSourceCode(relPlacementFun1, rmodel.nq) << std::endl;
    */

    // One more example, with another collision pair
    ADFun relPlacementFun2 = tapeADFunRelativePlacement(rmodel, fl_lower_leg, hl_lower_leg);
    generateCompileCLib("rel_placement",relPlacementFun2);
    // Print the C code to the console
    std::cout << "// Generated rel_placement(q) for frames HR_LOWER_LEG and FL_UPPER_LEG :\n";
    std::cout << generateCSourceCode(relPlacementFun2, rmodel.nq) << std::endl;

    /*************************** Segment-segment dist. code generation *****************************/
    // Generate segment segment distance code and compile it as library
    ADFun segDistFun = tapeADFunSegSegDist();
    generateCompileCLib("ssd",segDistFun);
    // Print the C code to the console
    std::cout << "// Generated seg_seg_dist(seg1, seg2) :\n";
    std::cout << generateCSourceCode(segDistFun, 12) << std::endl;

    /*************************** Code evaluation *****************************/
    const std::string LIBRARY_NAME_RP = "./libCGrel_placement";
    const std::string LIBRARY_NAME_RP_EXT = LIBRARY_NAME_RP + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;

    CppAD::cg::LinuxDynamicLib<double> dynamicLibRP(LIBRARY_NAME_RP_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > modelRP = dynamicLibRP.model("rel_placement");

    const std::string LIBRARY_NAME_SSD = "./libCGssd";
    const std::string LIBRARY_NAME_SSD_EXT = LIBRARY_NAME_SSD + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;

    CppAD::cg::LinuxDynamicLib<double> dynamicLibSSD(LIBRARY_NAME_SSD_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double> > modelSSD = dynamicLibSSD.model("ssd");

    // Generated code evaluation
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_fk;
    X_fk.resize(rmodel.nq);
        // Input : robot configuration
    /*X_fk[0] = 0;
    X_fk[1] = 0;
    X_fk[2] = 0;
    X_fk[3] = 0;
    X_fk[4] = 0;
    X_fk[5] = 0;
    X_fk[6] = 0;
    X_fk[7] = 0;
    X_fk[8] = 0;
    X_fk[9] = 0;
    X_fk[10] = 0;
    X_fk[11] = 0;*/

        // Input : Get a random config.
    X_fk = 3.1415*Eigen::Matrix<double,12,1>::Random(12,1);

        // Output : SE3
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_fk;
    Y_fk.resize(16);

    // Rel. placement evaluation with start and stop timestamps
    auto start_fk = high_resolution_clock::now();
    modelRP->ForwardZero(X_fk, Y_fk);
    auto stop_fk = high_resolution_clock::now(); 
    auto duration_fk = duration_cast<nanoseconds>(stop_fk - start_fk); 

    // Rel. placement evaluation with start and stop timestamps
    auto start_pio_fk = high_resolution_clock::now();
    SE3 Y_pio_fk = relativePlacement(rmodel, rdata, X_fk, fl_lower_leg, hl_lower_leg);
    auto stop_pio_fk = high_resolution_clock::now(); 
    auto duration_pio_fk = duration_cast<nanoseconds>(stop_pio_fk - start_pio_fk); 

    Eigen::Matrix<double, Eigen::Dynamic, 1> X_ssd;
    X_ssd.resize(12);
    Eigen::Matrix<double, Eigen::Dynamic, 1> Y_ssd;
    Y_ssd.resize(1);

    X_ssd = 5*Eigen::Matrix<double,12,1>::Random(12,1);

    // SSD evaluation with start and stop timestamps
    auto start_ssd = high_resolution_clock::now();
    modelSSD->ForwardZero(X_ssd, Y_ssd);
    auto stop_ssd = high_resolution_clock::now(); 
    auto duration_ssd = duration_cast<nanoseconds>(stop_ssd - start_ssd); 

    // Print output
    std::cout << "---- CODE EVALUATION ----" << std::endl;
    std::cout << "X_fk = \n" << X_fk << std::endl;
    std::cout << "X_ssd = \n" << X_ssd << std::endl;
    std::cout << "Y_pio_fk = \n" << Y_pio_fk << std::endl;
    std::cout << "Time taken by function (FK) : " << ((int)duration_fk.count())*0.001 << " microseconds" << std::endl; 
    std::cout << "Time taken by function (Pinocchio FK) : " << ((int)duration_pio_fk.count())*0.001 << " microseconds" << std::endl; 
    std::cout << "Time taken by function (SSD) : " << ((int)duration_ssd.count())*0.001 << " microseconds" << std::endl; 

}