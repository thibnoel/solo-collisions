#include <iostream>
#include "relative-placement-codegen.hpp"
#include "segment-segment-distance-codegen.hpp"

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
    int fl_upper_leg = (int)rmodel.getFrameId("FL_UPPER_LEG");
    int base_link = (int)rmodel.getFrameId("base_link");

    // Generate the code for the specified frames and compile it as library
    ADFun relPlacementFun = tapeADFunRelativePlacement(rmodel, base_link, fl_upper_leg);
    generateCompileCLib("rel_placement",relPlacementFun);
    // Print the C code to the console
    std::cout << "// Generated rel_placement(q) :\n";
    std::cout << generateCSourceCode(relPlacementFun, rmodel.nq) << std::endl;

    /*************************** Segment-segment dist. code generation *****************************/
    // Generate segment segment distance code and compile it as library
    ADFun segDistFun = tapeADFunSegSegDist();
    generateCompileCLib("ssd",segDistFun);
    // Print the C code to the console
    std::cout << "// Generated seg_seg_dist(seg1, seg2) :\n";
    std::cout << generateCSourceCode(segDistFun, 12) << std::endl;
}