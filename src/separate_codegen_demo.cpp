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
    int hr_lower_leg = (int)rmodel.getFrameId("HR_LOWER_LEG");
    int base_link = (int)rmodel.getFrameId("base_link");

    // Generate the code for the specified frames and compile it as library
    ADFun relPlacementFun1 = tapeADFunRelativePlacement(rmodel, base_link, fl_upper_leg);
    generateCompileCLib("rel_placement",relPlacementFun1);
    // Print the C code to the console
    std::cout << "// Generated rel_placement(q) for frames BASE_LINK and FL_UPPER_LEG :\n";
    std::cout << generateCSourceCode(relPlacementFun1, rmodel.nq) << std::endl;

    // One more example, with another collision pair
    ADFun relPlacementFun2 = tapeADFunRelativePlacement(rmodel, hr_lower_leg, fl_upper_leg);
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
}