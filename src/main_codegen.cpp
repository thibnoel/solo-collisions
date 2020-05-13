#include <iostream>
#include "relative-placement-codegen.hpp"
#include "segment-segment-distance-codegen.hpp"

using namespace pinocchio;

// For now, this main only generates the relative placement code as a string and prints it to the console
int main()
{
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

    // Generate the code for the specified frames
    std::string relPlacementGeneratedCode = generateCSourceRelativePlacement(rmodel, base_link, fl_upper_leg);

    // Generate segment segment distance code
    std::string segDistGeneratedCode = generateCSourceSegSegDist();

    // Print the C code to the console
    std::cout << "// Generated rel_placement(q) :\n";
    std::cout << relPlacementGeneratedCode << std::endl;

    std::cout << "// Generated seg_seg_dist(seg1, seg2) :\n";
    std::cout << segDistGeneratedCode << std::endl;
}