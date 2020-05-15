#include <iostream>
#include "end2end_legs_collision_check.hpp"

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
    int hr_lower_leg = (int)rmodel.getFrameId("HR_LOWER_LEG");

    // Generate the code for the specified frames and compile it as library
    ADFun genFun = tapeADFunEnd2End(rmodel, hr_lower_leg, fl_upper_leg);
    generateCompileCLib("end2end",genFun);
    // Print the C code to the console
    std::cout << "// Generated end2end(q) :\n";
    std::cout << generateCSourceCode(genFun, rmodel.nq) << std::endl;

}