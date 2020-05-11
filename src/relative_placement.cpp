#include <iosfwd>
//#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <iostream>

using namespace pinocchio;

template<typename Scalar>
SE3 relativePlacement(ModelTpl<Scalar> model, DataTpl<Scalar> data, Eigen::VectorXd config, int frameInd1, int frameInd2)
{
    forwardKinematics(model, data, config);
    updateFramePlacements(model, data);
    SE3 oMf1 = data.oMf[frameInd1];
    SE3 oMf2 = data.oMf[frameInd2];
    return oMf1.inverse() * oMf2;
}

int main()
{  
    /*********************************************************************
     * Initialisation
    *********************************************************************/
    // setting the Scalar template to represent doubles
    typedef double Scalar;

    // Setup URDF path
    const std::string urdf_filename = "/opt/openrobots/share/example-robot-data/robots/solo_description/robots/solo12.urdf";

    // Load URDF model
    Model rmodel;
    pinocchio::urdf::buildModel(urdf_filename,rmodel);
    Data rdata(rmodel);    
    Eigen::VectorXd config = Eigen::VectorXd::Zero(rmodel.nq);
    config[7] += 1.6; // Updating config 

    // Get the robot frames we're interested in
    int fl_upper_leg = rmodel.getFrameId("FL_UPPER_LEG");
    /*fl_lower_leg = rmodel.getFrameId("FL_LOWER_LEG")
    fr_upper_leg = rmodel.getFrameId("FR_UPPER_LEG")
    fr_lower_leg = rmodel.getFrameId("FR_LOWER_LEG")
    hl_upper_leg = rmodel.getFrameId("HL_UPPER_LEG")
    hl_lower_leg = rmodel.getFrameId("HL_LOWER_LEG")
    hr_upper_leg = rmodel.getFrameId("HR_UPPER_LEG")
    hr_lower_leg = rmodel.getFrameId("HR_LOWER_LEG")*/
    int base_link = rmodel.getFrameId("base_link");
   
    std::cout << "New config (eigen):" << std::endl;
    std::cout << config << std::endl;
    std::cout << "Frames considered : " << fl_upper_leg << ", " << base_link << std::endl;

    SE3 baseMflul = relativePlacement(rmodel, rdata, config, base_link, fl_upper_leg);
    std::cout << "Rel. placement :\n" << baseMflul << std::endl;
}