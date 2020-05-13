#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
//#include <pinocchio/codegen/cppadcg.hpp>
#include "codegen_helper.hpp"

using namespace pinocchio;

// Function to generate
// Wrapper for pinocchio::forwardKinematics for frames f1, f2
template<typename Scalar>
pinocchio::SE3Tpl<Scalar> relativePlacement(pinocchio::ModelTpl<Scalar> model, pinocchio::DataTpl<Scalar> data, Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& config, int frameInd1, int frameInd2)
{
    forwardKinematics(model, data, config);
    updateFramePlacements(model, data);
    SE3Tpl<Scalar> oMf1 = data.oMf[frameInd1];
    SE3Tpl<Scalar> oMf2 = data.oMf[frameInd2];
    return oMf1.inverse() * oMf2;
}

// Generates the function f12(q) = f1Mf2 
std::string generateCSourceRelativePlacement(pinocchio::Model model, int frameInd1, int frameInd2)
{
    // Cast the model to ADScalar type and regenerate the model data
    ModelTpl<ADScalar> cast_rmodel = model.cast<ADScalar>(); 
    DataTpl<ADScalar> cast_rdata(cast_rmodel);  

    //*********Setting up code gen variables and function*********
        // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(cast_rmodel.nq);
    ad_Y.resize(16);
    CppAD::Independent(ad_X);
        // Initialize AD function
    ADFun ad_fun;
    
        // Record tape
    SE3Tpl<ADScalar> result = relativePlacement<ADScalar>(cast_rmodel, cast_rdata, ad_X, frameInd1, frameInd2);
    ad_Y = Eigen::Map<const Eigen::Matrix<ADScalar, 16, 1> >(result.toHomogeneousMatrix().data(), result.toHomogeneousMatrix().size());
    ad_fun.Dependent(ad_X, ad_Y);

    // Use helper function from codegen_helper.hpp
    std::string code = generateCSourceCode(ad_fun, cast_rmodel.nq);
    return code;
}