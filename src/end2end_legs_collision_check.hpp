#include <iostream>
#include "relative-placement-codegen.hpp"
#include "segment-segment-distance-codegen.hpp"

using namespace pinocchio;

// Function to generate
// Wrapper for pinocchio::forwardKinematics for frames f1, f2 + segmentDistance
template<typename Scalar>
Scalar end2endWrapper(pinocchio::ModelTpl<Scalar> model, pinocchio::DataTpl<Scalar> data, Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& config, int frameInd1, int frameInd2,Scalar capsLength,Scalar capsRadius,pinocchio::SE3Tpl<Scalar> f1Mcaps)
{
    //Capsules parameters (hardcoded):
    // TODO : place and orient the capsule : SE3Tpl<Scalar> f1Mcaps;
    /*Scalar capsRadius;
    capsRadius = 0.02;
    Scalar capsLength;
    capsLength = 0.2;
    */
    Eigen::Matrix<Scalar, 4, 1> capsDirVec;
    capsDirVec[0] = 0;
    capsDirVec[1] = 0;
    capsDirVec[2] = capsLength;
    capsDirVec[3] = 1;

    // Get and resize relative placement between f1, f2
    pinocchio::SE3Tpl<Scalar> f1Mf2 = relativePlacement<Scalar>(model, data, config, frameInd1, frameInd2);
    
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> M = Eigen::Map<const Eigen::Matrix<Scalar, 16, 1> >(f1Mf2.toHomogeneousMatrix().data(), f1Mf2.toHomogeneousMatrix().size());
    M.resize(4,4);

    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Mcaps = Eigen::Map<const Eigen::Matrix<Scalar, 16, 1> >(f1Mcaps.toHomogeneousMatrix().data(), f1Mcaps.toHomogeneousMatrix().size());
    Mcaps.resize(4,4);

    // Initialize capsule positions
    Eigen::Matrix<Scalar, 4, 1> caps0Pos0;
    Eigen::Matrix<Scalar, 4, 1> caps0Pos1;
    Eigen::Matrix<Scalar, 4, 1> caps1Pos0;
    Eigen::Matrix<Scalar, 4, 1> caps1Pos1;
    
    // Update capsule positions
    caps0Pos0[0] = 0;
    caps0Pos0[1] = 0;
    caps0Pos0[2] = 0;
    caps0Pos0[3] = 1;

    caps0Pos0 << Mcaps*caps0Pos0;
    caps0Pos1 << caps0Pos0 + Mcaps*capsDirVec;
    caps1Pos0 << M*caps0Pos0;
    caps1Pos1 << M*caps0Pos1;

    // Compute min. distance between capsules segments minus capsules radii 
    return CppAD::sqrt(segmentSegmentSqrDistance_scalar<Scalar>(caps0Pos0[0], caps0Pos0[1], caps0Pos0[2],
                                         caps0Pos1[0], caps0Pos1[1], caps0Pos1[2],
                                         caps1Pos0[0], caps1Pos0[1], caps1Pos0[2],
                                         caps1Pos1[0], caps1Pos1[1], caps1Pos1[2])) - 2*capsRadius;

    
}

// Generates the model for the function f(q, f1, f2) = dist. between legs f1,f2 
ADFun tapeADFunEnd2End(pinocchio::Model model, int frameInd1, int frameInd2, ADScalar capsLength, ADScalar capsRadius, pinocchio::SE3Tpl<ADScalar> f1Mcaps)
{
    // Cast the model to ADScalar type and regenerate the model data
    ModelTpl<ADScalar> cast_rmodel = model.cast<ADScalar>(); 
    DataTpl<ADScalar> cast_rdata(cast_rmodel);  

    // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(cast_rmodel.nq);
    ad_Y.resize(1);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;

    ADScalar a = end2endWrapper<ADScalar>(cast_rmodel, cast_rdata, ad_X, frameInd1, frameInd2, capsLength, capsRadius, f1Mcaps);
    ad_Y[0] = a;
    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}