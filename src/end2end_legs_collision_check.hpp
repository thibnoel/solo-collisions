#include <iostream>
//#include <hpp/fcl/collision_object.h>
//#include <hpp/fcl/shape/geometric_shapes.h>

#include "relative-placement-codegen.hpp"
#include "segment-segment-distance-codegen.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
//#include "pinocchio/fcl.hpp"

using namespace pinocchio;

// Function to generate
// Wrapper for pinocchio::forwardKinematics for frames f1, f2 + segmentDistance
template<typename Scalar>
Scalar end2endWrapper(pinocchio::ModelTpl<Scalar> model, 
                      pinocchio::DataTpl<Scalar> data, 
                      Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& config, 
                      int frameInd1, 
                      int frameInd2,
                      Scalar capsLength,
                      Scalar capsRadius,
                      pinocchio::SE3Tpl<Scalar> f1Mcaps)
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

// Get the result of the FCL evaluation to test against
double getFCLResult(pinocchio::Model& model, 
                    pinocchio::GeometryModel& gmodel, 
                    pinocchio::Data& data, 
                    Eigen::Matrix<double, Eigen::Dynamic, 1>& config, 
                    std::string frameName1, 
                    std::string frameName2, 
                    double capsLength,
                    double capsRadius)
                    //pinocchio::SE3& f1Mcaps)
{
    typedef boost::shared_ptr< fcl::CollisionGeometry > CollisionGeometryPtr;

    // Get frames indices from the model
    pinocchio::FrameIndex frameInd1 = model.getFrameId(frameName1);
    pinocchio::FrameIndex frameInd2 = model.getFrameId(frameName2);

    // Offset f1Mcaps to an end of the capsule
    // Do the same for caps2
        // Translation
    Eigen::Matrix<double, 3, 1> capsPosOffset;
    capsPosOffset[0] = 0;
    capsPosOffset[1] = 0;
    capsPosOffset[2] = -0.5*capsLength;
        // Rotation
    Eigen::Quaternion<double> capsRotOffset;
    capsRotOffset.setIdentity();
        // SE3 object
    pinocchio::SE3 capsFrame(capsRotOffset, capsPosOffset);
    
    //f1Mcaps << f1Mcaps * capsFrame;

    const CollisionGeometryPtr caps_geom1 (new hpp::fcl::Capsule(capsRadius, 0.5*capsLength)); // WARNING : FCL takes the capsule halfLength !
    const CollisionGeometryPtr caps_geom2 (new hpp::fcl::Capsule(capsRadius, 0.5*capsLength)); // WARNING : FCL takes the capsule halfLength !

    std::string caps1_name = std::string("caps_") + frameName1;
    pinocchio::GeometryObject caps1_gobj(caps1_name,
                                         frameInd1,
                                         model.frames[frameInd1].parent,
                                         caps_geom1, 
                                         capsFrame);
    pinocchio::GeomIndex caps1 = gmodel.addGeometryObject(caps1_gobj, model);

    std::string caps2_name = std::string("caps_") + frameName2;  
    pinocchio::GeometryObject caps2_gobj(caps2_name,
                                         frameInd2,
                                         model.frames[frameInd2].parent,
                                         caps_geom2, 
                                         capsFrame);
    pinocchio::GeomIndex caps2 = gmodel.addGeometryObject(caps2_gobj, model);  

    gmodel.addCollisionPair(pinocchio::CollisionPair(caps1,caps2));
    
    pinocchio::GeometryData gdata(gmodel);
    pinocchio::computeDistances(model,data,gmodel,gdata,config);
    std::vector< fcl::DistanceResult > collisions_dist = gdata.distanceResults;              
    return collisions_dist[gmodel.findCollisionPair(pinocchio::CollisionPair(caps1,caps2))].min_distance;                                                                                                                                      
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