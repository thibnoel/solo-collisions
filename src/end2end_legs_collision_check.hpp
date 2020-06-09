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
Scalar legsCapsulesDistanceCheck(pinocchio::ModelTpl<Scalar> model, 
                      pinocchio::DataTpl<Scalar> data, 
                      Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& config, 
                      int frameInd1, 
                      int frameInd2,
                      Scalar capsLength1,
                      Scalar capsRadius1,
                      Scalar capsLength2,
                      Scalar capsRadius2,
                      pinocchio::SE3Tpl<Scalar> f1Mcaps1,
                      pinocchio::SE3Tpl<Scalar> f2Mcaps2)
{
    // Define capsule directing vector
    Eigen::Matrix<Scalar, 3, 1> capsDirVec1;
    capsDirVec1[0] = 0;
    capsDirVec1[1] = 0;
    capsDirVec1[2] = -1*capsLength1;

    Eigen::Matrix<Scalar, 3, 1> capsDirVec2;
    capsDirVec2[0] = 0;
    capsDirVec2[1] = 0;
    capsDirVec2[2] = -1*capsLength2;

    // Get relative placement between f1, f2
    pinocchio::SE3Tpl<Scalar> f1Mf2 = relativePlacement<Scalar>(model, data, config, frameInd1, frameInd2);

    // Initialize capsule positions
    Eigen::Matrix<Scalar, 3, 1> caps1P0;
    Eigen::Matrix<Scalar, 3, 1> caps1P1;
    Eigen::Matrix<Scalar, 3, 1> caps1P1bis;
    Eigen::Matrix<Scalar, 3, 1> caps2P0;
    Eigen::Matrix<Scalar, 3, 1> caps2P1;
    
    // Compute capsule ends positions
    caps1P0[0] = 0;
    caps1P0[1] = 0;
    caps1P0[2] = 0;
        // Rewrite needed maybe? (order matters here)
    caps1P1 << caps1P0 + capsDirVec1;
    caps1P1bis << caps1P0 + capsDirVec2;

    caps2P0 << f1Mf2.act(f2Mcaps2.act(caps1P0));
    caps2P1 << f1Mf2.act(f2Mcaps2.act(caps1P1bis));
    
    caps1P0 << f1Mcaps1.act(caps1P0);
    caps1P1 << f1Mcaps1.act(caps1P1);  

    std::cout << capsRadius1 << std::endl;
    // Compute min. distance between capsules segments minus capsules radii 
    return CppAD::sqrt(segmentSegmentSqrDistance_scalar<Scalar>(caps1P0[0], caps1P0[1], caps1P0[2],
                                         caps1P1[0], caps1P1[1], caps1P1[2],
                                         caps2P0[0], caps2P0[1], caps2P0[2],
                                         caps2P1[0], caps2P1[1], caps2P1[2])) - (capsRadius1 + capsRadius2);

    
}

// Helper function : returns the jMf SE3 placement given a frame f, j being its parent joint
pinocchio::SE3 jointToFrameRelativePlacement(pinocchio::Model model, pinocchio::Data data, Eigen::Matrix<double, Eigen::Dynamic, 1>& config, int frameInd)
{
    forwardKinematics(model, data, config);
    updateFramePlacements(model, data);
    SE3 oMf = data.oMf[frameInd];
    SE3 oMj = data.oMi[model.frames[frameInd].parent];
    return oMj.inverse() * oMf;
}

// Get the result of the FCL evaluation to test against
double getFCLResult(pinocchio::Model& model, 
                    pinocchio::GeometryModel& gmodel, 
                    pinocchio::Data& data, 
                    Eigen::Matrix<double, Eigen::Dynamic, 1>& config, 
                    std::string frameName1, 
                    std::string frameName2, 
                    double capsLength1,
                    double capsRadius1,
                    double capsLength2,
                    double capsRadius2,
                    pinocchio::SE3 f1Mcaps1,
                    pinocchio::SE3 f2Mcaps2)
{
    typedef boost::shared_ptr< fcl::CollisionGeometry > CollisionGeometryPtr;

    // Get frames indices from the model
    pinocchio::FrameIndex frameInd1 = model.getFrameId(frameName1);
    pinocchio::FrameIndex frameInd2 = model.getFrameId(frameName2);

    // Offset f1Mcaps1 to an end of the capsule
    // TODO : Do the same for caps2 (f2Mcaps as arg)
        // Translations
    Eigen::Matrix<double, 3, 1> capsPosOffset1;
    capsPosOffset1[0] = 0;
    capsPosOffset1[1] = 0;
    capsPosOffset1[2] = -0.5*capsLength1;

    Eigen::Matrix<double, 3, 1> capsPosOffset2;
    capsPosOffset2[0] = 0;
    capsPosOffset2[1] = 0;
    capsPosOffset2[2] = -0.5*capsLength2;
        // Rotation
    Eigen::Quaternion<double> capsRotOffset;
    capsRotOffset.setIdentity();
        // SE3 object

    pinocchio::SE3 capsFrame1(capsRotOffset, capsPosOffset1);
    capsFrame1 = f1Mcaps1.act(capsFrame1);

    pinocchio::SE3 capsFrame2(capsRotOffset, capsPosOffset2);
    capsFrame2 = f2Mcaps2.act(capsFrame2);

    pinocchio::SE3 j1Mframe1 = jointToFrameRelativePlacement(model, data, config, (int)frameInd1);
    pinocchio::SE3 j2Mframe2 = jointToFrameRelativePlacement(model, data, config, (int)frameInd2);

    // Initialize fcl geometries
    const CollisionGeometryPtr caps_geom1 (new hpp::fcl::Capsule(capsRadius1, capsLength1)); 
    const CollisionGeometryPtr caps_geom2 (new hpp::fcl::Capsule(capsRadius2, capsLength2));

    // Initialize geometry objects
        // Capsule 1
    std::string caps1_name = std::string("caps_") + frameName1;
    pinocchio::GeometryObject caps1_gobj(caps1_name,
                                         frameInd1,
                                         model.frames[frameInd1].parent,
                                         caps_geom1, 
                                         j1Mframe1.act(capsFrame1));    
        // Capsule 2
    std::string caps2_name = std::string("caps_") + frameName2;  
    pinocchio::GeometryObject caps2_gobj(caps2_name,
                                         frameInd2,
                                         model.frames[frameInd2].parent,
                                         caps_geom2, 
                                         j2Mframe2.act(capsFrame2));     
    // Add capsules to the model and make them a collision pair
    pinocchio::GeomIndex caps1 = gmodel.addGeometryObject(caps1_gobj, model);
    pinocchio::GeomIndex caps2 = gmodel.addGeometryObject(caps2_gobj, model); 
    gmodel.addCollisionPair(pinocchio::CollisionPair(caps1,caps2));

    // Compute and return the distance result
    pinocchio::GeometryData gdata(gmodel);
    pinocchio::computeDistances(model,data,gmodel,gdata,config);
    std::vector< fcl::DistanceResult > collisions_dist = gdata.distanceResults;              
    return collisions_dist[gmodel.findCollisionPair(pinocchio::CollisionPair(caps1,caps2))].min_distance;                                                                                                                                      
}

// Generates the model for the function f(q, f1, f2) = dist. between legs f1,f2 
ADFun tapeADFunEnd2End(pinocchio::Model model, 
                       int frameInd1, 
                       int frameInd2, 
                       ADScalar capsLength1, 
                       ADScalar capsRadius1, 
                       ADScalar capsLength2, 
                       ADScalar capsRadius2, 
                       pinocchio::SE3Tpl<ADScalar> f1Mcaps1,
                       pinocchio::SE3Tpl<ADScalar> f2Mcaps2 )
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

    pinocchio::forwardKinematics(cast_rmodel, cast_rdata, ad_X);
    pinocchio::updateFramePlacements(cast_rmodel, cast_rdata);

    ADScalar a = legsCapsulesDistanceCheck<ADScalar>(cast_rmodel, cast_rdata, ad_X, frameInd1, frameInd2, capsLength1, capsRadius1, capsLength2, capsRadius2, f1Mcaps1, f2Mcaps2);
    ad_Y[0] = a;
    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}