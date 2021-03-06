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
    computeFK(model, data, config);
    pinocchio::SE3Tpl<Scalar> f1Mf2 = getRelativePlacement<Scalar>(data, std::make_pair(frameInd1, frameInd2));

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

template<typename Scalar>
Scalar legsCapsulesDistanceCheckNoFK(pinocchio::DataTpl<Scalar> data,
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
    //computeFK(model, data, config);
    pinocchio::SE3Tpl<Scalar> f1Mf2 = getRelativePlacement<Scalar>(data, std::make_pair(frameInd1, frameInd2));

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

template<typename Scalar>
Eigen::Matrix<Scalar, 4, 1> legToLegDistanceCheck(pinocchio::ModelTpl<Scalar> model, 
                                                    pinocchio::DataTpl<Scalar> rdata,
                                                    std::string leg1, 
                                                    std::string leg2, 
                                                    Scalar upperCapsLength, 
                                                    Scalar upperCapsRadius, 
                                                    Scalar lowerCapsLength, 
                                                    Scalar lowerCapsRadius, 
                                                    pinocchio::SE3Tpl<Scalar> f1Mcaps1,
                                                    pinocchio::SE3Tpl<Scalar> f2Mcaps2 )
{
    const int frame1Up = (int)model.getFrameId(leg1 + "_UPPER_LEG");
    const int frame1Low = (int)model.getFrameId(leg1 + "_LOWER_LEG");
    const int frame2Up = (int)model.getFrameId(leg2 + "_UPPER_LEG");
    const int frame2Low = (int)model.getFrameId(leg2 + "_LOWER_LEG");   

    Scalar d1u2u = legsCapsulesDistanceCheckNoFK<Scalar>(rdata, frame1Up, frame2Up, upperCapsLength, upperCapsRadius, upperCapsLength, upperCapsRadius, f1Mcaps1, f2Mcaps2);
    Scalar d1u2l = legsCapsulesDistanceCheckNoFK<Scalar>(rdata, frame1Up, frame2Low, upperCapsLength, upperCapsRadius, lowerCapsLength, lowerCapsRadius, f1Mcaps1, f2Mcaps2);
    Scalar d1l2u = legsCapsulesDistanceCheckNoFK<Scalar>(rdata, frame1Low, frame2Up, lowerCapsLength, lowerCapsRadius, upperCapsLength, upperCapsRadius, f1Mcaps1, f2Mcaps2);
    Scalar d1l2l = legsCapsulesDistanceCheckNoFK<Scalar>(rdata, frame1Low, frame2Low, lowerCapsLength, lowerCapsRadius, lowerCapsLength, lowerCapsRadius, f1Mcaps1, f2Mcaps2);   

    Eigen::Matrix<Scalar, 4, 1> distVec;
    distVec << d1u2u, d1u2l, d1l2u, d1l2l;

    return distVec;                                           
}

// MOVED TO autocollision-code-generation.hpp
/*
std::pair<int,int> getFramesPair(std::string frameName1, std::string frameName2, pinocchio::Model model)
{
    return std::make_pair((int)model.getFrameId(frameName1),(int)model.getFrameId(frameName2));
}

std::pair<int,int>* getLegToLegPairs(std::string leg1, std::string leg2, pinocchio::Model model)
{
    static std::pair<int,int> pairs[4] = {getFramesPair(leg1 + "_UPPER_LEG", leg2 + "_UPPER_LEG", model),
                                   getFramesPair(leg1 + "_UPPER_LEG", leg2 + "_LOWER_LEG", model),
                                   getFramesPair(leg1 + "_LOWER_LEG", leg2 + "_UPPER_LEG", model),
                                   getFramesPair(leg1 + "_LOWER_LEG", leg2 + "_LOWER_LEG", model)};

    return pairs;
}
*/


// Returns the distances between all leg segments as a 8x8 matrix (symetric with null diagonal -> 24 coeffs determin the matrix)
// Matrix Shape : 
//      FLU     FLL     FRU     FRL     HLU     HLL     HRU     HRL     
//  FLU  x       x   dFlFr[0] dFlFr[2] dFlHl[0] ...     ...
//
//  FLL  x       x   dFlFr[1] dFlFr[3]  ...
//
//  FRU                  x       x      ...
//
//  FRL                  x       x
//
//  HLU
//
//  HLL
//  
//  HRU
//  
//  HRL
template<typename Scalar>
Eigen::Matrix<Scalar, 64, 1> allLegsDistanceCheck(pinocchio::ModelTpl<Scalar> model, 
                                                    pinocchio::DataTpl<Scalar> rdata, 
                                                    Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& config,
                                                    Scalar upperCapsLength, 
                                                    Scalar upperCapsRadius, 
                                                    Scalar lowerCapsLength, 
                                                    Scalar lowerCapsRadius, 
                                                    pinocchio::SE3Tpl<Scalar> f1Mcaps1,
                                                    pinocchio::SE3Tpl<Scalar> f2Mcaps2,
                                                    int npairs)
{
    std::string legs[4] = {"FL","FR","HL","HR"};
    Eigen::Matrix<Scalar, 8, 8> distMatrix;
    Eigen::Matrix<Scalar, 2, 2> distSubMatrices[6];
    Eigen::Matrix<Scalar,2,2> zero22 = Eigen::Matrix<Scalar,2,2>::Zero(2,2);

    computeFK(model, rdata, config);

    // TO DO : adjust f1Mcaps1, f2Mcaps2 (upper and lower caps poses in leg segments frames) to account for legs symetries
    // TO DO : check the placements with FCL

    int counter = 0;

    for(int i=0; i<4; i++)
    {
        for(int j=i+1; j<4; j++)
        {
            Eigen::Matrix<Scalar, 2, 2> legToLegMatrix;

            if(counter < npairs)
            {
                std::string leg1 = legs[i];
                std::string leg2 = legs[j];

                Eigen::Matrix<Scalar, 4, 1> legToLegResult = legToLegDistanceCheck<Scalar>(model, rdata, leg1, leg2, upperCapsLength, upperCapsRadius, lowerCapsLength, lowerCapsRadius, f1Mcaps1, f2Mcaps2);
                legToLegMatrix = Eigen::Map<const Eigen::Matrix<Scalar, 2, 2> >(legToLegResult.data());
            }
            else
            {
                legToLegMatrix = zero22;
            }
            int ind;
            if(i == 0){
                ind = i+j-1;
            } else {ind = i+j;} 
            distSubMatrices[ind] = legToLegMatrix;

            counter += 1;
        }
    }

    distMatrix << zero22, distSubMatrices[0], distSubMatrices[1], distSubMatrices[2],
                  distSubMatrices[0].transpose(), zero22, distSubMatrices[3], distSubMatrices[4],
                  distSubMatrices[1].transpose(), distSubMatrices[3].transpose(), zero22, distSubMatrices[5],
                  distSubMatrices[2].transpose(), distSubMatrices[4].transpose(), distSubMatrices[5].transpose(), zero22;

    return Eigen::Map<const Eigen::Matrix<Scalar, 64, 1> >(distMatrix.data());

}

// Helper function : returns the jMf SE3 placement given a frame f, j being its parent joint
pinocchio::SE3 jointToFrameRelativePlacement(pinocchio::Model model, pinocchio::Data data, int frameInd)
{
    //forwardKinematics(model, data, config);
    //updateFramePlacements(model, data);
    SE3 oMf = data.oMf[frameInd];
    SE3 oMj = data.oMi[model.frames[frameInd].parent];
    return oMj.inverse() * oMf;
}

// Get the result of the FCL evaluation to test against
double getPairFCLResult(pinocchio::Model& model, 
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

    pinocchio::SE3 j1Mframe1 = jointToFrameRelativePlacement(model, data, (int)frameInd1);
    pinocchio::SE3 j2Mframe2 = jointToFrameRelativePlacement(model, data, (int)frameInd2);

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

// Get the result of the FCL evaluation to test against
/*
Eigen::Matrix<double, 8, 8> getAllLegsFCLResult(pinocchio::Model& model, 
                    pinocchio::GeometryModel& gmodel, 
                    pinocchio::Data& data, 
                    Eigen::Matrix<double, Eigen::Dynamic, 1>& config, 
                    double upperCapsLength,
                    double upperCapsRadius,
                    double lowerCapsLength,
                    double lowerC apsRadius,
                    pinocchio::SE3 f1Mcaps1,
                    pinocchio::SE3 f2Mcaps2)
{

}
*/

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

// Generates the model for the function f(q, l1, l2) = dist.vec. between legs 1,2 
// Result is given as a vector of distances between the pairs, in the following order :
//      [(frame1Up, frame2Up)
//       (frame1Up, frame2Low)
//       (frame2Up, frame1Low)
//       (frame1Low, frame2Low)]
ADFun tapeADFunFullLeg(pinocchio::Model model, 
                       std::string leg1, 
                       std::string leg2, 
                       ADScalar upperCapsLength, 
                       ADScalar upperCapsRadius, 
                       ADScalar lowerCapsLength, 
                       ADScalar lowerCapsRadius, 
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
    ad_Y.resize(4);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;

    pinocchio::forwardKinematics(cast_rmodel, cast_rdata, ad_X);
    pinocchio::updateFramePlacements(cast_rmodel, cast_rdata);

    ad_Y = legToLegDistanceCheck<ADScalar>(cast_rmodel, cast_rdata, leg1, leg2, upperCapsLength, upperCapsRadius, lowerCapsLength, lowerCapsRadius, f1Mcaps1, f2Mcaps2);
    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}

ADFun tapeADFunAllLegs(pinocchio::Model model, 
                       ADScalar upperCapsLength, 
                       ADScalar upperCapsRadius, 
                       ADScalar lowerCapsLength, 
                       ADScalar lowerCapsRadius, 
                       pinocchio::SE3Tpl<ADScalar> f1Mcaps1,
                       pinocchio::SE3Tpl<ADScalar> f2Mcaps2,
                       int npairs )
{
    
    // Cast the model to ADScalar type and regenerate the model data
    ModelTpl<ADScalar> cast_rmodel = model.cast<ADScalar>(); 
    DataTpl<ADScalar> cast_rdata(cast_rmodel);  

    // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(cast_rmodel.nq);
    ad_Y.resize(64);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;

    pinocchio::forwardKinematics(cast_rmodel, cast_rdata, ad_X);
    pinocchio::updateFramePlacements(cast_rmodel, cast_rdata);

    ad_Y = allLegsDistanceCheck<ADScalar>(cast_rmodel, cast_rdata, ad_X, upperCapsLength, upperCapsRadius, lowerCapsLength, lowerCapsRadius, f1Mcaps1, f2Mcaps2, npairs);
    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}