#include <iostream>
#include <utility>  
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"

#include "geometric_distances.hpp"    // Seg-Seg and Rect-Point distances

using namespace pinocchio;
typedef boost::shared_ptr< fcl::CollisionGeometry > CollisionGeometryPtr;


// Returns a pair of int representing two frames, from their names and the robot model
std::pair<int,int> getFramesPair(std::string frameName1, std::string frameName2, pinocchio::Model model)
{
    return std::make_pair((int)model.getFrameId(frameName1),(int)model.getFrameId(frameName2));
}


// Returns a pair of int representing two geometries, from their names and the robot geometry model
std::pair<int, int> getGeomPair(std::string geomName1, std::string geomName2, pinocchio::GeometryModel gmodel)
{
    return std::make_pair((int)gmodel.getGeometryId(geomName1),(int)gmodel.getGeometryId(geomName2));
}


// Returns the relative placement of frames 1,2 from the updated robot model
template<typename Scalar>
pinocchio::SE3Tpl<Scalar> getRelativePlacement(pinocchio::DataTpl<Scalar> updatedData, std::pair<int, int> framesPair)
{
    SE3Tpl<Scalar> oMf1 = updatedData.oMf[framesPair.first];
    SE3Tpl<Scalar> oMf2 = updatedData.oMf[framesPair.second];
    return oMf1.inverse() * oMf2;
}


// Compute the distance and witness points for a pair of FCL capsules geometries, 
// given their parent frames and local placements 
template<typename Scalar>
DistanceResult<Scalar> computeCapsulesCollisionDistance(pinocchio::DataTpl<Scalar> data, 
                      std::pair<int,int> framesPair,
                      std::pair<boost::shared_ptr< fcl::Capsule >, boost::shared_ptr< fcl::Capsule>> fclCapsPair,
                      std::pair<SE3Tpl<Scalar>, SE3Tpl<Scalar>> capsPlacements)
{
    // Get relative placement between f1, f2
    // Assumes forwardKinematics and updateFramesPlacements have been called on the model
    pinocchio::SE3Tpl<Scalar> f1Mf2 = getRelativePlacement<Scalar>(data, framesPair);   

    // Declare capsules ends positions
    Eigen::Matrix<Scalar, 3, 1> caps1P0;
    Eigen::Matrix<Scalar, 3, 1> caps1P1;
    Eigen::Matrix<Scalar, 3, 1> caps2P0;
    Eigen::Matrix<Scalar, 3, 1> caps2P1;

    // Initialize capsules ends positions
    caps1P0 << (Scalar)0, (Scalar)0, (Scalar)fclCapsPair.first->halfLength;
    caps1P1 << (Scalar)0, (Scalar)0, (Scalar)-fclCapsPair.first->halfLength;

    caps1P0 << capsPlacements.first.act(caps1P0);
    caps1P1 << capsPlacements.first.act(caps1P1);

    caps2P0 << (Scalar)0, (Scalar)0, (Scalar)fclCapsPair.second->halfLength;
    caps2P1 << (Scalar)0, (Scalar)0, (Scalar)-fclCapsPair.second->halfLength;

    caps2P0 << capsPlacements.second.act(caps2P0);
    caps2P1 << capsPlacements.second.act(caps2P1);

    caps2P0 << f1Mf2.act(caps2P0);
    caps2P1 << f1Mf2.act(caps2P1);

    DistanceResult<Scalar> distResult;
    distResult = segmentSegmentSqrDistance_vector<Scalar>(caps1P0, caps1P1, caps2P0, caps2P1);
    // witness points accessible as wPoint1, wPoint2

    return distResult;
}


// Compute the jacobian J_Dp = d(Dp)/dq
// for a pair of witness points p0, p1 expressed in their local frames,
// given those frames
template<typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> computeCapsulesCollisionJacobian(pinocchio::ModelTpl<Scalar> model, 
                      pinocchio::DataTpl<Scalar> data, 
                      Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& config, 
                      std::pair<int,int> framesPair,
                      std::pair<Eigen::Matrix<Scalar, 3, 1>,Eigen::Matrix<Scalar, 3, 1>> witnessPoints)
{
    // Declare intermediary variables
    Eigen::Matrix<ADScalar, Eigen::Dynamic, Eigen::Dynamic> distJac;
    Eigen::Matrix<ADScalar, 6, 12> Jf1, Jf2;
    Eigen::Matrix<ADScalar, 3, 12> f1Jp1, f2Jp2;
    Eigen::Matrix<ADScalar, 3, 12> oJp1, oJp2;

    int base_frame_id = (int)model.getFrameId("base_link"); 
    // Compute FK
    pinocchio::SE3Tpl<Scalar> oMf1 = getRelativePlacement<Scalar>(data, std::make_pair(base_frame_id, framesPair.first));
    pinocchio::SE3Tpl<Scalar> oMf2 = getRelativePlacement<Scalar>(data, std::make_pair(base_frame_id, framesPair.second));
    // Compute frames jacobians
    Jf1 = Eigen::Matrix<ADScalar, 6, 12>::Zero(6,12);
    Jf2 = Eigen::Matrix<ADScalar, 6, 12>::Zero(6,12);
    pinocchio::computeFrameJacobian(model, data, config, framesPair.first, LOCAL, Jf1);
    pinocchio::computeFrameJacobian(model, data, config, framesPair.second, LOCAL, Jf2);
    // Compute local velocities
    f1Jp1 = Jf1.block(0,0,3,Jf1.cols()) + pinocchio::skew(witnessPoints.first)*Jf1.block(3,0,3,Jf1.cols()); 
    f2Jp2 = Jf2.block(0,0,3,Jf2.cols()) + pinocchio::skew(witnessPoints.second)*Jf2.block(3,0,3,Jf2.cols()); 
    // Compute world velocities
    oJp1 = oMf1.rotation()*f1Jp1;
    oJp2 = oMf2.rotation()*f2Jp2;

    distJac.resize(3,12);
    distJac = oJp2 - oJp1;
    return distJac;
}


// Generates the model for the collision distances and their jacobians,
// for a list of capsules geometries given their respective parent frames 
// and the robot models.
// REMARK: DIMENSIONS!!
// The jacobian for 1 pair has a dimension 3x12, which leads to the model having an output of size : n_pairs*(1 + 3*12)
ADFun tapeADCapsulesCollisionComputation(pinocchio::Model model, 
                       pinocchio::GeometryModel gmodel, 
                       std::pair<int,int>* framesPairs, 
                       std::pair<int,int>* geomPairs,
                       int nb_pairs)
{
    std::cout << "Nb pairs : " << nb_pairs  << std::endl;
    // Cast the model to ADScalar type and regenerate the model data
    ModelTpl<ADScalar> cast_rmodel = model.cast<ADScalar>(); 
    DataTpl<ADScalar> cast_rdata(cast_rmodel);

    // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(cast_rmodel.nq);
    ad_Y.resize(nb_pairs*(1+3*cast_rmodel.nq));
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;

    // Compute forward kinematics 
    pinocchio::forwardKinematics(cast_rmodel, cast_rdata, ad_X);
    pinocchio::updateFramePlacements(cast_rmodel, cast_rdata);    

    // Tape the function
    for(int i=0; i<nb_pairs; i++)
    {
        int ind_offset = (1+3*cast_rmodel.nq);

        // Get the geometries' CollisionGeometryPtr, casted as fcl::Capsule
        boost::shared_ptr< fcl::Capsule > caps1 = boost::dynamic_pointer_cast< fcl::Capsule >( gmodel.geometryObjects[geomPairs[i].first].geometry );
        boost::shared_ptr< fcl::Capsule > caps2 = boost::dynamic_pointer_cast< fcl::Capsule >( gmodel.geometryObjects[geomPairs[i].second].geometry );

        // Get the geometries placements in their parent frames
        pinocchio::SE3Tpl<ADScalar> c1Mf1 = gmodel.geometryObjects[geomPairs[i].first].placement.cast<ADScalar>();
        pinocchio::SE3Tpl<ADScalar> c2Mf2 = gmodel.geometryObjects[geomPairs[i].second].placement.cast<ADScalar>();
        
        // Make the pairs arguments for code gen.
        std::pair<boost::shared_ptr< fcl::Capsule > ,boost::shared_ptr< fcl::Capsule > > capsPair = std::make_pair(caps1, caps2);
        std::pair<SE3Tpl<ADScalar>, SE3Tpl<ADScalar>> placementsPair = std::make_pair(c1Mf1, c2Mf2);

        // Tape collision distance operations
        DistanceResult<ADScalar> distRes = computeCapsulesCollisionDistance<ADScalar>(cast_rdata, framesPairs[i], capsPair, placementsPair);
        ad_Y[ind_offset*i] = CppAD::sqrt(distRes.distance) - (caps1->radius + caps2->radius);

        std::pair<Eigen::Matrix<ADScalar, 3, 1>, Eigen::Matrix<ADScalar, 3, 1>> witnessPoints = std::make_pair(distRes.wPoint1, distRes.wPoint2);
        // Tape collision jacobian operations
        Eigen::Matrix<ADScalar, Eigen::Dynamic, Eigen::Dynamic> jac;
        jac = computeCapsulesCollisionJacobian<ADScalar>(cast_rmodel, cast_rdata, ad_X, framesPairs[i], witnessPoints);
        jac.resize(3*cast_rmodel.nq,1);
        ad_Y.block(ind_offset*i + 1, 0, 3*cast_rmodel.nq, 1) = jac;
    }

    ad_fun.Dependent(ad_X, ad_Y); 
    return ad_fun;;
}