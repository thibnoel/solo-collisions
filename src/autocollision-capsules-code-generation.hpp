#include <iostream>
//#include <hpp/fcl/collision_object.h>
//#include <hpp/fcl/shape/geometric_shapes.h>

#include "relative-placement-codegen.hpp"
#include "geometric_distances.hpp"
#include "simple-coll-geometries.hpp" // Capsule, sphere and RSS geometries
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
//#include "pinocchio/fcl.hpp"

using namespace pinocchio;

// Returns a pair of int representing two frames, from their names and the robot model
std::pair<int,int> getFramesPair(std::string frameName1, std::string frameName2, pinocchio::Model model)
{
    return std::make_pair((int)model.getFrameId(frameName1),(int)model.getFrameId(frameName2));
}

// Function to generate
// Wrapper for pinocchio::forwardKinematics for framesPair + segmentDistance
template<typename Scalar>
Scalar runCapsulesDistanceCheck(pinocchio::DataTpl<Scalar> data, 
                      std::pair<int,int> framesPair,
                      std::pair<Capsule<Scalar>,Capsule<Scalar>> capsulesPair)
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
    caps1P0 << capsulesPair.first.a;
    caps1P1 << capsulesPair.first.b;  
    caps2P0 << f1Mf2.act(capsulesPair.second.a);
    caps2P1 << f1Mf2.act(capsulesPair.second.b);

    DistanceResult<Scalar> distResult;
    distResult = segmentSegmentSqrDistance_vector<Scalar>(caps1P0, caps1P1, caps2P0, caps2P1);
    // witness points accessible as wPoint1, wPoint2

    return CppAD::sqrt(distResult.distance) - (capsulesPair.first.radius + capsulesPair.second.radius);
}

// Get the jacobian of the distance
// NOT TESTED
template<typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> getFramesDistanceJacobian(pinocchio::ModelTpl<Scalar> model, 
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

// Function to generate
// Wrapper for forward kinematics + pointRectDistance
template<typename Scalar> 
Scalar getSphereRSSDistanceCheck(pinocchio::DataTpl<Scalar> data,
                      std::pair<int,int> framesPair, // first of the pair must be the base link
                      RectSweptSph<Scalar> rss,
                      Sphere<Scalar> sph)
{
    // Get relative placement between f1, f2 (f1 being the base here)
    // Assumes forwardKinematics and updateFramesPlacements have been called on the model
    pinocchio::SE3Tpl<Scalar> bMf2 = getRelativePlacement<Scalar>(data, framesPair);

    pinocchio::SE3Tpl<Scalar> bMr = rss.get_fMc();

    // Declare sphere position in rect. frame
    Eigen::Matrix<Scalar, 3, 1> spherePosR;

    // Initialize sphere position
    spherePosR << bMr.actInv(bMf2.act(sph.c));

    // Compute min. distance between capsules segments minus capsules radii 
    return CppAD::sqrt(pointRectSqrDistance_scalar<Scalar>(spherePosR[0], spherePosR[1], spherePosR[2], rss.width, rss.length)) - (sph.radius + rss.radius);   
}

// Tape 1 pair lower leg - body
ADFun tapeADPointRSSDistanceCheck(pinocchio::ModelTpl<Scalar> model,
                      std::pair<int,int> framesPair, // first of the pair must be the base link
                      RectSweptSph<ADScalar> rss,
                      Sphere<ADScalar> sph)
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

    // Compute forward kinematics 
    pinocchio::forwardKinematics(cast_rmodel, cast_rdata, ad_X);
    pinocchio::updateFramePlacements(cast_rmodel, cast_rdata);

    ADScalar d = getSphereRSSDistanceCheck<ADScalar>(cast_rdata, framesPair, rss, sph);
    ad_Y[0] = d;

    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}

// Tape multiple pairs lower-leg - body but with the same parameters for the RSS and spherre geometries
ADFun tapeADPointRSSMultDistanceCheck(pinocchio::ModelTpl<Scalar> model,
                      std::pair<int,int>* framesPair, // first of the pair must be the base link
                      RectSweptSph<ADScalar> rss,
                      Sphere<ADScalar> sph,
                      int nb_pairs)
{
    // Cast the model to ADScalar type and regenerate the model data
    ModelTpl<ADScalar> cast_rmodel = model.cast<ADScalar>(); 
    DataTpl<ADScalar> cast_rdata(cast_rmodel);  

    // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(cast_rmodel.nq);
    ad_Y.resize(nb_pairs);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;

    // Compute forward kinematics 
    pinocchio::forwardKinematics(cast_rmodel, cast_rdata, ad_X);
    pinocchio::updateFramePlacements(cast_rmodel, cast_rdata);

    for(int k=0; k<nb_pairs; k++)
    {
        ADScalar d = getSphereRSSDistanceCheck<ADScalar>(cast_rdata, framesPair[k], rss, sph);
        ad_Y[k] = d;
    }
    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}

// Generates the model for the function f(q, pair) = dist. between frames of given pair
ADFun tapeADCapsulesDistanceCheck(pinocchio::Model model, 
                       std::pair<int,int>* framesPairs, 
                       std::pair<Capsule<ADScalar>,Capsule<ADScalar>>* capsulesPairs,
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
    ad_Y.resize(nb_pairs);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;

    // Compute forward kinematics 
    pinocchio::forwardKinematics(cast_rmodel, cast_rdata, ad_X);
    pinocchio::updateFramePlacements(cast_rmodel, cast_rdata);

    // Tape the function
    for(int i=0; i<nb_pairs; i++)
    {
        ADScalar d = runCapsulesDistanceCheck<ADScalar>(cast_rdata, framesPairs[i], capsulesPairs[i]);
        ad_Y[i] = d;
    }
    ad_fun.Dependent(ad_X, ad_Y);

    return ad_fun;
}

ADFun tapeADJacobianDistanceCheck(pinocchio::ModelTpl<Scalar> model,
                      std::pair<int,int> framesPair,
                      std::pair<Eigen::Matrix<Scalar, 3, 1>,Eigen::Matrix<Scalar, 3, 1>> witnessPoints)
{
    // Cast the model to ADScalar type and regenerate the model data
    ModelTpl<ADScalar> cast_rmodel = model.cast<ADScalar>(); 
    DataTpl<ADScalar> cast_rdata(cast_rmodel);  

    // Initnialize AD input and output
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_X;
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> ad_Y;
    ad_X.resize(cast_rmodel.nq);
    ad_Y.resize(3*cast_rmodel.nq);
    CppAD::Independent(ad_X);
    // Initialize AD function
    ADFun ad_fun;

    // Compute forward kinematics 
    pinocchio::forwardKinematics(cast_rmodel, cast_rdata, ad_X);
    pinocchio::updateFramePlacements(cast_rmodel, cast_rdata);

    Eigen::Matrix<ADScalar, Eigen::Dynamic, Eigen::Dynamic> res;
    res = getFramesDistanceJacobian<ADScalar>(cast_rmodel, cast_rdata, ad_X, framesPair, witnessPoints);
    res.resize(3*cast_rmodel.nq,1);
    ad_Y = res;

    ad_fun.Dependent(ad_X, ad_Y);
    return ad_fun;

}