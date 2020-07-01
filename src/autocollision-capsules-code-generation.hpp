#include <iostream>
//#include <hpp/fcl/collision_object.h>
//#include <hpp/fcl/shape/geometric_shapes.h>

#include "relative-placement-codegen.hpp"
#include "segment-segment-distance-codegen.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
//#include "pinocchio/fcl.hpp"

using namespace pinocchio;

// Capsule struct used to simplify arguments 
template<typename Scalar>
struct Capsule 
{
    // Ends coord. in reference frame
    Eigen::Matrix<Scalar, 3, 1> a;
    Eigen::Matrix<Scalar, 3, 1> b;
    // Radius
    Scalar radius;
    
    // Length
    Scalar getLength()
    {
        return (b-a).norm();
    }
    // Transform from parent to ref. frame
    pinocchio::SE3Tpl<Scalar> get_fMc()
    {   
        Scalar zero;
        zero = 0;
        Eigen::Matrix<Scalar, 3, 1> translation = 0.5*(a + b);
        Eigen::Matrix<Scalar, 3, 3> rotation;
        Eigen::Matrix<Scalar, 3, 1> capsDir = (b-a)/((b-a).norm());
        Eigen::Matrix<Scalar, 3, 1> upDir;
        upDir[0] = 0;
        upDir[1] = 0;
        upDir[2] = 1;
        Eigen::Matrix<Scalar, 3, 1> v;
        v = upDir.cross(capsDir);
        Scalar c = upDir.transpose()*capsDir;
        Scalar s = v.norm();

        // Check parallel case
        if(s > 1e-5)
        {
            Eigen::Matrix<Scalar, 3, 3> kmat;
            kmat << zero, -v[2], v[1],
                    v[2], zero, -v[0],
                    -v[1], v[0], zero;
            rotation = Eigen::Matrix<Scalar, 3, 3>::Identity(3,3) + kmat + ((1-c)/(s*s))*(kmat*kmat);            
        } else {
            rotation = Eigen::Matrix<Scalar, 3, 3>::Identity(3,3);
        }

        const pinocchio::SE3Tpl<Scalar> fMc(rotation, translation);

        return fMc;
    }

    template<typename AltScalar>
    Capsule<AltScalar> cast()
    {
        Eigen::Matrix<AltScalar, 3, 1> a_cast;
        a_cast[0] = a[0];
        a_cast[1] = a[1];
        a_cast[2] = a[2];
        Eigen::Matrix<AltScalar, 3, 1> b_cast;
        b_cast[0] = b[0];
        b_cast[1] = b[1];
        b_cast[2] = b[2];
        AltScalar radius_cast;
        radius_cast = radius;
        //pinocchio::SE3Tpl<AltScalar> fMc_cast(fMc.rotation(), fMc.translation());
        const struct Capsule<AltScalar> caps_cast = {a_cast, b_cast, radius_cast};

        return caps_cast;
    }
};

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

    // Compute min. distance between capsules segments minus capsules radii 
    return CppAD::sqrt(segmentSegmentSqrDistance_scalar<Scalar>(caps1P0[0], caps1P0[1], caps1P0[2],
                                         caps1P1[0], caps1P1[1], caps1P1[2],
                                         caps2P0[0], caps2P0[1], caps2P0[2],
                                         caps2P1[0], caps2P1[1], caps2P1[2])) - (capsulesPair.first.radius + capsulesPair.second.radius);
}

// Get the jacobian of the distance
template<typename Scalar>
Eigen::Matrix<Scalar, Eigen::Dynamic, 1> getCapsulesDistanceJacobian(pinocchio::ModelTpl<Scalar> model, 
                      pinocchio::DataTpl<Scalar> data, 
                      Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& config, 
                      std::pair<int,int> framesPair,
                      std::pair<Capsule<Scalar>,Capsule<Scalar>> capsulesPair)
{
    Eigen::Matrix<ADScalar, Eigen::Dynamic, 1> distJac;
    // Compute FK
    pinocchio::SE3Tpl<Scalar> f1Mf2 = getRelativePlacement<Scalar>(data, framesPair);
    // Compute frames jacobians
    Eigen::Matrix<ADScalar, 6, 12> Jf1, Jf2;
    Jf1 = Eigen::Matrix<ADScalar, 6, 12>::Zero(6,12);
    Jf2 = Eigen::Matrix<ADScalar, 6, 12>::Zero(6,12);
    pinocchio::computeFrameJacobian(model, data, config, framesPair.first, WORLD, Jf1);
    pinocchio::computeFrameJacobian(model, data, config, framesPair.second, WORLD, Jf2);
    // Extract translation
    

    return distJac;
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