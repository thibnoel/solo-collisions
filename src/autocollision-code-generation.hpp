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

        static pinocchio::SE3Tpl<Scalar> fMc(rotation, translation);

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
        static struct Capsule<AltScalar> caps_cast = {a_cast, b_cast, radius_cast};

        return caps_cast;
    }
};

// Returns a pair of int representing two frames, from their names and the robot model
std::pair<int,int> getFramesPair(std::string frameName1, std::string frameName2, pinocchio::Model model)
{
    return std::make_pair((int)model.getFrameId(frameName1),(int)model.getFrameId(frameName2));
}

// SOLO-specific
// Returns a list[4] of pairs of int representing all the collision pairs between 2 legs of SOLO
// The legs are given are passed as strings from the following list : {"FL","FR","HL","HR"}
std::pair<int,int>* getLegToLegFramesPairs(std::string leg1, std::string leg2, pinocchio::Model model)
{
    static std::pair<int,int> pairs[4] = {getFramesPair(leg1 + "_UPPER_LEG", leg2 + "_UPPER_LEG", model),
                                        getFramesPair(leg1 + "_UPPER_LEG", leg2 + "_LOWER_LEG", model),
                                        getFramesPair(leg1 + "_LOWER_LEG", leg2 + "_UPPER_LEG", model),
                                        getFramesPair(leg1 + "_LOWER_LEG", leg2 + "_LOWER_LEG", model)};
    return pairs;
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
    // Modifies the model ! NOT GOOD
double getPairFCLResult(pinocchio::Model& model, 
                    pinocchio::GeometryModel& gmodel, 
                    pinocchio::Data& data, 
                    Eigen::Matrix<double, Eigen::Dynamic, 1>& config, 
                    std::pair<int,int> framesPair,
                    std::pair<Capsule<double>,Capsule<double>> capsulesPair)
{
    typedef boost::shared_ptr< fcl::CollisionGeometry > CollisionGeometryPtr;

    pinocchio::SE3 j1Mframe1 = jointToFrameRelativePlacement(model, data, framesPair.first);
    pinocchio::SE3 j2Mframe2 = jointToFrameRelativePlacement(model, data, framesPair.second);

    pinocchio::SE3 f1Mc1 = capsulesPair.first.get_fMc();
    pinocchio::SE3 f2Mc2 = capsulesPair.second.get_fMc();

    // Initialize fcl geometries
    const CollisionGeometryPtr caps_geom1 (new hpp::fcl::Capsule(capsulesPair.first.radius, capsulesPair.first.getLength())); 
    const CollisionGeometryPtr caps_geom2 (new hpp::fcl::Capsule(capsulesPair.second.radius, capsulesPair.second.getLength()));

    // Initialize geometry objects
        // Capsule 1
    std::string caps1_name = std::string("caps1_" + std::to_string(framesPair.first));
    pinocchio::GeometryObject caps1_gobj(caps1_name,
                                         framesPair.first,
                                         model.frames[framesPair.first].parent,
                                         caps_geom1, 
                                         j1Mframe1.act(f1Mc1));    
        // Capsule 2
    std::string caps2_name = std::string("caps2_" + std::to_string(framesPair.second));  
    pinocchio::GeometryObject caps2_gobj(caps2_name,
                                         framesPair.second,
                                         model.frames[framesPair.second].parent,
                                         caps_geom2, 
                                         j2Mframe2.act(f2Mc2));     
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