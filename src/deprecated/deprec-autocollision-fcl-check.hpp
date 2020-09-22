#include "autocollision-capsules-code-generation.hpp"

typedef boost::shared_ptr< fcl::CollisionGeometry > CollisionGeometryPtr;
// Helper function : returns the jMf SE3 placement given a frame f, j being its parent joint
pinocchio::SE3 parentToFrameRelativePlacement(pinocchio::Model model, pinocchio::Data data, int frameInd)
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
    pinocchio::SE3 j1Mframe1 = parentToFrameRelativePlacement(model, data, framesPair.first);
    pinocchio::SE3 j2Mframe2 = parentToFrameRelativePlacement(model, data, framesPair.second);

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

// Add an hpp::fcl::Capsule geometry to a specified frame of the robot geometry model
pinocchio::GeomIndex addFCLCapsule(pinocchio::Model& model, 
                    pinocchio::GeometryModel& gmodel, 
                    pinocchio::Data& data,
                    Capsule<double> caps, 
                    int frameIndex, 
                    std::string caps_name)
{
    // Get frames
    pinocchio::SE3 jMf = parentToFrameRelativePlacement(model, data, frameIndex);
    pinocchio::SE3 fMc = caps.get_fMc();
    // Initialize FCL geometry
    const CollisionGeometryPtr caps_geom (new hpp::fcl::Capsule(caps.radius, caps.getLength())); 
    pinocchio::GeometryObject caps_gobj(caps_name,
                                         frameIndex,
                                         model.frames[frameIndex].parent,
                                         caps_geom, 
                                         jMf.act(fMc));
    const pinocchio::GeomIndex caps_index = gmodel.addGeometryObject(caps_gobj, model);
    // return geometry index
    return caps_index;
}

// Legs capsules approximations given as a list (len 4) of Capsules in following order :
// legsCapsulesApprox =[LU_capsApprox, LL_capsApprox, RU_capsApprox, RL_capsApprox]
Eigen::Matrix<double, 8, 8> getSoloFCLResult(pinocchio::Model& model, 
                                            pinocchio::GeometryModel& gmodel, 
                                            pinocchio::Data& data, 
                                            Eigen::Matrix<double, Eigen::Dynamic, 1>& config, 
                                            Capsule<double>* legsCapsulesApprox)
{
    // Initialize legs frames
    int frames[8] = {(int)model.getFrameId("FL_UPPER_LEG"),
                        (int)model.getFrameId("FL_LOWER_LEG"),
                        (int)model.getFrameId("FR_UPPER_LEG"),
                        (int)model.getFrameId("FR_LOWER_LEG"),
                        (int)model.getFrameId("HL_UPPER_LEG"),
                        (int)model.getFrameId("HL_LOWER_LEG"),
                        (int)model.getFrameId("HR_UPPER_LEG"),
                        (int)model.getFrameId("HR_LOWER_LEG")};
    // Attach corresponding capsules 
    pinocchio::GeomIndex capsulesInd[8] = {addFCLCapsule(model, gmodel, data, legsCapsulesApprox[0], frames[0], "FLU_caps"),
                                           addFCLCapsule(model, gmodel, data, legsCapsulesApprox[1], frames[1], "FLL_caps"),
                                           addFCLCapsule(model, gmodel, data, legsCapsulesApprox[2], frames[2], "FRU_caps"),
                                           addFCLCapsule(model, gmodel, data, legsCapsulesApprox[3], frames[3], "FRL_caps"),
                                           addFCLCapsule(model, gmodel, data, legsCapsulesApprox[0], frames[4], "HLU_caps"),
                                           addFCLCapsule(model, gmodel, data, legsCapsulesApprox[1], frames[5], "HLL_caps"),
                                           addFCLCapsule(model, gmodel, data, legsCapsulesApprox[2], frames[6], "HRU_caps"),
                                           addFCLCapsule(model, gmodel, data, legsCapsulesApprox[3], frames[7], "HRL_caps")};
    // Add corresponding collision pairs to the model
    for(int i=0; i<8; i++)
    {
        for(int j=i+1; j<8; j++)
        {
            gmodel.addCollisionPair(pinocchio::CollisionPair(capsulesInd[i],capsulesInd[j]));
        }
    }
    // Compute the distance results
    pinocchio::GeometryData gdata(gmodel);
    pinocchio::computeDistances(model,data,gmodel,gdata,config);
    std::vector< fcl::DistanceResult > collisions_dist = gdata.distanceResults; 
    // Populate a nicely shaped matrix with the results
    Eigen::Matrix<double, 8, 8> result;
    for(int i=0; i<8; i++)
    {
        result(i,i) = -1;
        for(int j=i+1; j<8; j++)
        {
            result(i,j) = collisions_dist[gmodel.findCollisionPair(pinocchio::CollisionPair(capsulesInd[i],capsulesInd[j]))].min_distance;
            result(j,i) = collisions_dist[gmodel.findCollisionPair(pinocchio::CollisionPair(capsulesInd[i],capsulesInd[j]))].min_distance;
        }
    }
    return result;
}

// New FCL check, directly from simplified URDF file
Eigen::Matrix<double, 8, 8> newGetSoloFCLResult(pinocchio::Model& model, 
                                            pinocchio::GeometryModel& gmodel, 
                                            pinocchio::Data& data, 
                                            Eigen::Matrix<double, Eigen::Dynamic, 1>& config)
{
    // Initialize legs frames
    /*int frames[8] = {(int)model.getFrameId("FL_UPPER_LEG"),
                        (int)model.getFrameId("FL_LOWER_LEG"),
                        (int)model.getFrameId("FR_UPPER_LEG"),
                        (int)model.getFrameId("FR_LOWER_LEG"),
                        (int)model.getFrameId("HL_UPPER_LEG"),
                        (int)model.getFrameId("HL_LOWER_LEG"),
                        (int)model.getFrameId("HR_UPPER_LEG"),
                        (int)model.getFrameId("HR_LOWER_LEG")};*/

    // Initialize legs geometries (capsules in the simplified model)
    int geoms[8] = {(int)gmodel.getGeometryId("FL_UPPER_LEG_0"),
                        (int)gmodel.getGeometryId("FL_LOWER_LEG_0"),
                        (int)gmodel.getGeometryId("FR_UPPER_LEG_0"),
                        (int)gmodel.getGeometryId("FR_LOWER_LEG_0"),
                        (int)gmodel.getGeometryId("HL_UPPER_LEG_0"),
                        (int)gmodel.getGeometryId("HL_LOWER_LEG_0"),
                        (int)gmodel.getGeometryId("HR_UPPER_LEG_0"),
                        (int)gmodel.getGeometryId("HR_LOWER_LEG_0")};

    // Add corresponding collision pairs to the model
    for(int i=0; i<8; i++)
    {
        for(int j=i+1; j<8; j++)
        {
            gmodel.addCollisionPair(pinocchio::CollisionPair(geoms[i],geoms[j]));
        }
    }

    // Compute the distance results
    pinocchio::GeometryData gdata(gmodel);
    pinocchio::computeDistances(model,data,gmodel,gdata,config);
    std::vector< fcl::DistanceResult > collisions_dist = gdata.distanceResults; 
    // Populate a nicely shaped matrix with the results
    Eigen::Matrix<double, 8, 8> result;
    for(int i=0; i<8; i++)
    {
        result(i,i) = -1;
        for(int j=i+1; j<8; j++)
        {
            result(i,j) = collisions_dist[gmodel.findCollisionPair(pinocchio::CollisionPair(geoms[i],geoms[j]))].min_distance;
            result(j,i) = collisions_dist[gmodel.findCollisionPair(pinocchio::CollisionPair(geoms[i],geoms[j]))].min_distance;
        }
    }
    return result;
}