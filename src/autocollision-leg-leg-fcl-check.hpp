#include "autocollision-leg-leg-codegen.hpp"

// Checks the collision distances with FCL
// directly from simplified URDF file data
Eigen::Matrix<double, 8, 8> getSoloFCLResult(pinocchio::Model& model, 
                                            pinocchio::GeometryModel& gmodel, 
                                            pinocchio::Data& data, 
                                            Eigen::Matrix<double, Eigen::Dynamic, 1>& config)
{
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