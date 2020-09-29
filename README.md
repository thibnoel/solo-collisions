# SOLO autocollisions avoidance
This repository contains the methods required to characterize and approximate the collisions distances and jacobians for the relevant pairs of the SOLO8 and SOLO12 robots. A procedure for C source code generation that returns the same collision data is also provided. Finally, we use this data in a collisions avoidance controller using virtual repulsive torques.

It relies on the Pinocchio and HPP-FCL open-source C++ libraries, developed at LAAS-CNRS, and other standard Python or C++ packages. 

## Leg to leg collision
The case of a collision involving a pair of legs segments is handled with a standard, forward kinematics-based approach. The relative placement of the segments frames can be obtained with Pinocchio; the legs meshes are then approximatd by capsules primitives, for which the distance computation boils down to a  3D segment-to-segment distance. This distance is easy to reimplement and is then used as part of the code generation.

## Leg to body collision
For the collisions with the body, the previous approach is not so relevant (fixed spatial distance at the shoulder). We thus  take a different route, and start by sampling the collision obstacle for the relevant pairs in the robot configurations space. This sampling allows us to determin the collision obstacle boundary in articular space. A new definition for the distance, defined with respect to this boundary, then provides a better distance field in articular space. This distance field must then be approximated as a quickly-evaluated model. Our method does this using a standard MLP neural network.

## Example scripts
**Python :**\
`solo_shoulder_coll_sampling.py` : characterizing the shoulder collision for SOLO12\
`talos_shoulder_coll_sampling.py` : characterizing the shoulder collision for Talos\
`solo_pybullet_sim.py` : control example for the collisions avoidance of SOLO in simulation\

**C++ :**
