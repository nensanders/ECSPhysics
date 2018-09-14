# ECSPhysics

![Alt Text](https://i.gyazo.com/0faf0c71d649f52227055b9e5a1bfb60.gif)

(Under Construction)
A physics engine made with Unity ECS

Task management (Trello): https://trello.com/b/XcYxQFrh/ecsphysics

Forum discussion: https://forum.unity.com/threads/physics-in-pure-ecs.531716/

# Overview
- "TestScene_ECS" is the main test scene. Press play to see the physics simulation happen. Modify the parameters under the "Init" GameObject in the scene (The "TestScene_PhysX" scene is for comparing with Unity's built-in physics)
- There are some additional settings stored in the "\PhysicsEngine\Data\Resources\DefaultPhysicsSettings" ScriptableObject. It's possible that not everything in there actually works
- Everything in the scene starts with the "TestSceneInitializer" script, which spawns the physics objects
- The physics simulation is all done through "PhysicsSystem". This is where the fixed update is called and all other systems are manually called from it:
  - UniformGravitySystem: Modifies rigidbody velocities to simulate gravity
  - RigidBodySystem: handles moving rigidbodies based on their velocity
  - UpdateColliderPoseSystem: handles making colliders follow their parented rigidbody (this is still incomplete, might be replaced with new Transform System later)
  - ComputeColliderAABBSystem: computes an AABB for each collider
  - BroadphaseSystem: the goal of this system is to ultimately produce a list of "CollisionPairs", which are pairs of collider entities which are potentially overlapping. It does that in three main steps:
    - ComputeAndSortMortonCodes: sort colliders in order of morton codes (also known as z-order curve)
    - BuildBVH: Creates a BVH with all collider AABBs
    - BuildCollisionPairsParallel: finds overlapping AABBs and stores them as pairs
  - ContactsGenerationSystem: This system's jobs is to take all collision pairs from the previous step and do the actual collision test on them to generate "CollisionManifold"s
  - ConstraintSolverSystem: uses the CollisionManifolds and feeds them to a constraints solver that will compute a set of forces to apply to all rigidbodies in response to the constraints
  
