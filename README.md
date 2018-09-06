# ECSPhysics
(Under Construction)
A physics engine made with Unity ECS

Made on Unity 2018.2.5f1

Forum discussion: https://forum.unity.com/threads/physics-in-pure-ecs.531716/

# Overview
- "TestScene" is the main test scene. Press play to see the physics simulation happen. Modify the parameters under the "Init" GameObject in the scene (The "PhysxBenchmark" scene is for comparing with Unity's built-in physics)
- There are some additional settings stored in "\PhysicsEngine\Data\Resources\DefaultPhysicsSettings" ScriptableObject. It's possible not everything in there actually works
- Everything in the scene starts with "TestSceneInitializer", which spawns the physics objects
- The physics simulation is all done through "PhysicsSystem". This is where the fixed update is called and all other systems are manually called from it:
  - GlobalGravitySystem: Modifies rigidbody velocities to simulate gravity
  - RigidBodySystem: handles moving rigidbodies based on their velocity
  - UpdateColliderPoseSystem: handles making colliders follow their parented rigidbody (this is still incomplete, might be replaced with new Transform System later)
  - ComputeColliderAABBSystem: computes an AABB for each collider
  - BroadphaseSystem: the goal of this system is to ultimately produce a list of "CollisionPairs", which are pairs of collider entities which are potentially overlapping. It does that in three main steps:
    - ComputeAndSortMortonCodes: sort colliders in order of morton codes (also known as z-order curve)
    - BuildBVH: Creates a BVH with all collider AABBs
    - BuildCollisionPairsParallel: finds overlapping AABBs and stores them as pairs
  - ContactsGenerationSystem: This system's jobs is to take all collision pairs from the previous step and do the actual collision test on them to generate "CollisionManifold"s
  - ConstraintSolverSystem: uses the CollisionManifolds and feeds them to a constraints solver that will compute a set of forces to apply to all rigidbodies in response to the constraints
  
# TODO List (short-term)
- Box colliders
- Rotations/Angular velocity/Moment of Inertia
- friction and coefficient of restitution
- Fix Constraint Solver giving things too much friction (?)
- Optimize broadphase's "BuildCollisionPairsParallel" job. This is by far the most costly thing right now, and I feel like there must be a way to make it much faster. Try simply commenting out the line where we do "CollisionPairsQueue.Enqueue(newPair);" and you'll see that the job is now 100X faster. Cache misses are probably to blame here (?)
- Put debug tools in their own separated editor-only systems

# TODO List (long-term)
- Support multiple contact points per manifold
- rigidbody sleep mecanism
- Capsule colliders
- GJK and EPA for mesh colliders
- Physics queries (raycasts, overlaps, etc...)
- Joints
- Collision events system
- Collision layers & filtering
- Cloth
