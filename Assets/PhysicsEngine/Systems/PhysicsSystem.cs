using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Collections;
using Unity.Jobs;

namespace PhysicsEngine
{
    public struct CollisionPair
    {
        public Entity ColliderEntityA;
        public Entity ColliderEntityB;
    }

    [UpdateAfter(typeof(UnityEngine.Experimental.PlayerLoop.FixedUpdate))]
    public class PhysicsSystem : JobComponentSystem
    {
        public float DeltaTime;

        public PhysicsSettings Settings;

        //public NativeArray<BVHNode> ColliderBVHArray;
        public NativeQueue<CollisionManifold> CollisionManifoldsQueue;
        public NativeArray<CollisionManifold> CollisionManifoldsArray;

        public NativeArray<CollisionPair> SphereSphereCollisionPairsArray;
        public NativeArray<CollisionPair> SphereBoxCollisionPairsArray;
        public NativeArray<CollisionPair> BoxBoxCollisionPairsArray;

        public NativeCounter ConstraintsCounter;
        public NativeArray<Constraint> ConstraintsArray;

        private List<ComponentSystemBase> _physicsSystems;

        protected override void OnCreateManager(int capacity)
        {
            Settings = PhysicsSettings.LoadFromResources();

            _physicsSystems = new List<ComponentSystemBase>();
            _physicsSystems.Add(World.GetOrCreateManager<GlobalGravitySystem>());
            _physicsSystems.Add(World.GetOrCreateManager<RigidBodySystem>());
            _physicsSystems.Add(World.GetOrCreateManager<UpdateColliderPoseSystem>());
            _physicsSystems.Add(World.GetOrCreateManager<ComputeColliderAABBSystem>());
            _physicsSystems.Add(World.GetOrCreateManager<BroadphaseSystem>());
            _physicsSystems.Add(World.GetOrCreateManager<ContactsPhaseInitializationSystem>());
            _physicsSystems.Add(World.GetOrCreateManager<ContactsGenerationSystem>());
            _physicsSystems.Add(World.GetOrCreateManager<ConstraintSolverSystem>());

            for (int i = 0; i < _physicsSystems.Count; i++)
            {
                _physicsSystems[i].Enabled = false;
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            Simulate(Time.fixedDeltaTime);

            return inputDeps;
        }

        public void Simulate(float deltaTime)
        {
            DeltaTime = deltaTime;

            for (int i = 0; i < _physicsSystems.Count; i++)
            {
                _physicsSystems[i].Enabled = true;
                _physicsSystems[i].Update();
                _physicsSystems[i].Enabled = false;
            }
        }
    }
}