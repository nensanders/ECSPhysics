using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Transforms;
using UnityEngine.Experimental.LowLevel;
using Unity.Burst;

namespace PhysicsEngine
{
    [DisableAutoCreation]
    public class GlobalGravitySystem : JobComponentSystem
    {
        public float3 GlobalGravity = new float3(0, -9.81f, 0);

        struct RigidBodyGroup
        {
            public readonly int Length;
            public ComponentDataArray<RigidBody> RigidBody;
            public ComponentDataArray<GlobalGravity> GlobalGravity;
            public ComponentDataArray<Velocity> Velocity;
        }
        [Inject] RigidBodyGroup _rigidbodies;

        [Inject] PhysicsSystem _physicsSystem;

        [BurstCompile]
        struct ApplyGlobalGravity : IJobParallelFor
        {
            [ReadOnly] public float DeltaTime;
            [ReadOnly] public float3 GlobalGravity;
            public ComponentDataArray<Velocity> Velocity;

            public void Execute(int index)
            {
                Velocity v = Velocity[index];
                v.Value += GlobalGravity * DeltaTime;
                Velocity[index] = v;
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var applyGlobalGravity = new ApplyGlobalGravity
            {
                DeltaTime = _physicsSystem.DeltaTime,
                GlobalGravity = GlobalGravity,
                Velocity = _rigidbodies.Velocity,
            }.Schedule(_rigidbodies.Length, 64, inputDeps);

            return applyGlobalGravity;
        }
    }
}