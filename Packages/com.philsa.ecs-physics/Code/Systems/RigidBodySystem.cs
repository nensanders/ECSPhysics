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
    public class RigidBodySystem : JobComponentSystem
    {
        struct RigidBodyDragGroup
        {
            public readonly int Length;
            public ComponentDataArray<RigidBody> RigidBody;
            public ComponentDataArray<Velocity> Velocity;
            public ComponentDataArray<LinearDamping> Drag;
        }
        [Inject] RigidBodyDragGroup _rigidBodyDragGroup;

        [Inject] PhysicsSystem _physicsSystem;

        struct RigidBodyAngularDragGroup
        {
            public readonly int Length;
            public ComponentDataArray<RigidBody> RigidBody;
            public ComponentDataArray<AngularVelocity> AngularVelocity;
            public ComponentDataArray<AngularDamping> AngularDrag;
        }
        [Inject] RigidBodyAngularDragGroup _rigidBodyAngularDragGroup;
        
        struct RigidBodyVelocityGroup
        {
            public readonly int Length;
            public ComponentDataArray<RigidBody> RigidBody;
            public ComponentDataArray<Position> RigidBodyPosition;
            public ComponentDataArray<Velocity> Velocity;
        }
        [Inject] RigidBodyVelocityGroup _rigidBodyVelocityGroup;
        
        struct RigidBodyAngularVelocityGroup
        {
            public readonly int Length;
            public ComponentDataArray<RigidBody> RigidBody;
            public ComponentDataArray<Rotation> RigidBodyRotation;
            public ComponentDataArray<AngularVelocity> AngularVelocity;
        }
        [Inject] RigidBodyAngularVelocityGroup _rigidBodyAngularVelocityGroup;

        [Inject] PhysicsSystem PhysicsSystem;

        [BurstCompile]
        struct ProcessDrag : IJobParallelFor
        {
            [ReadOnly] public float DeltaTime;
            public ComponentDataArray<Velocity> Velocity;
            [ReadOnly] public ComponentDataArray<LinearDamping> Drag;

            public void Execute(int index)
            {
                if (Drag[index].Value > 0f)
                {
                    Velocity v = Velocity[index];
                    v.Value = v.Value * math.clamp(1f - Drag[index].Value, 0f, 1f);
                    Velocity[index] = v;
                }
            }
        }

        [BurstCompile]
        struct ProcessAngularDrag : IJobParallelFor
        {
            [ReadOnly] public float DeltaTime;
            public ComponentDataArray<AngularVelocity> AngularVelocity;
            [ReadOnly] public ComponentDataArray<AngularDamping> AngularDrag;

            public void Execute(int index)
            {
                if (AngularDrag[index].Value > 0f)
                {
                    AngularVelocity a = AngularVelocity[index];
                    a.Value = a.Value * math.clamp(1f - AngularDrag[index].Value, 0f, 1f);
                    AngularVelocity[index] = a;
                }
            }
        }

        [BurstCompile]
        struct ProcessVelocity : IJobParallelFor
        {
            [ReadOnly] public float DeltaTime;
            [ReadOnly] public ComponentDataArray<Velocity> Velocity;
            public ComponentDataArray<Position> RigidBodyPosition;

            public void Execute(int index)
            {
                Position p = RigidBodyPosition[index];
                p.Value = p.Value + (Velocity[index].Value * DeltaTime);
                RigidBodyPosition[index] = p;
            }
        }

        [BurstCompile]
        struct ProcessAngularVelocity : IJobParallelFor
        {
            [ReadOnly] public float DeltaTime;
            public ComponentDataArray<Rotation> RigidBodyRotation;
            [ReadOnly] public ComponentDataArray<AngularVelocity> AngularVelocity;

            public void Execute(int index)
            {
                Rotation r = RigidBodyRotation[index];
                //r.Value = math.mul(math.euler(AngularVelocity[index].Value * DeltaTime), RigidBodyRotation[index].Value);
                RigidBodyRotation[index] = r;
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var processVelocity = new ProcessVelocity
            {
                DeltaTime = _physicsSystem.DeltaTime,
                RigidBodyPosition = _rigidBodyVelocityGroup.RigidBodyPosition,
                Velocity = _rigidBodyVelocityGroup.Velocity,
            }.Schedule(_rigidBodyVelocityGroup.Length, PhysicsSystem.Settings.RigidbodySystemBatchCount, inputDeps);

            var processAngularVelocity = new ProcessAngularVelocity
            {
                DeltaTime = _physicsSystem.DeltaTime,
                RigidBodyRotation = _rigidBodyAngularVelocityGroup.RigidBodyRotation,
                AngularVelocity = _rigidBodyAngularVelocityGroup.AngularVelocity,
            }.Schedule(_rigidBodyAngularVelocityGroup.Length, PhysicsSystem.Settings.RigidbodySystemBatchCount, processVelocity);

            var processDrag = new ProcessDrag
            {
                DeltaTime = _physicsSystem.DeltaTime,
                Velocity = _rigidBodyDragGroup.Velocity,
                Drag = _rigidBodyDragGroup.Drag,
            }.Schedule(_rigidBodyDragGroup.Length, PhysicsSystem.Settings.RigidbodySystemBatchCount, processAngularVelocity);

            var processAngularDrag = new ProcessAngularDrag
            {
                DeltaTime = _physicsSystem.DeltaTime,
                AngularVelocity = _rigidBodyAngularDragGroup.AngularVelocity,
                AngularDrag = _rigidBodyAngularDragGroup.AngularDrag,
            }.Schedule(_rigidBodyAngularDragGroup.Length, PhysicsSystem.Settings.RigidbodySystemBatchCount, processDrag);

            return processAngularDrag;
        }
    }
}
