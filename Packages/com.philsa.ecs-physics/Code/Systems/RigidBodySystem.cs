using Unity.Entities;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Transforms;
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
            public ComponentDataArray<PositionD> RigidBodyPosition;
            public ComponentDataArray<Velocity> Velocity;
        }
        [Inject] RigidBodyVelocityGroup _rigidBodyVelocityGroup;

        struct RigidBodyAngularVelocityGroup
        {
            public readonly int Length;
            public ComponentDataArray<RigidBody> RigidBody;
            public ComponentDataArray<RotationD> RigidBodyRotation;
            public ComponentDataArray<AngularVelocity> AngularVelocity;
        }
        [Inject] RigidBodyAngularVelocityGroup _rigidBodyAngularVelocityGroup;

        [Inject] PhysicsSystem PhysicsSystem;

        [BurstCompile]
        struct ProcessDrag : IJobParallelFor
        {
            [ReadOnly] public double DeltaTime;
            public ComponentDataArray<Velocity> Velocity;
            [ReadOnly] public ComponentDataArray<LinearDamping> Drag;

            public void Execute(int index)
            {
                if (Drag[index].Value > double.Epsilon)
                {
                    Velocity v = Velocity[index];
                    v.Value = v.Value * math.clamp(1.0 - Drag[index].Value, 0.0, 1.0);
                    Velocity[index] = v;
                }
            }
        }

        [BurstCompile]
        struct ProcessAngularDrag : IJobParallelFor
        {
            [ReadOnly] public double DeltaTime;
            public ComponentDataArray<AngularVelocity> AngularVelocity;
            [ReadOnly] public ComponentDataArray<AngularDamping> AngularDrag;

            public void Execute(int index)
            {
                if (AngularDrag[index].Value > double.Epsilon)
                {
                    AngularVelocity a = AngularVelocity[index];
                    a.Value = a.Value * math.clamp(1.0 - AngularDrag[index].Value, 0.0, 1.0);
                    AngularVelocity[index] = a;
                }
            }
        }

        [BurstCompile]
        struct ProcessVelocity : IJobParallelFor
        {
            [ReadOnly] public double DeltaTime;
            [ReadOnly] public ComponentDataArray<Velocity> Velocity;
            public ComponentDataArray<PositionD> RigidBodyPosition;

            public void Execute(int index)
            {
                PositionD p = RigidBodyPosition[index];
                p.Value = p.Value + (Velocity[index].Value * DeltaTime);
                RigidBodyPosition[index] = p;
            }
        }

        [BurstCompile]
        struct ProcessAngularVelocity : IJobParallelFor
        {
            [ReadOnly] public double DeltaTime;
            public ComponentDataArray<RotationD> RigidBodyRotation;
            [ReadOnly] public ComponentDataArray<AngularVelocity> AngularVelocity;

            public void Execute(int index)
            {
                RotationD r = RigidBodyRotation[index];
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
