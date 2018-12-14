using Unity.Entities;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Transforms;
using Unity.Burst;

namespace PhysicsEngine
{
    [DisableAutoCreation]
    public class GlobalGravitySystem : JobComponentSystem
    {
        struct RigidBodyGroup
        {
            public readonly int Length;
            public ComponentDataArray<RigidBody> RigidBody;
            public ComponentDataArray<UniformGravity> GlobalGravity;
            public ComponentDataArray<Velocity> Velocity;
            public ComponentDataArray<PositionD> PositionD;
            public ComponentDataArray<Position> Position;
        }

        [Inject] RigidBodyGroup _rigidbodies;

        [Inject] PhysicsSystem _physicsSystem;

        [BurstCompile]
        struct ApplyGlobalGravity : IJobParallelFor
        {
            [ReadOnly] public double DeltaTime;
            [ReadOnly] public ComponentDataArray<RigidBody> RigidBody;
            [ReadOnly] public ComponentDataArray<PositionD> PositionD;
            public ComponentDataArray<Position> Position;
            public ComponentDataArray<Velocity> Velocity;

            public void Execute(int index)
            {
                Velocity v = Velocity[index];
                PositionD p = PositionD[index];
                Position pFloat = Position[index];
                RigidBody r = RigidBody[index];


                // double comp1 = 6.6740831e-11 / distance;  // G
                //  double comp2 = vesselInvMass/5.9722e24; // Planet Mass


                double3 aPos = new double3(0f, 0f, 0f);  // Planet
                double3 bPos = p.Value;                  // Vessel

                double3 fromAToBVector = aPos - bPos;

                double vesselInvMass = r.InverseMass;
                double distance = math.lengthsq(fromAToBVector);

                double comp1 = 0.006674 / distance;  // G
                double comp2 = vesselInvMass/75000000.0; // Planet Mass

                double resultingForce = comp1 / comp2;
                fromAToBVector = math.normalize(fromAToBVector) * resultingForce;
                v.Value += fromAToBVector * DeltaTime;

                Velocity[index] = v;


                pFloat.Value = new float3((float)p.Value.x, (float)p.Value.y, (float)p.Value.z);
                Position[index] = pFloat;
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var applyGlobalGravity = new ApplyGlobalGravity
            {
                DeltaTime = _physicsSystem.DeltaTime,
                RigidBody = _rigidbodies.RigidBody,
                Velocity = _rigidbodies.Velocity,
                PositionD = _rigidbodies.PositionD,
                Position = _rigidbodies.Position
            }.Schedule(_rigidbodies.Length, 64, inputDeps);

            return applyGlobalGravity;
        }
    }
}