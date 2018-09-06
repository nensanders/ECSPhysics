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
    public class FollowRigidBodySystem : JobComponentSystem
    {
        struct FollowRigidBodyGroup
        {
            public readonly int Length;
            public EntityArray Entities;
            public ComponentDataArray<FollowRigidBody> FollowRigidBody;
        }
        [Inject] FollowRigidBodyGroup _followRigidbodies;

        [Inject] ComponentDataFromEntity<Unity.Transforms.Position> RigidBodyPositionFromEntity;
        [Inject] ComponentDataFromEntity<Unity.Transforms.Rotation> RigidBodyRotationFromEntity;

        [BurstCompile]
        struct FollowRigidBodyJob : IJobParallelFor
        {
            [ReadOnly] public EntityArray Entities;
            [ReadOnly] public ComponentDataArray<FollowRigidBody> FollowRigidbody;
            [NativeDisableParallelForRestriction]
            public ComponentDataFromEntity<Unity.Transforms.Position> PositionFromEntity;
            [NativeDisableParallelForRestriction]
            public ComponentDataFromEntity<Unity.Transforms.Rotation> RotationFromEntity;

            public void Execute(int index)
            {
                Entity transformEntity = Entities[index];
                Entity followedRigidBodyEntity = FollowRigidbody[index].RigidBodyEntity;

                Unity.Transforms.Position p = PositionFromEntity[transformEntity];
                p.Value = PositionFromEntity[followedRigidBodyEntity].Value;
                PositionFromEntity[transformEntity] = p;

                Unity.Transforms.Rotation r = RotationFromEntity[transformEntity];
                r.Value = RotationFromEntity[followedRigidBodyEntity].Value;
                RotationFromEntity[transformEntity] = r;
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var moveRigidbodies = new FollowRigidBodyJob
            {
                FollowRigidbody = _followRigidbodies.FollowRigidBody,
                Entities = _followRigidbodies.Entities,
                PositionFromEntity = RigidBodyPositionFromEntity,
                RotationFromEntity = RigidBodyRotationFromEntity,
            }.Schedule(_followRigidbodies.Length, 1, inputDeps);

            return moveRigidbodies;
        }
    }
}
