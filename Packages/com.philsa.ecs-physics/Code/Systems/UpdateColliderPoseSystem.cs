using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using Unity.Transforms;
using UnityEngine.Experimental.LowLevel;

namespace PhysicsEngine
{
    [DisableAutoCreation]
    public class UpdateColliderPoseSystem : JobComponentSystem
    {
        struct ColliderGroup
        {
            public readonly int Length;
            public EntityArray Entity;
            public ComponentDataArray<Collider> Collider;
        }
        [Inject] ColliderGroup _colliders;

        [Inject] ComponentDataFromEntity<Unity.Transforms.Position> PositionFromEntity;
        [Inject] ComponentDataFromEntity<Unity.Transforms.Rotation> RotationFromEntity;

        [Inject] PhysicsSystem PhysicsSystem;

        [BurstCompile]
        struct UpdateColliderPose : IJobParallelFor
        {
            [ReadOnly] public EntityArray ColliderEntity;
            [ReadOnly] public ComponentDataArray<Collider> Collider;

            [NativeDisableParallelForRestriction]
            public ComponentDataFromEntity<Unity.Transforms.Position> PositionFromEntity;
            [NativeDisableParallelForRestriction]
            public ComponentDataFromEntity<Unity.Transforms.Rotation> RotationFromEntity;

            public void Execute(int index)
            {
                Entity colliderEntity = ColliderEntity[index];

                Unity.Transforms.Position cp = PositionFromEntity[colliderEntity];
                cp.Value = PositionFromEntity[Collider[index].RigidBodyEntity].Value;
                PositionFromEntity[colliderEntity] = cp;

                Unity.Transforms.Rotation cr = RotationFromEntity[colliderEntity];
                cr.Value = RotationFromEntity[Collider[index].RigidBodyEntity].Value;
                RotationFromEntity[colliderEntity] = cr;
            }
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var updateColliderPose = new UpdateColliderPose
            {
                ColliderEntity = _colliders.Entity,
                Collider = _colliders.Collider,
                PositionFromEntity = PositionFromEntity,
                RotationFromEntity = RotationFromEntity,
            }.Schedule(_colliders.Length, PhysicsSystem.Settings.ColliderPoseSystemBatchCount, inputDeps);

            return updateColliderPose;
        }
    }
}
