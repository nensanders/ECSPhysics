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
    public class ComputeColliderAABBSystem : JobComponentSystem
    {
        struct SphereColliderGroup
        {
            public readonly int Length;
            public ComponentDataArray<SphereCollider> SphereCollider;
            public ComponentDataArray<Unity.Transforms.Position> ColliderPosition;
            public ComponentDataArray<AABB> AABB;
        }
        [Inject] SphereColliderGroup _sphereColliders;

        //struct BoxColliderGroup
        //{
        //    public int Length;
        //    public ComponentDataArray<BoxCollider> BoxCollider;
        //    public ComponentDataArray<ColliderPosition> ColliderPosition;
        //    public ComponentDataArray<ColliderRotation> ColliderRotation;
        //    public ComponentDataArray<AABB> AABB;
        //}
        //[Inject] BoxColliderGroup _boxColliders;

        struct AABBGroup
        {
            public readonly int Length;
            public ComponentDataArray<AABB> AABB;
        }
        [Inject] AABBGroup _aabbs;

        [Inject] PhysicsSystem PhysicsSystem;

        [BurstCompile]
        struct ComputeShpereColliderAABB : IJobParallelFor
        {
            [ReadOnly] public ComponentDataArray<SphereCollider> SphereCollider;
            [ReadOnly] public ComponentDataArray<Unity.Transforms.Position> ColliderPosition;
            public ComponentDataArray<AABB> AABB;

            public void Execute(int index)
            {
                float3 halfSize = new float3(SphereCollider[index].Radius, SphereCollider[index].Radius, SphereCollider[index].Radius);

                AABB aabb = AABB[index];
                aabb.Min = ColliderPosition[index].Value - halfSize;
                aabb.Max = ColliderPosition[index].Value + halfSize;
                AABB[index] = aabb;
            }
        }

        //[BurstCompile]
        //struct ComputeBoxColliderAABB : IJobParallelFor
        //{
        //    [ReadOnly] public ComponentDataArray<BoxCollider> BoxCollider;
        //    [ReadOnly] public ComponentDataArray<ColliderPosition> ColliderPosition;
        //    [ReadOnly] public ComponentDataArray<ColliderRotation> ColliderRotation;
        //    public ComponentDataArray<AABB> AABB;

        //    public void Execute(int index)
        //    {
        //        float3 origHalfSize = BoxCollider[index].HalfSize;

        //        AABB aabb = new AABB();
                
        //        aabb.Min = ColliderPosition[index].Value;
        //        aabb.Max = ColliderPosition[index].Value;

        //        PhysicsMath.GrowAABB(ref aabb, ColliderPosition[index].Value + math.mul(ColliderRotation[index].Value, new float3(origHalfSize.x, origHalfSize.y, origHalfSize.z)));
        //        PhysicsMath.GrowAABB(ref aabb, ColliderPosition[index].Value + math.mul(ColliderRotation[index].Value, new float3(origHalfSize.x, origHalfSize.y, -origHalfSize.z)));
        //        PhysicsMath.GrowAABB(ref aabb, ColliderPosition[index].Value + math.mul(ColliderRotation[index].Value, new float3(origHalfSize.x, -origHalfSize.y, origHalfSize.z)));
        //        PhysicsMath.GrowAABB(ref aabb, ColliderPosition[index].Value + math.mul(ColliderRotation[index].Value, new float3(origHalfSize.x, -origHalfSize.y, -origHalfSize.z)));
        //        PhysicsMath.GrowAABB(ref aabb, ColliderPosition[index].Value + math.mul(ColliderRotation[index].Value, new float3(-origHalfSize.x, origHalfSize.y, origHalfSize.z)));
        //        PhysicsMath.GrowAABB(ref aabb, ColliderPosition[index].Value + math.mul(ColliderRotation[index].Value, new float3(-origHalfSize.x, origHalfSize.y, -origHalfSize.z)));
        //        PhysicsMath.GrowAABB(ref aabb, ColliderPosition[index].Value + math.mul(ColliderRotation[index].Value, new float3(-origHalfSize.x, -origHalfSize.y, origHalfSize.z)));
        //        PhysicsMath.GrowAABB(ref aabb, ColliderPosition[index].Value + math.mul(ColliderRotation[index].Value, new float3(-origHalfSize.x, -origHalfSize.y, -origHalfSize.z)));

        //        AABB[index] = aabb;
        //    }
        //}

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            var computeShpereColliderAABB = new ComputeShpereColliderAABB
            {
                SphereCollider = _sphereColliders.SphereCollider,
                ColliderPosition = _sphereColliders.ColliderPosition,
                AABB = _sphereColliders.AABB,
            }.Schedule(_sphereColliders.Length, PhysicsSystem.Settings.ColliderAABBSystemBatchCount, inputDeps);

            //var computeBoxColliderAABB = new ComputeBoxColliderAABB
            //{
            //    BoxCollider = _boxColliders.BoxCollider,
            //    ColliderPosition = _boxColliders.ColliderPosition,
            //    ColliderRotation = _boxColliders.ColliderRotation,
            //    AABB = _boxColliders.AABB,
            //}.Schedule(_boxColliders.Length, PhysicsSystem.Settings.ColliderAABBSystemBatchCount, computeShpereColliderAABB);

            if(PhysicsSystem.Settings.ShowAABBs)
            {
                computeShpereColliderAABB.Complete();
                for (int i = 0; i < _aabbs.Length; i++)
                {
                    DebugUtils.DrawAABB(_aabbs.AABB[i], UnityEngine.Random.ColorHSV());
                }
            }

            return computeShpereColliderAABB;
        }
    }
}
