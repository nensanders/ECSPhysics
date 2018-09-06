using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Transforms;
using Unity.Burst;
using UnityEngine.Experimental.LowLevel;
using System;
using System.Runtime.CompilerServices;
using Unity.Collections.LowLevel.Unsafe;

namespace PhysicsEngine
{
    [DisableAutoCreation]
    public class BroadphaseSystem : JobComponentSystem
    {
        struct ColliderGroup
        {
            public readonly int Length;
            public EntityArray Entity;
            public ComponentDataArray<Collider> Collider;
            public ComponentDataArray<AABB> AABB;
            public ComponentDataArray<ColliderType> ColliderType;
        }
        [Inject] ColliderGroup _colliderGroup;

        [Inject] ComponentDataFromEntity<AABB> AABBFromEntity;

        [Inject] PhysicsSystem PhysicsSystem;

        NativeArray<int> mortonCodesA = new NativeArray<int>(10, Allocator.Persistent);
        NativeArray<int> mortonCodesB = new NativeArray<int>(10, Allocator.Persistent);
        NativeArray<int> indexConverterB = new NativeArray<int>(10, Allocator.Persistent);
        NativeArray<int> indexConverterA = new NativeArray<int>(10, Allocator.Persistent);
        NativeArray<int> radixSortBitValues = new NativeArray<int>(10, Allocator.Persistent);
        NativeArray<int> radixSortOffsets = new NativeArray<int>(10, Allocator.Persistent);
        NativeArray<int> sortResultsArrayIsA = new NativeArray<int>(1, Allocator.Persistent);


        public NativeQueue<CollisionPair> SphereSphereCollisionPairsQueue;
        public NativeQueue<CollisionPair> SphereBoxCollisionPairsQueue;
        public NativeQueue<CollisionPair> BoxBoxCollisionPairsQueue;

        public NativeArray<AABB> BVHAABB;
        public NativeArray<Entity> BVHAssociatedEntity;
        public NativeArray<int> BVHRightmostLeafIndex;
        public NativeArray<int> BVHFirstChildIndex;
        public NativeArray<byte> BVHIsValid;
        public NativeArray<byte> BVHColliderType;

        [BurstCompile]
        struct ComputeAndSortMortonCodes : IJob
        {
            [ReadOnly]
            public ComponentDataArray<AABB> aabbs;
            public NativeArray<int> mortonCodesA;
            public NativeArray<int> mortonCodesB;
            public NativeArray<int> indexConverterA;
            public NativeArray<int> indexConverterB;
            public NativeArray<int> radixSortBitValues;
            public NativeArray<int> radixSortOffsets;
            [WriteOnly]
            public NativeArray<int> sortResultsArrayIsA;

            private int zeroesHistogramCounter;
            private int onesHistogramCounter;
            private int zeroesPrefixSum;
            private int onesPrefixSum;

            public void Execute()
            {
                // Calculate all morton codes and init sorted index map
                for (int i = 0; i < aabbs.Length; i++)
                {
                    mortonCodesA[i] = PhysicsMath.CalculateMortonCode(PhysicsMath.GetAABBCenter(aabbs[i]));

                    indexConverterA[i] = i;
                    indexConverterB[i] = i;
                }

                // Radix sort ascending
                for (int bitPosition = 0; bitPosition < 31; bitPosition++)
                {
                    bool isEvenIteration = math.mod(bitPosition, 2) == 0;

                    // init histogram counts
                    zeroesHistogramCounter = 0;
                    onesHistogramCounter = 0;
                    zeroesPrefixSum = 0;
                    onesPrefixSum = 0;

                    // Compute histograms and offsets
                    for (int i = 0; i < aabbs.Length; i++)
                    {
                        int bitVal = 0;
                        if (isEvenIteration)
                        {
                            bitVal = (mortonCodesA[i] & (1 << bitPosition)) >> bitPosition;
                        }
                        else
                        {
                            bitVal = (mortonCodesB[i] & (1 << bitPosition)) >> bitPosition;
                        }
                        
                        radixSortBitValues[i] = bitVal;

                        if (bitVal == 0)
                        {
                            radixSortOffsets[i] = zeroesHistogramCounter;
                            zeroesHistogramCounter += 1;
                        }
                        else
                        {
                            radixSortOffsets[i] = onesHistogramCounter;
                            onesHistogramCounter += 1;
                        }
                    }

                    // calc prefix sum from histogram
                    zeroesPrefixSum = 0;
                    onesPrefixSum = zeroesHistogramCounter;

                    // Reorder array
                    for (int i = 0; i < aabbs.Length; i++)
                    {
                        int newIndex = 0;
                        if (radixSortBitValues[i] == 0)
                        {
                            newIndex = zeroesPrefixSum + radixSortOffsets[i];
                        }
                        else
                        {
                            newIndex = onesPrefixSum + radixSortOffsets[i];
                        }

                        if (isEvenIteration)
                        {
                            mortonCodesB[newIndex] = mortonCodesA[i];
                            indexConverterB[newIndex] = indexConverterA[i];
                        }
                        else
                        {
                            mortonCodesA[newIndex] = mortonCodesB[i];
                            indexConverterA[newIndex] = indexConverterB[i];
                        }
                    }

                    sortResultsArrayIsA[0] = isEvenIteration ? 0 : 1; // it's the A array only for odd number iterations
                }
            }
        }
        
        [BurstCompile]
        struct ConstructBVH : IJob
        {
            [ReadOnly]
            public EntityArray Entity;
            [ReadOnly]
            public ComponentDataArray<AABB> AABB;
            [ReadOnly]
            public ComponentDataArray<ColliderType> ColliderType;
            [ReadOnly]
            public NativeArray<int> indexConverterA;
            [ReadOnly]
            public NativeArray<int> indexConverterB;
            [ReadOnly]
            public NativeArray<int> sortResultsArrayIsA;

            public NativeArray<AABB> BVHAABB;
            public NativeArray<Entity> BVHAssociatedEntity;
            public NativeArray<int> BVHRightmostLeafIndex;
            public NativeArray<int> BVHFirstChildIndex;
            public NativeArray<byte> BVHIsValid;
            public NativeArray<byte> BVHColliderType;

            public void Execute()
            {
                int halfBVHArrayLength = BVHAABB.Length / 2;

                // Populate leaf nodes
                for (int i = 0; i < AABB.Length; i++)
                {
                    int colliderIndex = sortResultsArrayIsA[0] == 1 ? indexConverterA[i] : indexConverterB[i];
                    int bvhIndex = halfBVHArrayLength + i;

                    BVHAABB[bvhIndex] = AABB[colliderIndex];
                    BVHAssociatedEntity[bvhIndex] = Entity[sortResultsArrayIsA[0] == 1 ? indexConverterA[i] : indexConverterB[i]];
                    BVHRightmostLeafIndex[bvhIndex] = bvhIndex;
                    BVHFirstChildIndex[bvhIndex] = -1;
                    BVHIsValid[bvhIndex] = 1;
                    BVHColliderType[bvhIndex] = ColliderType[colliderIndex].Value;
                }

                // Set all parent nodes to valid
                int indexOfLastValidLeafOnLevel = halfBVHArrayLength + AABB.Length - 1;
                while(indexOfLastValidLeafOnLevel > 0)
                {
                    indexOfLastValidLeafOnLevel = PhysicsMath.GetParentBVHNodeIndex(indexOfLastValidLeafOnLevel);

                    if (indexOfLastValidLeafOnLevel == 0)
                    { 
                        BVHIsValid[0] = 1;
                    }
                    else
                    {
                        for (int i = PhysicsMath.GetNextLowestPowerOf2(indexOfLastValidLeafOnLevel) - 1; i <= indexOfLastValidLeafOnLevel; i++)
                        {
                            BVHIsValid[i] = 1;
                        }
                    }
                }

                // Populate parent nodes bottom-up
                for (int i = halfBVHArrayLength - 1; i >= 0; i--)
                {
                    int childNode1 = PhysicsMath.GetFirstChildBVHNodeIndex(i);
                    int childNode2 = childNode1 + 1;

                    int nbNodesOnLevel = PhysicsMath.GetNextLowestPowerOf2(i + 1);
                    int nodeOrderOnLevel = i - nbNodesOnLevel + 2;
                    int leafNodesCount = (BVHAABB.Length + 1) / 2;

                    BVHFirstChildIndex[i] = PhysicsMath.GetFirstChildBVHNodeIndex(i);
                    BVHRightmostLeafIndex[i] = (BVHAABB.Length - 1) - ((leafNodesCount / nbNodesOnLevel) * (nbNodesOnLevel - nodeOrderOnLevel));
                    BVHAABB[i] = PhysicsMath.GetEncompassingAABB(BVHAABB[childNode1], BVHAABB[childNode2]);
                }
            }
        }

        [BurstCompile]
        struct BuildCollisionPairsParallel : IJobParallelFor
        {
            public int HalfBVHArrayLength;

            [ReadOnly]
            [NativeDisableParallelForRestriction]
            public NativeArray<AABB> BVHAABB;
            [ReadOnly]
            [NativeDisableParallelForRestriction]
            public NativeArray<Entity> BVHAssociatedEntity;
            [ReadOnly]
            [NativeDisableParallelForRestriction]
            public NativeArray<int> BVHRightmostLeafIndex;
            [ReadOnly]
            [NativeDisableParallelForRestriction]
            public NativeArray<int> BVHFirstChildIndex;
            [ReadOnly]
            [NativeDisableParallelForRestriction]
            public NativeArray<byte> BVHIsValid;
            [ReadOnly]
            [NativeDisableParallelForRestriction]
            public NativeArray<byte> BVHColliderType;

            [WriteOnly] 
            [NativeDisableParallelForRestriction]
            public NativeQueue<CollisionPair>.Concurrent CollisionPairsQueue;

            void QueryBVHNode(int comparedToNode, int leafNodeIndex)
            {
                if (BVHRightmostLeafIndex[comparedToNode] > leafNodeIndex &&
                    BVHIsValid[comparedToNode] > 0 &&
                    PhysicsMath.AABBToAABBOverlap(BVHAABB[leafNodeIndex], BVHAABB[comparedToNode]))
                {
                    // leaf node
                    if (BVHFirstChildIndex[comparedToNode] < 0)
                    {
                        CollisionPair newPair = new CollisionPair
                        {
                            ColliderEntityA = BVHAssociatedEntity[leafNodeIndex],
                            ColliderEntityB = BVHAssociatedEntity[comparedToNode],
                        };
                        CollisionPairsQueue.Enqueue(newPair);
                    }
                    else
                    {
                        int firstChildIndex = BVHFirstChildIndex[comparedToNode];
                        QueryBVHNode(firstChildIndex, leafNodeIndex);
                        QueryBVHNode(firstChildIndex + 1, leafNodeIndex);
                    }
                }
            }

            public void Execute(int index)
            {
                QueryBVHNode(1, HalfBVHArrayLength + index);
                QueryBVHNode(2, HalfBVHArrayLength + index);
            }
        }

        [BurstCompile]
        struct BuildCollisionPairsSingle : IJob
        {
            public int StartIndex;
            public int IndexCount;

            [ReadOnly]
            public NativeArray<AABB> BVHAABB;
            [ReadOnly]
            public NativeArray<Entity> BVHAssociatedEntity;
            [ReadOnly]
            public NativeArray<int> BVHRightmostLeafIndex;
            [ReadOnly]
            public NativeArray<int> BVHFirstChildIndex;
            [ReadOnly]
            public NativeArray<byte> BVHIsValid;
            [ReadOnly]
            public NativeArray<byte> BVHColliderType;

            [WriteOnly]
            public NativeQueue<CollisionPair> CollisionPairsQueue;

            void QueryBVHNode(int comparedToNode, int leafNodeIndex)
            {
                if (BVHRightmostLeafIndex[comparedToNode] > leafNodeIndex &&
                    BVHIsValid[comparedToNode] > 0 &&
                    PhysicsMath.AABBToAABBOverlap(BVHAABB[leafNodeIndex], BVHAABB[comparedToNode]))
                {
                    // leaf node
                    if (BVHFirstChildIndex[comparedToNode] < 0)
                    {
                        CollisionPairsQueue.Enqueue(new CollisionPair
                        {
                            ColliderEntityA = BVHAssociatedEntity[leafNodeIndex],
                            ColliderEntityB = BVHAssociatedEntity[comparedToNode],
                        });
                    }
                    else
                    {
                        int firstChildIndex = BVHFirstChildIndex[comparedToNode];
                        QueryBVHNode(firstChildIndex, leafNodeIndex);
                        QueryBVHNode(firstChildIndex + 1, leafNodeIndex);
                    }
                }
            }

            public void Execute()
            {
                for (int i = StartIndex; i < StartIndex + IndexCount; i++)
                {
                    QueryBVHNode(1, i);
                    QueryBVHNode(2, i);
                }
            }
        }

        protected override void OnCreateManager(int capacity)
        {
            BVHAABB = new NativeArray<AABB>(10, Allocator.Persistent);
            BVHAssociatedEntity = new NativeArray<Entity>(10, Allocator.Persistent);
            BVHRightmostLeafIndex = new NativeArray<int>(10, Allocator.Persistent);
            BVHFirstChildIndex = new NativeArray<int>(10, Allocator.Persistent);
            BVHIsValid = new NativeArray<byte>(10, Allocator.Persistent);
            BVHColliderType = new NativeArray<byte>(10, Allocator.Persistent);
        }

        protected override void OnDestroyManager()
        {   
            SphereSphereCollisionPairsQueue.Dispose();
            PhysicsSystem.SphereSphereCollisionPairsArray.Dispose();

            BVHAABB.Dispose();
            BVHAssociatedEntity.Dispose();
            BVHRightmostLeafIndex.Dispose();
            BVHFirstChildIndex.Dispose();
            BVHIsValid.Dispose();
            BVHColliderType.Dispose();

            mortonCodesA.Dispose();
            mortonCodesB.Dispose();
            indexConverterA.Dispose();
            indexConverterB.Dispose();
            radixSortBitValues.Dispose();
            radixSortOffsets.Dispose();

            sortResultsArrayIsA.Dispose();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            int collidersCount = _colliderGroup.AABB.Length;
            if(collidersCount <= 0)
            {
                return inputDeps;
            }

            // Resize arrays if needed
            // TODO: just make these NativeLists?
            if (mortonCodesA.Length < collidersCount)
            {
                mortonCodesA.Dispose();
                mortonCodesB.Dispose();
                indexConverterB.Dispose();
                indexConverterA.Dispose();
                radixSortBitValues.Dispose();
                radixSortOffsets.Dispose();

                mortonCodesA = new NativeArray<int>(collidersCount, Allocator.Persistent);
                mortonCodesB = new NativeArray<int>(collidersCount, Allocator.Persistent);
                indexConverterB = new NativeArray<int>(collidersCount, Allocator.Persistent);
                indexConverterA = new NativeArray<int>(collidersCount, Allocator.Persistent);
                radixSortBitValues = new NativeArray<int>(collidersCount, Allocator.Persistent);
                radixSortOffsets = new NativeArray<int>(collidersCount, Allocator.Persistent);
            }

            var sortMortonCodes = new ComputeAndSortMortonCodes
            {
                aabbs = _colliderGroup.AABB,
                mortonCodesA = mortonCodesA,
                mortonCodesB = mortonCodesB,
                indexConverterA = indexConverterA,
                indexConverterB = indexConverterB,
                radixSortBitValues = radixSortBitValues,
                radixSortOffsets = radixSortOffsets,
                sortResultsArrayIsA = sortResultsArrayIsA,
            }.Schedule(inputDeps);

            // Debug sorted mortons
            if (PhysicsSystem.Settings.ShowZOrderCurve)
            {
                sortMortonCodes.Complete();
                bool indexConverterIsA = sortResultsArrayIsA[0] == 1;

                for (int i = 0; i < _colliderGroup.AABB.Length - 1; i++)
                {
                    float3 fromPoint;
                    float3 toPoint;

                    if (indexConverterIsA)
                    {
                        fromPoint = PhysicsMath.GetAABBCenter(_colliderGroup.AABB[indexConverterA[i]]);
                        toPoint = PhysicsMath.GetAABBCenter(_colliderGroup.AABB[indexConverterA[i + 1]]);
                    }
                    else
                    {
                        fromPoint = PhysicsMath.GetAABBCenter(_colliderGroup.AABB[indexConverterB[i]]);
                        toPoint = PhysicsMath.GetAABBCenter(_colliderGroup.AABB[indexConverterB[i + 1]]);
                    }

                    Debug.DrawLine(fromPoint, toPoint, Color.red);
                }
            }

            // Build BVH
            int requiredBVHLength = (PhysicsMath.GetNextHighestPowerOf2(math.ceil_pow2(collidersCount) + 1)) - 1;

            BVHAABB.Dispose();
            BVHAssociatedEntity.Dispose();
            BVHRightmostLeafIndex.Dispose();
            BVHFirstChildIndex.Dispose();
            BVHIsValid.Dispose();
            BVHColliderType.Dispose();
            BVHAABB = new NativeArray<AABB>(requiredBVHLength, Allocator.TempJob);
            BVHAssociatedEntity = new NativeArray<Entity>(requiredBVHLength, Allocator.TempJob);
            BVHRightmostLeafIndex = new NativeArray<int>(requiredBVHLength, Allocator.TempJob);
            BVHFirstChildIndex = new NativeArray<int>(requiredBVHLength, Allocator.TempJob);
            BVHIsValid = new NativeArray<byte>(requiredBVHLength, Allocator.TempJob);
            BVHColliderType = new NativeArray<byte>(requiredBVHLength, Allocator.TempJob);

            var constructBVH = new ConstructBVH()
            {
                Entity = _colliderGroup.Entity,
                AABB = _colliderGroup.AABB,
                ColliderType = _colliderGroup.ColliderType,
                BVHAABB = BVHAABB,
                BVHAssociatedEntity = BVHAssociatedEntity,
                BVHRightmostLeafIndex = BVHRightmostLeafIndex,
                BVHFirstChildIndex = BVHFirstChildIndex,
                BVHIsValid = BVHIsValid,
                BVHColliderType = BVHColliderType,
                indexConverterA = indexConverterA,
                indexConverterB = indexConverterB,
                sortResultsArrayIsA = sortResultsArrayIsA,
            }.Schedule(sortMortonCodes);

            // Draw bvh second-to-last groups
            if (PhysicsSystem.Settings.ShowBVHBounds)
            {
                constructBVH.Complete();
                for (int i = BVHAABB.Length / 4; i < BVHAABB.Length / 2; i++)
                {
                    if (BVHIsValid[i] > 0)
                    {
                        DebugUtils.DrawAABB(BVHAABB[i], UnityEngine.Random.ColorHSV());
                    }
                }
            }

            // Init pairs queue
            if (!SphereSphereCollisionPairsQueue.IsCreated)
            {
                SphereSphereCollisionPairsQueue = new NativeQueue<CollisionPair>(Allocator.Persistent);
            }
            SphereSphereCollisionPairsQueue.Clear();

            JobHandle buildCollisionPairs = new JobHandle();

            //Build Pairs job
            {
                BuildCollisionPairsParallel buildCollisionPairsJob = new BuildCollisionPairsParallel()
                {
                    HalfBVHArrayLength = BVHAABB.Length / 2,
                    BVHAABB = BVHAABB,
                    BVHAssociatedEntity = BVHAssociatedEntity,
                    BVHRightmostLeafIndex = BVHRightmostLeafIndex,
                    BVHFirstChildIndex = BVHFirstChildIndex,
                    BVHIsValid = BVHIsValid,
                    BVHColliderType = BVHColliderType,
                    CollisionPairsQueue = SphereSphereCollisionPairsQueue,
                };
                buildCollisionPairs = buildCollisionPairsJob.Schedule(_colliderGroup.AABB.Length, PhysicsSystem.Settings.BroadphaseSystemBatchCount, constructBVH);
            }

            //{
            //    BuildCollisionPairsSingle buildCollisionPairsJob = new BuildCollisionPairsSingle()
            //    {
            //        StartIndex = BVHAABB.Length / 2,
            //        IndexCount = _colliderGroup.AABB.Length,
            //        BVHAABB = BVHAABB,
            //        BVHAssociatedEntity = BVHAssociatedEntity,
            //        BVHRightmostLeafIndex = BVHRightmostLeafIndex,
            //        BVHFirstChildIndex = BVHFirstChildIndex,
            //        BVHIsValid = BVHIsValid,
            //        BVHColliderType = BVHColliderType,
            //        CollisionPairsQueue = SphereSphereCollisionPairsQueue,
            //    };
            //    buildCollisionPairs = buildCollisionPairsJob.Schedule(constructBVH);
            //}

            // Init pairs array
            buildCollisionPairs.Complete();
            if (PhysicsSystem.SphereSphereCollisionPairsArray.IsCreated)
            {
                PhysicsSystem.SphereSphereCollisionPairsArray.Dispose();
            }
            PhysicsSystem.SphereSphereCollisionPairsArray = new NativeArray<CollisionPair>(SphereSphereCollisionPairsQueue.Count, Allocator.TempJob);

            // Dequeue pairs into array
            DequeueIntoArray<CollisionPair> dequeuePairsJob = new DequeueIntoArray<CollisionPair>()
            {
                InputQueue = SphereSphereCollisionPairsQueue,
                OutputArray = PhysicsSystem.SphereSphereCollisionPairsArray,
            };
            JobHandle dequeueCollisionPairs = dequeuePairsJob.Schedule(buildCollisionPairs);

            //// Debug CollisionPairsArray
            if (PhysicsSystem.Settings.ShowCollisionPairs)
            {
                dequeueCollisionPairs.Complete();

                for (int i = 0; i < PhysicsSystem.SphereSphereCollisionPairsArray.Length; i++)
                {
                    Color randomCol = new Color(UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f));
                    CollisionPair pair = PhysicsSystem.SphereSphereCollisionPairsArray[i];
                    Debug.DrawLine(PhysicsMath.GetAABBCenter(AABBFromEntity[pair.ColliderEntityA]), PhysicsMath.GetAABBCenter(AABBFromEntity[pair.ColliderEntityB]), randomCol);
                }
            }

            // Need to complete jobs here because counter will be read in the next system
            dequeueCollisionPairs.Complete();

            return dequeueCollisionPairs;
        }
    }
}
