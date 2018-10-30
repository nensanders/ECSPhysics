using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using UnityEngine.Experimental.LowLevel;
using System;
using System.Runtime.CompilerServices;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs.LowLevel.Unsafe;

namespace PhysicsEngine
{
    public struct NodePair
    {
        public int A;
        public int B;
    }

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

        public NativeArray<BVHNode> BVHArray;

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
                    bool isEvenIteration = math.fmod(bitPosition, 2) == 0;

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

            public NativeArray<BVHNode> BVHArray;

            public void Execute()
            {
                int bvhLength = BVHArray.Length;
                int halfBVHArrayLength = bvhLength / 2;

                // Populate leaf nodes
                for (int i = 0; i < AABB.Length; i++)
                {
                    int colliderIndex = sortResultsArrayIsA[0] == 1 ? indexConverterA[i] : indexConverterB[i];
                    int bvhIndex = halfBVHArrayLength + i;

                    BVHNode bvhNode = BVHArray[bvhIndex];
                    bvhNode.aabb = AABB[colliderIndex];
                    bvhNode.AssociatedEntity = Entity[sortResultsArrayIsA[0] == 1 ? indexConverterA[i] : indexConverterB[i]];
                    bvhNode.RightmostLeafIndex = bvhIndex;
                    bvhNode.FirstChildIndex = -1;
                    bvhNode.IsValid = 1;
                    bvhNode .ColliderType = ColliderType[colliderIndex].Value;
                    BVHArray[bvhIndex] = bvhNode;
                }

                // Set all parent nodes to valid
                int indexOfLastValidLeafOnLevel = halfBVHArrayLength + AABB.Length - 1;
                while(indexOfLastValidLeafOnLevel > 0)
                {
                    indexOfLastValidLeafOnLevel = PhysicsMath.GetParentBVHNodeIndex(indexOfLastValidLeafOnLevel);

                    if (indexOfLastValidLeafOnLevel == 0)
                    {
                        BVHNode bvhNode = BVHArray[0];
                        bvhNode.IsValid = 1;
                        BVHArray[0] = bvhNode;
                    }
                    else
                    {
                        for (int i = PhysicsMath.GetNextLowestPowerOf2(indexOfLastValidLeafOnLevel) - 1; i <= indexOfLastValidLeafOnLevel; i++)
                        {
                            BVHNode bvhNode = BVHArray[i];
                            bvhNode.IsValid = 1;
                            BVHArray[i] = bvhNode;
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
                    int leafNodesCount = (bvhLength + 1) / 2;

                    BVHNode bvhNode = BVHArray[i];
                    bvhNode.FirstChildIndex = PhysicsMath.GetFirstChildBVHNodeIndex(i);
                    bvhNode.RightmostLeafIndex = (bvhLength - 1) - ((leafNodesCount / nbNodesOnLevel) * (nbNodesOnLevel - nodeOrderOnLevel));
                    bvhNode.aabb = PhysicsMath.GetEncompassingAABB(BVHArray[childNode1].aabb, BVHArray[childNode2].aabb);
                    BVHArray[i] = bvhNode;
                }
            }
        }

        [BurstCompile]
        struct BuildCollisionPairsParallel : IJobParallelFor
        {
            public int HalfBVHArrayLength;

            [ReadOnly]
            [NativeDisableParallelForRestriction]
            public NativeArray<BVHNode> BVHArray;

            [WriteOnly] 
            [NativeDisableParallelForRestriction]
            public NativeQueue<CollisionPair>.Concurrent CollisionPairsQueue;

            void QueryBVHNode(int comparedToNode, int leafNodeIndex)
            {
                if (BVHArray[comparedToNode].RightmostLeafIndex > leafNodeIndex &&
                    BVHArray[comparedToNode].IsValid > 0 &&
                    PhysicsMath.AABBToAABBOverlap(BVHArray[leafNodeIndex].aabb, BVHArray[comparedToNode].aabb))
                {
                    // leaf node
                    if (BVHArray[comparedToNode].FirstChildIndex < 0)
                    {
                        CollisionPair newPair = new CollisionPair
                        {
                            ColliderEntityA = BVHArray[leafNodeIndex].AssociatedEntity,
                            ColliderEntityB = BVHArray[comparedToNode].AssociatedEntity,
                        };
                        CollisionPairsQueue.Enqueue(newPair);
                    }
                    else
                    {
                        int firstChildIndex = BVHArray[comparedToNode].FirstChildIndex;
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
            public NativeArray<BVHNode> BVHArray;

            [WriteOnly]
            public NativeQueue<CollisionPair> CollisionPairsQueue;

            void QueryBVHNode(int comparedToNode, int leafNodeIndex)
            {
                if (BVHArray[comparedToNode].RightmostLeafIndex > leafNodeIndex &&
                    BVHArray[comparedToNode].IsValid > 0 &&
                    PhysicsMath.AABBToAABBOverlap(BVHArray[leafNodeIndex].aabb, BVHArray[comparedToNode].aabb))
                {
                    // leaf node
                    if (BVHArray[comparedToNode].FirstChildIndex < 0)
                    {
                        CollisionPair newPair = new CollisionPair
                        {
                            ColliderEntityA = BVHArray[leafNodeIndex].AssociatedEntity,
                            ColliderEntityB = BVHArray[comparedToNode].AssociatedEntity,
                        };
                        CollisionPairsQueue.Enqueue(newPair);
                    }
                    else
                    {
                        int firstChildIndex = BVHArray[comparedToNode].FirstChildIndex;
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

        [BurstCompile]
        struct BuildCollisionPairsGroupStack : IJob
        {
            [ReadOnly]
            public NativeArray<BVHNode> BVHArray;

            public NativeQueue<NodePair> NodePairsStack;

            [WriteOnly]
            public NativeQueue<CollisionPair> CollisionPairsQueue;

            public void CompareNodes(int A, int B)
            {
                if (A >= 0 &&
                    B >= 0 &&
                    BVHArray[A].IsValid > 0 &&
                    BVHArray[B].IsValid > 0)
                {
                    if (PhysicsMath.AABBToAABBOverlap(BVHArray[A].aabb, BVHArray[B].aabb))
                    {
                        // if leaf node
                        if (BVHArray[A].FirstChildIndex < 0)
                        {
                            CollisionPair newPair = new CollisionPair
                            {
                                ColliderEntityA = BVHArray[A].AssociatedEntity,
                                ColliderEntityB = BVHArray[B].AssociatedEntity,
                            };
                            CollisionPairsQueue.Enqueue(newPair);
                        }
                        else
                        {
                            NodePairsStack.Enqueue(new NodePair() { A = A, B = B });
                        }
                    }
                }
            }

            public void Execute()
            {
                // Add first node pair
                NodePairsStack.Enqueue(new NodePair() { A = 1, B = 2 });

                // Dequeue stack
                while (NodePairsStack.Count > 0)
                {
                    NodePair pair = NodePairsStack.Dequeue();

                    int childA1Index = BVHArray[pair.A].FirstChildIndex;
                    int childA2Index = childA1Index + 1;
                    int childB1Index = BVHArray[pair.B].FirstChildIndex;
                    int childB2Index = childB1Index + 1;

                    // Compare child nodes
                    CompareNodes(childA1Index, childA2Index);
                    CompareNodes(childA1Index, childB1Index);
                    CompareNodes(childA1Index, childB2Index);
                    CompareNodes(childA2Index, childB1Index);
                    CompareNodes(childA2Index, childB2Index);
                    CompareNodes(childB1Index, childB2Index);
                }
            }
        }

        [BurstCompile]
        struct BuildCollisionPairsGroupRecursive : IJob
        {
            [ReadOnly]
            public NativeArray<BVHNode> BVHArray;

            [WriteOnly]
            public NativeQueue<CollisionPair> CollisionPairsQueue;

            public void CompareNodes(int A, int B)
            {
                int childA1Index = BVHArray[A].FirstChildIndex;
                int childA2Index = childA1Index + 1;
                int childB1Index = BVHArray[B].FirstChildIndex;
                int childB2Index = childB1Index + 1;

                // always compare own children
                if (BVHArray[A].IsValid > 0 &&
                    BVHArray[A].FirstChildIndex >= 0)
                {
                    CompareNodes(childA1Index, childA2Index);
                }
                if (BVHArray[B].IsValid > 0 &&
                    BVHArray[B].FirstChildIndex >= 0)
                {
                    CompareNodes(childB1Index, childB2Index);
                }

                if (BVHArray[A].IsValid > 0 &&
                    BVHArray[B].IsValid > 0)
                {
                    if (PhysicsMath.AABBToAABBOverlap(BVHArray[A].aabb, BVHArray[B].aabb))
                    {
                        // if leaf node, add as collision pair
                        if (BVHArray[A].FirstChildIndex < 0)
                        {
                            CollisionPair newPair = new CollisionPair
                            {
                                ColliderEntityA = BVHArray[A].AssociatedEntity,
                                ColliderEntityB = BVHArray[B].AssociatedEntity,
                            };
                            CollisionPairsQueue.Enqueue(newPair); 
                        }
                        // if not, compare their children
                        else
                        {
                            // Compare child nodes
                            CompareNodes(childA1Index, childB1Index);
                            CompareNodes(childA1Index, childB2Index);
                            CompareNodes(childA2Index, childB1Index);
                            CompareNodes(childA2Index, childB2Index);
                        }
                    }
                }
            }

            public void Execute()
            {
                CompareNodes(1, 2);
            }
        }

        protected override void OnCreateManager()
        {
            BVHArray = new NativeArray<BVHNode>(10, Allocator.TempJob);

            SphereSphereCollisionPairsQueue = new NativeQueue<CollisionPair>(Allocator.Persistent);
        }

        protected override void OnDestroyManager()
        {   
            SphereSphereCollisionPairsQueue.Dispose();
            PhysicsSystem.SphereSphereCollisionPairsArray.Dispose();

            BVHArray.Dispose();

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
            int requiredBVHLength = (PhysicsMath.GetNextHighestPowerOf2(math.ceilpow2(collidersCount) + 1)) - 1;

            BVHArray.Dispose();
            BVHArray = new NativeArray<BVHNode>(requiredBVHLength, Allocator.TempJob);

            var constructBVH = new ConstructBVH()
            {
                Entity = _colliderGroup.Entity,
                AABB = _colliderGroup.AABB,
                ColliderType = _colliderGroup.ColliderType,
                BVHArray = BVHArray,
                indexConverterA = indexConverterA,
                indexConverterB = indexConverterB,
                sortResultsArrayIsA = sortResultsArrayIsA,
            }.Schedule(sortMortonCodes);

            // Draw bvh second-to-last groups
            if (PhysicsSystem.Settings.ShowBVHBounds)
            {
                constructBVH.Complete();
                for (int i = BVHArray.Length / 4; i < BVHArray.Length / 2; i++)
                {
                    if (BVHArray[i].IsValid > 0)
                    {
                        DebugUtils.DrawAABB(BVHArray[i].aabb, UnityEngine.Random.ColorHSV());
                    }
                }
            }

            // Init pairs queue
            SphereSphereCollisionPairsQueue.Clear();

            JobHandle buildCollisionPairs = new JobHandle();

            //Build Pairs job
            {
                BuildCollisionPairsParallel buildCollisionPairsJob = new BuildCollisionPairsParallel()
                {
                    HalfBVHArrayLength = BVHArray.Length / 2,
                    BVHArray = BVHArray,
                    CollisionPairsQueue = SphereSphereCollisionPairsQueue.ToConcurrent(),
                };
                buildCollisionPairs = buildCollisionPairsJob.Schedule(_colliderGroup.AABB.Length, PhysicsSystem.Settings.BroadphaseSystemBatchCount, constructBVH);
            }

            //{
            //    BuildCollisionPairsSingle buildCollisionPairsJob = new BuildCollisionPairsSingle()
            //    {
            //        StartIndex = BVHArray.Length / 2,
            //        IndexCount = _colliderGroup.AABB.Length,
            //        BVHArray = BVHArray,
            //        CollisionPairsQueue = SphereSphereCollisionPairsQueue,
            //    };
            //    buildCollisionPairs = buildCollisionPairsJob.Schedule(constructBVH);
            //}

            //{
            //    NativeQueue<NodePair> NodeStack = new NativeQueue<NodePair>(Allocator.TempJob);
            //    BuildCollisionPairsGroupStack buildCollisionPairsJob = new BuildCollisionPairsGroupStack()
            //    {
            //        BVHArray = BVHArray,
            //        NodePairsStack = NodeStack,
            //        CollisionPairsQueue = SphereSphereCollisionPairsQueue,
            //    };
            //    buildCollisionPairs = buildCollisionPairsJob.Schedule(constructBVH);

            //    buildCollisionPairs.Complete();
            //    NodeStack.Dispose();
            //}

            //{
            //    BuildCollisionPairsGroupRecursive buildCollisionPairsJob = new BuildCollisionPairsGroupRecursive()
            //    {
            //        BVHArray = BVHArray,
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
