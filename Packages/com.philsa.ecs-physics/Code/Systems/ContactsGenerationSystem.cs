using Unity.Entities;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;

namespace PhysicsEngine
{
    [DisableAutoCreation]
    public class ContactsGenerationSystem : JobComponentSystem
    {
        [Inject] ComponentDataFromEntity<RigidBody> RigidBodyFromEntity;
        [Inject] ComponentDataFromEntity<Collider> ColliderFromEntity;
        [Inject] ComponentDataFromEntity<PositionD> ColliderPositionFromEntity;
        [Inject] ComponentDataFromEntity<ColliderPhysicsProperties> ColliderPhysicsPropertiesFromEntity;
        [Inject] ComponentDataFromEntity<SphereCollider> SphereColliderFromEntity;
        [Inject] ComponentDataFromEntity<Velocity> VelocityFromEntity;

        [Inject] PhysicsSystem PhysicsSystem;

        private NativeQueue<CollisionManifold> CollisionManifoldsQueue;

        [BurstCompile]
        struct ComputeSphereSphereContacts : IJobParallelFor
        {
            [ReadOnly] public NativeArray<CollisionPair> CollisionPairsArray;

            public NativeQueue<CollisionManifold>.Concurrent CollisionManifoldsQueue;

            [NativeDisableParallelForRestriction]
            [ReadOnly] public ComponentDataFromEntity<RigidBody> RigidBodyFromEntity;
            [NativeDisableParallelForRestriction]
            [ReadOnly] public ComponentDataFromEntity<Collider> ColliderFromEntity;
            [NativeDisableParallelForRestriction]
            [ReadOnly] public ComponentDataFromEntity<PositionD> ColliderPositionFromEntity;
            [NativeDisableParallelForRestriction]
            [ReadOnly] public ComponentDataFromEntity<ColliderPhysicsProperties> ColliderPhysicsPropertiesFromEntity;
            [NativeDisableParallelForRestriction]
            [ReadOnly] public ComponentDataFromEntity<SphereCollider> SphereColliderFromEntity;
            [NativeDisableParallelForRestriction]
            [ReadOnly] public ComponentDataFromEntity<Velocity> VelocityFromEntity;

            public void Execute(int index)
            {
                Entity collAEntity = CollisionPairsArray[index].ColliderEntityA;
                Entity collBEntity = CollisionPairsArray[index].ColliderEntityB;
                Collider collA = ColliderFromEntity[collAEntity];
                Collider collB = ColliderFromEntity[collBEntity];
                Entity rigidBodyAEntity = collA.RigidBodyEntity;
                Entity rigidBodyBEntity = collB.RigidBodyEntity;

                double3 aPos = ColliderPositionFromEntity[collAEntity].Value;
                double3 bPos = ColliderPositionFromEntity[collBEntity].Value;
                double3 fromAToBVector = bPos - aPos;
                if(math.lengthsq(fromAToBVector) < double.Epsilon)
                {
                    fromAToBVector = new double3(0.0, 1.0, 0.0);
                }

                double overlapDistance = -math.length(fromAToBVector) + SphereColliderFromEntity[collAEntity].Radius + SphereColliderFromEntity[collBEntity].Radius;

                if (overlapDistance > 0.0)
                {
                    double3 relativeVelocity = VelocityFromEntity[collB.RigidBodyEntity].Value - VelocityFromEntity[collA.RigidBodyEntity].Value;
                    double3 collisionNormalAToB = math.normalize(fromAToBVector);

                    bool aIsKinematic = RigidBodyFromEntity[collA.RigidBodyEntity].IsKinematic > 0;
                    bool bIsKinematic = RigidBodyFromEntity[collB.RigidBodyEntity].IsKinematic > 0;

                    if (!(aIsKinematic && bIsKinematic))
                    {
                        CollisionManifold manifold = new CollisionManifold()
                        {
                            ColliderEntityA = collAEntity,
                            ColliderEntityB = collBEntity,

                            RigidBodyEntityA = rigidBodyAEntity,
                            RigidBodyEntityB = rigidBodyBEntity,

                            ContactPointA = aPos + (collisionNormalAToB * SphereColliderFromEntity[collAEntity].Radius),
                            ContactPointB = bPos + (-collisionNormalAToB * SphereColliderFromEntity[collBEntity].Radius),

                            CollisionNormalAToB = collisionNormalAToB,
                            OverlapDistance = overlapDistance,
                        };

                        CollisionManifoldsQueue.Enqueue(manifold);
                    }
                }
            }
        }

        protected override void OnCreateManager()
        {

            CollisionManifoldsQueue = new NativeQueue<CollisionManifold>(Allocator.Persistent);
        }

        protected override void OnDestroyManager()
        {
            CollisionManifoldsQueue.Dispose();
            PhysicsSystem.CollisionManifoldsArray.Dispose();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            CollisionManifoldsQueue.Clear();

            var computeSphereSphereContactsJob = new ComputeSphereSphereContacts
            {
                CollisionPairsArray = PhysicsSystem.SphereSphereCollisionPairsArray,
                ColliderFromEntity = ColliderFromEntity,
                RigidBodyFromEntity = RigidBodyFromEntity,
                ColliderPositionFromEntity = ColliderPositionFromEntity,
                ColliderPhysicsPropertiesFromEntity = ColliderPhysicsPropertiesFromEntity,
                SphereColliderFromEntity = SphereColliderFromEntity,
                VelocityFromEntity = VelocityFromEntity,
                CollisionManifoldsQueue = CollisionManifoldsQueue.ToConcurrent(),
            };
            var computeSphereSphereContacts = computeSphereSphereContactsJob.Schedule(PhysicsSystem.SphereSphereCollisionPairsArray.Length, PhysicsSystem.Settings.ContactsGenerationSystemBatchCount, inputDeps);

            computeSphereSphereContacts.Complete();
            if (PhysicsSystem.CollisionManifoldsArray.IsCreated)
            {
                PhysicsSystem.CollisionManifoldsArray.Dispose();
            }
            PhysicsSystem.CollisionManifoldsArray = new NativeArray<CollisionManifold>(CollisionManifoldsQueue.Count, Allocator.TempJob);
            DequeueIntoArray<CollisionManifold> dequeueManifoldsJob = new DequeueIntoArray<CollisionManifold>()
            {
                InputQueue = CollisionManifoldsQueue,
                OutputArray = PhysicsSystem.CollisionManifoldsArray,
            };
            JobHandle dequeueCollisionManifolds = dequeueManifoldsJob.Schedule(computeSphereSphereContacts);

            // Need to complete jobs here because counter will be read in the next system
            dequeueCollisionManifolds.Complete();

            return dequeueCollisionManifolds;
        }
    }
}
