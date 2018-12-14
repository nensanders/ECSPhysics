using Unity.Entities;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;

public struct Constraint
{
    public double12 ConstraintJacobianMatrix;
    public Entity RigidbodyA;
    public Entity RigidbodyB;
    public double BaumgarteDepth;
}

public struct double12
{
    public double3 aa;
    public double3 ab;
    public double3 ba;
    public double3 bb;

    public static double12 operator -(double12 val)
    {
        return (val * -1);
    }

    public static double12 operator *(double12 left, double right)
    {
        return new double12
        {
            aa = new double3(left.aa.x * right, left.aa.y * right, left.aa.z * right),
            ab = new double3(left.ab.x * right, left.ab.y * right, left.ab.z * right),
            ba = new double3(left.ba.x * right, left.ba.y * right, left.ba.z * right),
            bb = new double3(left.bb.x * right, left.bb.y * right, left.bb.z * right),
        };
    }

    public static double operator *(double12 left, double12 right)
    {
        return (left.aa.x * right.aa.x) +
                (left.aa.y * right.aa.y) +
                (left.aa.z * right.aa.z) +
                (left.ab.x * right.ab.x) +
                (left.ab.y * right.ab.y) +
                (left.ab.z * right.ab.z) +
                (left.ba.x * right.ba.x) +
                (left.ba.y * right.ba.y) +
                (left.ba.z * right.ba.z) +
                (left.bb.x * right.bb.x) +
                (left.bb.y * right.bb.y) +
                (left.bb.z * right.bb.z);
    }

    public static double12 operator *(double12 left, double12x12Mass right)
    {
        return new double12
        {
            aa = new double3
            {
                x = (left.aa.x * right.massA.x),
                y = (left.aa.y * right.massA.y),
                z = (left.aa.z * right.massA.z),
            },
            ab = new double3
            {
                x = (left.ab.x * right.inertiaTensorA.c0.x) + (left.ab.y * right.inertiaTensorA.c0.y) + (left.ab.z * right.inertiaTensorA.c0.z),
                y = (left.ab.x * right.inertiaTensorA.c1.x) + (left.ab.y * right.inertiaTensorA.c1.y) + (left.ab.z * right.inertiaTensorA.c1.z),
                z = (left.ab.x * right.inertiaTensorA.c2.x) + (left.ab.y * right.inertiaTensorA.c2.y) + (left.ab.z * right.inertiaTensorA.c2.z),
            },
            ba = new double3
            {
                x = (left.ba.x * right.massB.x),
                y = (left.ba.y * right.massB.y),
                z = (left.ba.z * right.massB.z),
            },
            bb = new double3
            {
                x = (left.bb.x * right.inertiaTensorB.c0.x) + (left.bb.y * right.inertiaTensorB.c0.y) + (left.bb.z * right.inertiaTensorB.c0.z),
                y = (left.bb.x * right.inertiaTensorB.c1.x) + (left.bb.y * right.inertiaTensorB.c1.y) + (left.bb.z * right.inertiaTensorB.c1.z),
                z = (left.bb.x * right.inertiaTensorB.c2.x) + (left.bb.y * right.inertiaTensorB.c2.y) + (left.bb.z * right.inertiaTensorB.c2.z),
            }
        };
    }
}

public struct double12x12Mass
{
    public double3 massA;
    public double3x3 inertiaTensorA;
    public double3 massB;
    public double3x3 inertiaTensorB;

    public static double12 operator *(double12x12Mass left, double12 right)
    {
        return new double12
        {
            aa = new double3
            {
                x = (left.massA.x * right.aa.x),
                y = (left.massA.y * right.aa.y),
                z = (left.massA.z * right.aa.z),
            },
            ab = new double3
            {
                x = (left.inertiaTensorA.c0.x * right.ab.x) + (left.inertiaTensorA.c1.x * right.ab.y) + (left.inertiaTensorA.c2.x * right.ab.z),
                y = (left.inertiaTensorA.c0.y * right.ab.x) + (left.inertiaTensorA.c1.y * right.ab.y) + (left.inertiaTensorA.c2.y * right.ab.z),
                z = (left.inertiaTensorA.c0.z * right.ab.x) + (left.inertiaTensorA.c1.z * right.ab.y) + (left.inertiaTensorA.c2.z * right.ab.z),
            },
            ba = new double3
            {
                x = (left.massB.x * right.ba.x),
                y = (left.massB.y * right.ba.y),
                z = (left.massB.z * right.ba.z),
            },
            bb = new double3
            {
                x = (left.inertiaTensorB.c0.x * right.bb.x) + (left.inertiaTensorB.c1.x * right.bb.y) + (left.inertiaTensorB.c2.x * right.bb.z),
                y = (left.inertiaTensorB.c0.y * right.bb.x) + (left.inertiaTensorB.c1.y * right.bb.y) + (left.inertiaTensorB.c2.y * right.bb.z),
                z = (left.inertiaTensorB.c0.z * right.bb.x) + (left.inertiaTensorB.c1.z * right.bb.y) + (left.inertiaTensorB.c2.z * right.bb.z),
            }
        };
    }
}

namespace PhysicsEngine
{
    [DisableAutoCreation]
    public class ConstraintSolverSystem : JobComponentSystem
    {
        [Inject] ComponentDataFromEntity<RigidBody> RigidBodyFromEntity;
        [Inject] ComponentDataFromEntity<Velocity> VelocityFromEntity;
        [Inject] ComponentDataFromEntity<AngularVelocity> AngularVelocityFromEntity;

        [Inject] PhysicsSystem PhysicsSystem;

        private NativeQueue<Constraint> ConstraintsQueue;

        [BurstCompile]
        struct BuildSystemConstraints : IJob
        {
            [WriteOnly] public NativeQueue<Constraint> ConstraintsQueue;
            [ReadOnly] public NativeArray<CollisionManifold> CollisionManifoldsArray;
            [ReadOnly] public ComponentDataFromEntity<RigidBody> RigidBodyFromEntity;

            public void Execute()
            {
                // Contact constraints
                for (int i = 0; i < CollisionManifoldsArray.Length; i++)
                {
                    double3 comAToContactPoint = (CollisionManifoldsArray[i].ContactPointA - RigidBodyFromEntity[CollisionManifoldsArray[i].RigidBodyEntityA].CenterOfMass);
                    double3 comBToContactPoint = (CollisionManifoldsArray[i].ContactPointB - RigidBodyFromEntity[CollisionManifoldsArray[i].RigidBodyEntityB].CenterOfMass);

                    ConstraintsQueue.Enqueue(
                        new Constraint
                        {
                            ConstraintJacobianMatrix = new double12
                            {
                                aa = -CollisionManifoldsArray[i].CollisionNormalAToB,
                                ab = -math.cross(comAToContactPoint, CollisionManifoldsArray[i].CollisionNormalAToB),
                                ba = CollisionManifoldsArray[i].CollisionNormalAToB,
                                bb = math.cross(comBToContactPoint, CollisionManifoldsArray[i].CollisionNormalAToB),
                            },
                            RigidbodyA = CollisionManifoldsArray[i].RigidBodyEntityA,
                            RigidbodyB = CollisionManifoldsArray[i].RigidBodyEntityB,
                            BaumgarteDepth = CollisionManifoldsArray[i].OverlapDistance,
                        }
                    );
                }
            }
        }

        [BurstCompile]
        struct SolveConstraintsSequentialImpulses : IJob
        {
            public int IterationsCount;
            public double DeltaTime;
            public double BaumgarteBias;

            [ReadOnly] public ComponentDataFromEntity<RigidBody> RigidBodyFromEntity;
            public ComponentDataFromEntity<Velocity> VelocityFromEntity;
            public ComponentDataFromEntity<AngularVelocity> AngularVelocityFromEntity;

            [ReadOnly] public NativeArray<Constraint> ConstraintsArray;

            public void Execute()
            {
                for (int iteration = 0; iteration < IterationsCount; iteration++)
                {
                    // For each constraint....
                    for (int i = 0; i < ConstraintsArray.Length; i++)
                    {
                        Velocity velA = VelocityFromEntity[ConstraintsArray[i].RigidbodyA];
                        Velocity velB = VelocityFromEntity[ConstraintsArray[i].RigidbodyB];
                        AngularVelocity angVelA = AngularVelocityFromEntity[ConstraintsArray[i].RigidbodyA];
                        AngularVelocity angVelB = AngularVelocityFromEntity[ConstraintsArray[i].RigidbodyB];
                        double invMassA = RigidBodyFromEntity[ConstraintsArray[i].RigidbodyA].InverseMass;
                        double invMassB = RigidBodyFromEntity[ConstraintsArray[i].RigidbodyB].InverseMass;
                        //float3x3 invMomentA = math.inverse(RigidBodyFromEntity[ConstraintsArray[i].RigidbodyA].MomentOfInertia);
                        //float3x3 invMomentB = math.inverse(RigidBodyFromEntity[ConstraintsArray[i].RigidbodyB].MomentOfInertia);
                        double3x3 invMomentA = new double3x3();
                        double3x3 invMomentB = new double3x3();

                        // Build matrices for this constraint
                        double12 constraintJacobianMatrix = ConstraintsArray[i].ConstraintJacobianMatrix;

                        double12 velocityMatrix = new double12
                        {
                            aa = velA.Value,
                            ab = angVelA.Value,
                            ba = velB.Value,
                            bb = angVelB.Value,
                        };

                        double12x12Mass massInverseMatrix = new double12x12Mass
                        {
                            massA = new double3(invMassA, invMassA, invMassA),
                            inertiaTensorA = invMomentA,
                            massB = new double3(invMassB, invMassB, invMassB),
                            inertiaTensorB = invMomentB,
                        };

                        // Calculate lambda
                        double lambda = 0f;
                        double baumgarte = (BaumgarteBias / DeltaTime) * ConstraintsArray[i].BaumgarteDepth;
                        double numerator = (-(constraintJacobianMatrix * velocityMatrix) + baumgarte);
                        double demunerator = constraintJacobianMatrix * massInverseMatrix * constraintJacobianMatrix;
                        lambda = numerator / demunerator;

                        // Calculate and apply resulting forces
                        double12 constraintImpulses = constraintJacobianMatrix * lambda;
                        double12 constraintVelocityChanges = massInverseMatrix * constraintImpulses;

                        velA.Value += constraintVelocityChanges.aa;
                        angVelA.Value += constraintVelocityChanges.ab;
                        velB.Value += constraintVelocityChanges.ba;
                        angVelB.Value += constraintVelocityChanges.bb;

                        VelocityFromEntity[ConstraintsArray[i].RigidbodyA] = velA;
                        //AngularVelocityFromEntity[ConstraintsArray[i].RigidbodyA] = angVelA;
                        VelocityFromEntity[ConstraintsArray[i].RigidbodyB] = velB;
                        //AngularVelocityFromEntity[ConstraintsArray[i].RigidbodyB] = angVelB;
                    }
                }
            }
        }

        protected override void OnCreateManager()
        {
            ConstraintsQueue = new NativeQueue<Constraint>(Allocator.Persistent);
        }

        protected override void OnDestroyManager()
        {
            ConstraintsQueue.Dispose();
            PhysicsSystem.ConstraintsArray.Dispose();
        }

        protected override JobHandle OnUpdate(JobHandle inputDeps)
        {
            if (!PhysicsSystem.CollisionManifoldsArray.IsCreated)
            {
                return inputDeps;
            }

            inputDeps.Complete();


            var buildConstraints = new BuildSystemConstraints
            {
                ConstraintsQueue = ConstraintsQueue,
                CollisionManifoldsArray = PhysicsSystem.CollisionManifoldsArray,
                RigidBodyFromEntity = RigidBodyFromEntity,
            }.Schedule(inputDeps);

            buildConstraints.Complete();

            if (PhysicsSystem.ConstraintsArray.IsCreated)
            {
                PhysicsSystem.ConstraintsArray.Dispose();
            }
            PhysicsSystem.ConstraintsArray = new NativeArray<Constraint>(ConstraintsQueue.Count, Allocator.TempJob);
            DequeueIntoArray<Constraint> dequeueManifoldsJob = new DequeueIntoArray<Constraint>()
            {
                InputQueue = ConstraintsQueue,
                OutputArray = PhysicsSystem.ConstraintsArray,
            };
            JobHandle dequeueCollisionManifolds = dequeueManifoldsJob.Schedule(buildConstraints);

            // Need to complete jobs here because counter will be read in the next system
            dequeueCollisionManifolds.Complete();

            var solveConstraints = new SolveConstraintsSequentialImpulses
            {
                IterationsCount = PhysicsSystem.Settings.ConstraintSolverIterations,
                DeltaTime = PhysicsSystem.DeltaTime,
                BaumgarteBias = PhysicsSystem.Settings.BaumgarteBias,
                RigidBodyFromEntity = RigidBodyFromEntity,
                VelocityFromEntity = VelocityFromEntity,
                AngularVelocityFromEntity = AngularVelocityFromEntity,
                ConstraintsArray = PhysicsSystem.ConstraintsArray,
            }.Schedule(buildConstraints);

            solveConstraints.Complete();

            return solveConstraints;
        }
    }
}
