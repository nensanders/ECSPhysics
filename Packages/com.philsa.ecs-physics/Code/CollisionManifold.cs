using Unity.Entities;
using Unity.Mathematics;

public struct CollisionManifold
{
    public Entity ColliderEntityA;
    public Entity ColliderEntityB;

    public Entity RigidBodyEntityA;
    public Entity RigidBodyEntityB;

    public double3 ContactPointA;
    public double3 ContactPointB;

    public double3 CollisionNormalAToB;
    public double OverlapDistance;
}
