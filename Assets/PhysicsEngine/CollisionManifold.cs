using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Collections;

public struct CollisionManifold
{
    public Entity ColliderEntityA;
    public Entity ColliderEntityB;

    public Entity RigidBodyEntityA;
    public Entity RigidBodyEntityB;

    public float3 ContactPointA;
    public float3 ContactPointB;

    public float3 CollisionNormalAToB;
    public float OverlapDistance;
}
