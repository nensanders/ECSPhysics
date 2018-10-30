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

public struct RigidbodySolvingGroup
{
    public NativeList<Entity> RigidbodyEntities;
    public NativeList<float3> GroupVelocityMatrix;
    public NativeList<float3> Group ;
}
