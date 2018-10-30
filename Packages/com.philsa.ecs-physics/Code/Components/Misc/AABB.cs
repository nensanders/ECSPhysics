using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct AABB : IComponentData
    {
        public float3 Min;
        public float3 Max;
    }
}