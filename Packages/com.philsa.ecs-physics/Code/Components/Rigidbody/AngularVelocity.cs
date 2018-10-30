using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct AngularVelocity : IComponentData
    {
        public float3 Value;
    }
}