using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct RigidBody : IComponentData
    {
        public byte IsKinematic;
        public float InverseMass;
        public float3 CenterOfMass;
        public float3x3 MomentOfInertia;
    }
}