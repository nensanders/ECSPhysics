using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct ColliderPhysicsProperties : IComponentData
    {
        public float Friction;
        public float CoefficientOfRestitution;
    }
}