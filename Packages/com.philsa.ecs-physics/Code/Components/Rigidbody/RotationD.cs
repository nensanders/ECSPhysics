using System;
using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    [Serializable]
    public struct RotationD : IComponentData
    {
        public double3 Value;
    }
}