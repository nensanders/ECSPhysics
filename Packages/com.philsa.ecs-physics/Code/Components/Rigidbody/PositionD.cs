using System;
using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    [Serializable]
    public struct PositionD : IComponentData
    {
        public double3 Value;
    }
}