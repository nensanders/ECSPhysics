using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct AABB : IComponentData
    {
        public double3 Min;
        public double3 Max;
    }
}