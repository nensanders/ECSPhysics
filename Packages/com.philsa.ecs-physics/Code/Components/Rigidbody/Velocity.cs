using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct Velocity : IComponentData
    {
        public double3 Value;
    }
}