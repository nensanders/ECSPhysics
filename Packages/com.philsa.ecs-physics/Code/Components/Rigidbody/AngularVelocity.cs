using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct AngularVelocity : IComponentData
    {
        public double3 Value;
    }
}