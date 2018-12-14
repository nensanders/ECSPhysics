using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct BoxCollider : IComponentData
    {
        public double3 HalfSize;
    }
}