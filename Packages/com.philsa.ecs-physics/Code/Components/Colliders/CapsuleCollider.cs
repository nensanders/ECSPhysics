using Unity.Entities;

namespace PhysicsEngine
{
    public struct CapsuleCollider : IComponentData
    {
        public double Radius;
        public double Height;
    }
}