using Unity.Entities;

namespace PhysicsEngine
{
    public struct ColliderPhysicsProperties : IComponentData
    {
        public double Friction;
        public double CoefficientOfRestitution;
    }
}