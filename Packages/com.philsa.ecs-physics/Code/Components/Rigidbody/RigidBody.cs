using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct RigidBody : IComponentData
    {
        public byte IsKinematic;
        public double InverseMass;
        public double3 CenterOfMass;
        public double3x3 MomentOfInertia;
    }
}