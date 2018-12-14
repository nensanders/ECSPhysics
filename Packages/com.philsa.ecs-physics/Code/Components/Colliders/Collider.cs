using Unity.Entities;

namespace PhysicsEngine
{
    public struct Collider : IComponentData
    {
        public Entity RigidBodyEntity;
    }
}