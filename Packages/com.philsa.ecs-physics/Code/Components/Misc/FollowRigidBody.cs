using Unity.Entities;

namespace PhysicsEngine
{
    public struct FollowRigidBody : IComponentData
    {
        //public bool Interpolate;
        public Entity RigidBodyEntity;
    }
}