using Unity.Entities;

namespace PhysicsEngine
{
    public struct ColliderType : IComponentData
    {
        public byte Value;
        /*
         * 0 = UNDEFINED
         * 1 = Sphere
         * 2 = Box
         * 3 = Capsule
         * 4 = Convex Mesh
         * 5 = Generic Mesh
        */
    }
}