using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using UnityEngine;

namespace PhysicsEngine
{
    public struct BVHNode
    {
        public AABB aabb;
        public Entity AssociatedEntity;
        public int RightmostLeafIndex;
        public int FirstChildIndex;
        public byte IsValid;
        public byte ColliderType;
    }
}