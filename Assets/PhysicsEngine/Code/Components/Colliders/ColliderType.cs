using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

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