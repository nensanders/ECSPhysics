using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct CapsuleCollider : IComponentData
    {
        public float Radius;
        public float Height;
    }
}