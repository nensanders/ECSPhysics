using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;

namespace PhysicsEngine
{
    public struct FollowRigidBody : IComponentData
    {
        //public bool Interpolate;
        public Entity RigidBodyEntity;
    }
}