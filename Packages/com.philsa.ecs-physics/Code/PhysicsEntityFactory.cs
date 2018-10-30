using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

public static class PhysicsEntityFactory
{
    public static EntityArchetype CreateKineticRigidbodyArchetype(EntityManager entityManager)
    {
        return entityManager.CreateArchetype(
            typeof(PhysicsEngine.RigidBody),
            typeof(Unity.Transforms.Position),
            typeof(Unity.Transforms.Rotation),
            typeof(PhysicsEngine.Velocity),
            typeof(PhysicsEngine.AngularVelocity),
            typeof(PhysicsEngine.LinearDamping),
            typeof(PhysicsEngine.AngularDamping),
            typeof(PhysicsEngine.UniformGravity)
            );
    }

    public static EntityArchetype CreateKinematicRigidbodyArchetype(EntityManager entityManager)
    {
        return entityManager.CreateArchetype(
           typeof(PhysicsEngine.RigidBody),
           typeof(Unity.Transforms.Position),
           typeof(Unity.Transforms.Rotation),
           typeof(PhysicsEngine.Velocity),
           typeof(PhysicsEngine.AngularVelocity)
           );
    }

    public static EntityArchetype CreateSphereColliderArchetype(EntityManager entityManager)
    {
        return entityManager.CreateArchetype(
            typeof(PhysicsEngine.Collider),
            typeof(PhysicsEngine.ColliderType),
            typeof(PhysicsEngine.SphereCollider),
            typeof(PhysicsEngine.AABB),
            typeof(Unity.Transforms.Position),
            typeof(Unity.Transforms.Rotation),
            typeof(PhysicsEngine.ColliderPhysicsProperties)
            );
    }

    public static EntityArchetype CreateBoxColliderArchetype(EntityManager entityManager)
    {
        return entityManager.CreateArchetype(
            typeof(PhysicsEngine.Collider),
            typeof(PhysicsEngine.ColliderType),
            typeof(PhysicsEngine.BoxCollider),
            typeof(PhysicsEngine.AABB),
            typeof(Unity.Transforms.Position),
            typeof(Unity.Transforms.Rotation),
            typeof(PhysicsEngine.ColliderPhysicsProperties)
            );
    }
}