using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Unity.Rendering;
using Unity.Transforms;
using PhysicsEngine;

public class TestSceneInitializer : MonoBehaviour
{
    [Header("General")]
    public bool SpawnGround = false;
    public int SpawnCount = 50;
    public float SpawnDistance = 10;
    public float SpawnVelocityRange = 5;
    public Transform GroundSpheresRoot;

    [Header("Renderers")]
    public MeshInstanceRendererComponent SphereSmallRendererComponent;
    public MeshInstanceRendererComponent SphereBigRendererComponent;
    //public MeshInstanceRendererComponent BoxSmallRendererComponent;

    public static EntityArchetype KineticRigidBodyArchetype;
    public static EntityArchetype KinematicRigidBodyArchetype;
    public static EntityArchetype SphereColliderArchetype;
    //public static EntityArchetype BoxColliderArchetype;

    public static EntityArchetype SimpleRendererArchetype;

    void Start()
    {
        EntityManager entityManager = World.Active.GetOrCreateManager<EntityManager>();

        // Create archetypes
        KineticRigidBodyArchetype = PhysicsEntityFactory.CreateKineticRigidbodyArchetype(entityManager);
        KinematicRigidBodyArchetype = PhysicsEntityFactory.CreateKinematicRigidbodyArchetype(entityManager);
        SphereColliderArchetype = PhysicsEntityFactory.CreateSphereColliderArchetype(entityManager);
        //BoxColliderArchetype = PhysicsEntityFactory.CreateBoxColliderArchetype(entityManager);

        SimpleRendererArchetype = entityManager.CreateArchetype(
            typeof(Unity.Transforms.Position),
            typeof(Unity.Transforms.Rotation),
            typeof(MeshInstanceRenderer),
            typeof(PhysicsEngine.FollowRigidBody)
            );

        // Spawn kinetic objects random
        for (int i = 0; i < SpawnCount; i++)
        {
            float3 startPos = new float3(
                transform.position.x + UnityEngine.Random.Range(-SpawnDistance, SpawnDistance),
                transform.position.y + UnityEngine.Random.Range(-SpawnDistance, SpawnDistance),
                transform.position.z + UnityEngine.Random.Range(-SpawnDistance, SpawnDistance)
                );
            float3 startVel = new float3(
                startPos.x >= 0f ? -SpawnVelocityRange : SpawnVelocityRange,
                0,
                0
                );

            if (i % 2 == 0)
            {
                //SpawnBoxSmall(startPos, startVel);
                SpawnSphereSmall(startPos, startVel);
            }
            else
            {
                SpawnSphereSmall(startPos, startVel);
            }
        }

        // spawn ground spheres
        if (SpawnGround)
        {
            foreach (Transform t in GroundSpheresRoot)
            {
                SpawnSphereBig(t.position, new float3(0, 0, 0));
            }
        }
    }

    public void SpawnSphereSmall(float3 startPos, float3 startVel)
    {
        EntityManager entityManager = World.Active.GetOrCreateManager<EntityManager>();

        Entity rigidBody = entityManager.CreateEntity(KineticRigidBodyArchetype);
        entityManager.SetComponentData(rigidBody, new RigidBody { IsKinematic = 0, InverseMass = 1f, MomentOfInertia = float3x3.identity });
        entityManager.SetComponentData(rigidBody, new Unity.Transforms.Position { Value = startPos });
        entityManager.SetComponentData(rigidBody, new Velocity { Value = startVel });

        Entity sphereCollider = entityManager.CreateEntity(SphereColliderArchetype);
        entityManager.SetComponentData(sphereCollider, new PhysicsEngine.Collider { RigidBodyEntity = rigidBody });
        entityManager.SetComponentData(sphereCollider, new PhysicsEngine.SphereCollider { Radius = 0.5f });
        entityManager.SetComponentData(sphereCollider, new PhysicsEngine.ColliderPhysicsProperties { Friction = 0f, CoefficientOfRestitution = 0f });

        Entity sphereRender = entityManager.CreateEntity(SimpleRendererArchetype);
        entityManager.SetSharedComponentData(sphereRender, SphereSmallRendererComponent.Value);
        entityManager.SetComponentData(sphereRender, new Unity.Transforms.Position { Value = startPos });
        entityManager.SetComponentData(sphereRender, new FollowRigidBody { RigidBodyEntity = rigidBody });
    }

    public void SpawnSphereBig(float3 startPos, float3 startVel)
    {
        EntityManager entityManager = World.Active.GetOrCreateManager<EntityManager>();

        Entity rigidBody = entityManager.CreateEntity(KinematicRigidBodyArchetype);
        entityManager.SetComponentData(rigidBody, new RigidBody { IsKinematic = 1, InverseMass = 0f, MomentOfInertia = float3x3.identity });
        entityManager.SetComponentData(rigidBody, new Unity.Transforms.Position { Value = startPos });
        entityManager.SetComponentData(rigidBody, new Velocity { Value = startVel });

        Entity sphereCollider = entityManager.CreateEntity(SphereColliderArchetype);
        entityManager.SetComponentData(sphereCollider, new PhysicsEngine.Collider { RigidBodyEntity = rigidBody });
        entityManager.SetComponentData(sphereCollider, new PhysicsEngine.SphereCollider { Radius = 50f });
        entityManager.SetComponentData(sphereCollider, new PhysicsEngine.ColliderPhysicsProperties { Friction = 0f, CoefficientOfRestitution = 0f });

        Entity sphereRender = entityManager.CreateEntity(SimpleRendererArchetype);
        entityManager.SetSharedComponentData(sphereRender, SphereBigRendererComponent.Value);
        entityManager.SetComponentData(sphereRender, new Unity.Transforms.Position { Value = startPos });
        entityManager.SetComponentData(sphereRender, new FollowRigidBody { RigidBodyEntity = rigidBody });
    }

    //public void SpawnBoxSmall(float3 startPos, float3 startVel)
    //{
    //    EntityManager entityManager = World.Active.GetOrCreateManager<EntityManager>();

    //    Entity rigidBody = entityManager.CreateEntity(KineticRigidBodyArchetype);
    //    entityManager.SetComponentData(rigidBody, new RigidBody { Mass = 1f });
    //    entityManager.SetComponentData(rigidBody, new RigidBodyPosition { Value = startPos });
    //    entityManager.SetComponentData(rigidBody, new Velocity { Value = startVel });

    //    Entity boxCollider = entityManager.CreateEntity(BoxColliderArchetype);
    //    entityManager.SetComponentData(boxCollider, new PhysicsEngine.Collider { RigidBodyEntity = rigidBody });
    //    entityManager.SetComponentData(boxCollider, new PhysicsEngine.BoxCollider { HalfSize = BoxSmallRendererComponent.Value.mesh.bounds.extents });
    //    entityManager.SetComponentData(boxCollider, new PhysicsEngine.ColliderPhysicsProperties { Friction = 0f, CoefficientOfRestitution = 0f });

    //    Entity sphereRender = entityManager.CreateEntity(SimpleRendererArchetype);
    //    entityManager.SetSharedComponentData(sphereRender, BoxSmallRendererComponent.Value);
    //    entityManager.SetComponentData(sphereRender, new Position { Value = startPos });
    //    entityManager.SetComponentData(sphereRender, new FollowRigidBody { RigidBodyEntity = rigidBody });
    //}
}
