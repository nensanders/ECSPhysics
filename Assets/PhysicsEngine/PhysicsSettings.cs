using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu]
public class PhysicsSettings : ScriptableObject
{
    [Header("General Settings")]
    public int ConstraintSolverIterations = 6;
    public float BaumgarteBias = 0.01f;
    public int BVHLevelParallelism = 5;

    [Header("Jobs Settings")]
    public int RigidbodySystemBatchCount = 32;
    public int ColliderPoseSystemBatchCount = 32;
    public int ColliderAABBSystemBatchCount = 32;
    public int BroadphaseSystemBatchCount = 32;
    public int ContactsPhaseInitializationSystemBatchCount = 32;
    public int ContactsGenerationSystemBatchCount = 32;
    public int ContactsResolutionSystemBatchCount = 32;

    [Header("Debug")]
    public bool ShowAABBs = false;
    public bool ShowZOrderCurve = false;
    public bool ShowBVHBounds = false;
    public bool ShowCollisionPairs = false;

    public static PhysicsSettings LoadFromResources()
    {
        return Resources.Load<PhysicsSettings>("DefaultPhysicsSettings");
    }
}
