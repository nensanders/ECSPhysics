using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysXSceneInitializer : MonoBehaviour
{

    public int SphereCount = 50;
    public float SpawnDistance = 10;
    public float SpawnVelocityRange = 5;
    public Rigidbody SpherePrefab;

    void Start()
    {
        // Spawn spheres random
        for (int i = 0; i < SphereCount; i++)
        {
            Vector3 startPos = new Vector3(
                transform.position.x + UnityEngine.Random.Range(-SpawnDistance, SpawnDistance),
                transform.position.y + UnityEngine.Random.Range(-SpawnDistance, SpawnDistance),
                transform.position.z + UnityEngine.Random.Range(-SpawnDistance, SpawnDistance)
                );

            Vector3 startVel = new Vector3(
                startPos.x >= 0f ? -SpawnVelocityRange : SpawnVelocityRange,
                0,
                0
                );

            Rigidbody sphere = Instantiate(SpherePrefab, startPos, Quaternion.identity);
            sphere.velocity = startVel;
        }
    }
}
