/*
    This MonoBehaviour spawns and setup the boids on start
 */
using Unity.Entities;
using Unity.Rendering;
using Unity.Transforms;
using Unity.Mathematics;
using UnityEngine;

public class Bootstrap : MonoBehaviour
{
    [SerializeField] Mesh mesh;
    [SerializeField] Material material;

    [SerializeField] float maxSpeed = 3.0f;
    [SerializeField] uint boidCount = 10;

    void Start()
    {
        // Init random number generator
        Unity.Mathematics.Random random = new Unity.Mathematics.Random(42);

        // Get EntityManager
        EntityManager entityManager = World.Active.GetOrCreateManager<EntityManager>();

        // SPAWN ALL THE BOIDS
        for (uint i = 0; i < boidCount; i++)
        {
            // Create entity
            Entity boid = entityManager.CreateEntity(
                ComponentType.Create<BoidData>(),
                ComponentType.Create<Position>(),
                ComponentType.Create<Rotation>(),
                ComponentType.Create<LocalToWorld>(),
                ComponentType.Create<RenderMesh>()
            );

            // Fill boid data
            entityManager.SetComponentData(boid, new BoidData{
                Speed = random.NextFloat(1.0f, maxSpeed)
            });

            // Fill position data
            entityManager.SetComponentData(boid, new Position{
                Value = new float3(
                    random.NextFloat(-200.0f, 200.0f), 
                    random.NextFloat(-200.0f, 200.0f), 
                    random.NextFloat(-200.0f, 200.0f)
                )
            });

            // Fill rotation data
            entityManager.SetComponentData(boid, new Rotation{
                Value = quaternion.Euler(
                    random.NextFloat(-180.0f, 180.0f),
                    random.NextFloat(-180.0f, 180.0f),
                    random.NextFloat(-180.0f, 180.0f)
                )
            });

            // Fill rendermesh data
            entityManager.SetSharedComponentData(boid, new RenderMesh{
                mesh = this.mesh,
                material = this.material
            });
        }
    }
}
