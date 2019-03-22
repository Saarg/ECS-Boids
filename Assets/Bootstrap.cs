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
        Unity.Mathematics.Random random = new Unity.Mathematics.Random(42);

        EntityManager entityManager = World.Active.GetOrCreateManager<EntityManager>();

        for (uint i = 0; i < boidCount; i++)
        {
            Entity boid = entityManager.CreateEntity(
                ComponentType.Create<BoidData>(),
                ComponentType.Create<Position>(),
                ComponentType.Create<Rotation>(),
                ComponentType.Create<LocalToWorld>(),
                ComponentType.Create<RenderMesh>()
            );

            entityManager.SetComponentData(boid, new BoidData{
                Speed = random.NextFloat(1.0f, maxSpeed)
            });

            entityManager.SetComponentData(boid, new Position{
                Value = new float3(random.NextFloat(-100.0f, 100.0f), random.NextFloat(-100.0f, 100.0f), random.NextFloat(-100.0f, 100.0f))
            });

            entityManager.SetComponentData(boid, new Rotation{
                Value = quaternion.Euler(
                    random.NextFloat(-180.0f, 180.0f),
                    random.NextFloat(-180.0f, 180.0f),
                    random.NextFloat(-180.0f, 180.0f)
                )
            });

            entityManager.SetSharedComponentData(boid, new RenderMesh{
                mesh = this.mesh,
                material = this.material
            });
        }
    }
}
