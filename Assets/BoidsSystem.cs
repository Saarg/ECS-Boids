using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;


[UpdateBefore(typeof(TransformSystem))]
public class BoidsSystem : JobComponentSystem
{
    private Unity.Mathematics.Random _random;
    private ComponentGroup _BoidGroup;

    NativeMultiHashMap<int, int> _hashMap;

    struct UsedData
    {
        public float3 pos;
        public float4 rot;
        
        float3 pad1;
        float3 pad2;
        float3 pad3;
    }

    NativeArray<UsedData> _inData;
    NativeArray<UsedData> _outData;

    protected override void OnCreateManager()
    {
        // Cached access to a set of ComponentData based on a specific query
        _BoidGroup = GetComponentGroup(typeof(Position), typeof(Rotation), typeof(BoidData));

        _random = new Unity.Mathematics.Random(404);
    }

    [BurstCompile]
    private struct CopyDataJob : IJobProcessComponentDataWithEntity<Position, Rotation, BoidData>
    {
        [WriteOnly] public NativeArray<UsedData> inData;

        public void Execute(Entity entity, int index, [ReadOnly] ref Position position, [ReadOnly] ref Rotation rotation, ref BoidData boidData)
        {
            inData[index] = new UsedData()
            {
                pos = position.Value,
                rot = new float4(rotation.Value.value.x, rotation.Value.value.y, rotation.Value.value.z, rotation.Value.value.w)
            };
        }
    }

    [BurstCompile]
    private struct HashPositionJob : IJobProcessComponentDataWithEntity<Position>
    {
        public NativeMultiHashMap<int,int>.Concurrent hashMap;

        public void Execute(Entity entity, int index, [ReadOnly] ref Position position)
        {
            hashMap.Add((position.Value.x > 0 ? 1 : 0) + (position.Value.y > 0 ? 2 : 0) + (position.Value.z > 0 ? 4 : 0), index);
        }
    }

    [BurstCompile]
    private struct BoidMovementJob : IJobNativeMultiHashMapMergedSharedKeyIndices
    {

        public Unity.Mathematics.Random random;
        [ReadOnly] public int BoidCount;
        [ReadOnly] public float DeltaTime;

        [ReadOnly] public NativeArray<UsedData> inData;
        [WriteOnly] public NativeArray<UsedData> outData;

        public void ComputeMovement(int index)
        {
            float3 position = inData[index].pos;
            quaternion rotation = new quaternion(inData[index].rot.x, inData[index].rot.y, inData[index].rot.z, inData[index].rot.w);

            float3 forward  = math.mul(rotation, new float3(0.0f, 0.0f, 1.0f));
            float3 up       = math.mul(rotation, new float3(0.0f, 1.0f, 0.0f));

            float speed = 30.0f * DeltaTime;

            // follow
            float3 avgHeading = float3.zero;
            float3 towardsForce = float3.zero;
            float3 outwardsForce = float3.zero;

            uint inRangeCount = 0;
            
            for (var j = 0; j < inData.Length; j++)
            {
                if (j == index)
                    continue;

                float3 other_position = inData[j].pos;
                quaternion other_rotation = inData[j].rot;
                
                if (math.distance(other_position, position) < 10.0f)
                {
                    float3 other_forward  = math.mul(other_rotation, new float3(0.0f, 0.0f, 1.0f));
                    float3 other_up       = math.mul(other_rotation, new float3(0.0f, 1.0f, 0.0f));
                    
                    avgHeading      += math.mul(other_rotation, new float3(0.0f, 0.0f, 1.0f));
                    towardsForce    += other_position;
                    outwardsForce   += other_position - position;

                    inRangeCount++;
                }
            }

            if (inRangeCount > 0)
            {
                avgHeading      = math.normalize(avgHeading / inRangeCount);
                towardsForce    = math.normalize((towardsForce / inRangeCount) - position);
                outwardsForce   = math.normalize(outwardsForce / inRangeCount * -1.0f); 
            }

            up = math.mul(
                quaternion.Euler(
                    random.NextFloat(-speed, speed),
                    random.NextFloat(-speed, speed),
                    random.NextFloat(-speed, speed)
                ),
                up
            );

            float3 force = (
                (up * 0.3f) + 
                (avgHeading * 0.4f) + 
                (towardsForce * 0.1f) + 
                (outwardsForce * 0.2f) + 
                (-position / 1000.0f)
            ) * speed;

            rotation = quaternion.LookRotation(forward, force);

            outData[index] = new UsedData()
            {
                pos = position + force,
                rot = rotation.value
            };
        } 

        public void ExecuteFirst(int index)
        {
            ComputeMovement(index);
        }

        public void ExecuteNext(int cellIndex, int index)
        {
            ComputeMovement(index);
        }
    }

    [BurstCompile]
    struct ApplyValues : IJobProcessComponentDataWithEntity<Position, Rotation>
    {
        [ReadOnly] public NativeArray<UsedData> outData;

        public void Execute(Entity entity, int index, ref Position position, ref Rotation rotation)
        {
            position.Value = outData[index].pos;
            rotation.Value = new quaternion(outData[index].rot.x, outData[index].rot.y, outData[index].rot.z, outData[index].rot.w);
        }
    }

    protected override void OnStartRunning()
    {
        int boidCount = _BoidGroup.CalculateLength();

        _hashMap = new NativeMultiHashMap<int,int>(boidCount,Allocator.Persistent);

        for (int i = 0 ; i < boidCount ; i++)
        {
            _hashMap.Add(0, i);
        }

        _inData  = new NativeArray<UsedData>(boidCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
        _outData = new NativeArray<UsedData>(boidCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        int boidCount = _BoidGroup.CalculateLength();

        var copyDataJob = new CopyDataJob()
        {
            inData = _inData
        };
        var copyDataJobHandle = copyDataJob.ScheduleGroup(_BoidGroup, inputDeps);

        _hashMap.Clear();
        var hashPositionJob = new HashPositionJob()
        {
            hashMap = _hashMap.ToConcurrent()
        };
        var hashPositionJobHandle = hashPositionJob.ScheduleGroup(_BoidGroup, inputDeps);

        var initJobHandle = JobHandle.CombineDependencies(copyDataJobHandle, hashPositionJobHandle);

        var movementJob = new BoidMovementJob()
        {
            random = new Unity.Mathematics.Random(_random.NextUInt()),
            DeltaTime = Time.deltaTime,
            BoidCount = boidCount,
            inData = _inData,
            outData = _outData

        };
        JobHandle movementJobHandle = movementJob.Schedule(_hashMap, 64, initJobHandle);

        var applyJob = new ApplyValues()
        {
            outData = _outData
        };
        var applyJobHandle = applyJob.ScheduleGroup(_BoidGroup, movementJobHandle);

        _BoidGroup.AddDependency(applyJobHandle);
        return applyJobHandle;
    }

    protected override void OnStopRunning()
    {
        _hashMap.Dispose();

        _inData.Dispose();
        _outData.Dispose();
    }
}