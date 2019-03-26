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
    NativeArray<float3> _outforce;

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
    private struct BoidSteeringJob : IJobNativeMultiHashMapMergedSharedKeyIndices
    {

        public Unity.Mathematics.Random random;
        [ReadOnly] public int BoidCount;
        [ReadOnly] public float DeltaTime;

        [ReadOnly] public NativeArray<UsedData> inData;
        [WriteOnly] public NativeArray<float3> outforce;

        public void ComputeMovement(int index)
        {
            float3 position = inData[index].pos;
            quaternion rotation = new quaternion(inData[index].rot.x, inData[index].rot.y, inData[index].rot.z, inData[index].rot.w);

            float3 forward  = math.mul(rotation, new float3(0.0f, 0.0f, 1.0f));
            float3 up       = math.mul(rotation, new float3(0.0f, 1.0f, 0.0f));

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
                    
                    avgHeading      += other_forward;
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

            forward = math.mul(
                quaternion.Euler(
                    random.NextFloat(-10.0f, 10.0f),
                    random.NextFloat(-10.0f, 10.0f),
                    random.NextFloat(-10.0f, 10.0f)
                ),
                forward
            );

            float3 force = (
                (forward * 0.25f) + 
                (avgHeading * 0.4f) + 
                (towardsForce * 0.2f) + 
                (outwardsForce * 0.15f)
            );

            if (math.distance(float3.zero, position) > 1000.0f)
            {
                force += math.normalize(-position);
            }

            outforce[index] = math.normalize(force);
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
        [ReadOnly] public float DeltaTime;
        [ReadOnly] public NativeArray<float3> outforce;

        public void Execute(Entity entity, int index, ref Position position, ref Rotation rotation)
        {
            float3 up  = math.mul(rotation.Value, new float3(0.0f, 1.0f, 0.0f));
            float speed = 5.0f * DeltaTime;

            position.Value += outforce[index] * speed;
            rotation.Value = quaternion.LookRotation(outforce[index], up);
        }
    }

    [BurstCompile]
    struct MoveForward : IJobProcessComponentDataWithEntity<Position, Rotation>
    {
        [ReadOnly] public float DeltaTime;

        public void Execute(Entity entity, int index, ref Position position, ref Rotation rotation)
        {
            float3 forward  = math.mul(rotation.Value, new float3(0.0f, 0.0f, 1.0f));
            float speed = 5.0f * DeltaTime;

            position.Value += forward * speed;
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
        _outforce = new NativeArray<float3>(boidCount, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
    }

    float lastSteeringStart = -1.0f;
    BoidSteeringJob steeringJob;
    JobHandle steeringJobHandle;

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        int boidCount = _BoidGroup.CalculateLength();

        if (lastSteeringStart <= 0.0f)
        {
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

            JobHandle initJobHandle = JobHandle.CombineDependencies(copyDataJobHandle, hashPositionJobHandle);

            steeringJob = new BoidSteeringJob()
            {
                random = new Unity.Mathematics.Random(_random.NextUInt()),
                DeltaTime = Time.deltaTime,
                BoidCount = boidCount,
                inData = _inData,
                outforce = _outforce

            };

            steeringJobHandle = steeringJob.Schedule(_hashMap, 64, initJobHandle);
            lastSteeringStart = Time.realtimeSinceStartup;

            MoveForward moveJob = new MoveForward()
            {
                DeltaTime = Time.deltaTime
            };
            JobHandle moveJobHandle = moveJob.ScheduleGroup(_BoidGroup, initJobHandle);

            _BoidGroup.AddDependency(moveJobHandle);
            return moveJobHandle;
        }
        else if (Time.realtimeSinceStartup - lastSteeringStart > 0.5f)
        {
            JobHandle initJobHandle = JobHandle.CombineDependencies(inputDeps, steeringJobHandle);

            ApplyValues applyValues = new ApplyValues()
            {
                DeltaTime = Time.deltaTime,
                outforce = _outforce
            };
            JobHandle applyValuesJobHandle = applyValues.ScheduleGroup(_BoidGroup, initJobHandle);

            lastSteeringStart = -1.0f;

            _BoidGroup.AddDependency(applyValuesJobHandle);
            return applyValuesJobHandle;
        }
        else
        {
            MoveForward moveJob = new MoveForward()
            {
                DeltaTime = Time.deltaTime,
            };
            JobHandle moveJobHandle = moveJob.ScheduleGroup(_BoidGroup, inputDeps);

            _BoidGroup.AddDependency(moveJobHandle);
            return moveJobHandle;
        }
    }

    protected override void OnStopRunning()
    {
        _hashMap.Dispose();

        _inData.Dispose();
        _outforce.Dispose();
    }
}