/*
    This System implements a flocking mecanism inspired unity's own flocking exemple
    In it's current state the boids move at every frame and only steer every n seconds with n being hardcoded
    The flocking algorithm is very basic as it's not the point of the demo, I'm only trying to learn ECS

    I know there is probably some MEMORY MANAGMENT ISSUES as there is a few errors in unity but I don't think there is a memory leak
    on runtime.

    Also adding some lerp on the force application over the frames would make for a smoother display
 */

using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

// Update the BoidsSystem before the trasnforms because it will affect it
[UpdateBefore(typeof(TransformSystem))]
public class BoidsSystem : JobComponentSystem
{
//====================================================================================================================
// Jobs
//====================================================================================================================

    //================================================================================================================
    // CopyDataJob
    //
    // Job that copies the data needed into the _inData list, this is then sent to the SteeringJob
    //================================================================================================================
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

    //================================================================================================================
    // HashPositionJob
    //
    // Generate the hasmap needed for the SteeringJob, the hashmap keys represent the cell the boid is in
    // Currently we only have 8 cells separated by the x, y and z plans
    // In the future this should probably use the same bucket system as the official sample with a cell width of perception radius of the boids
    //================================================================================================================
    [BurstCompile]
    private struct HashPositionJob : IJobProcessComponentDataWithEntity<Position>
    {
        public NativeMultiHashMap<int,int>.Concurrent hashMap;

        public void Execute(Entity entity, int index, [ReadOnly] ref Position position)
        {
            hashMap.Add((position.Value.x > 0 ? 1 : 0) + (position.Value.y > 0 ? 2 : 0) + (position.Value.z > 0 ? 4 : 0), index);
        }
    }

    //================================================================================================================
    // BoidSteeringJob
    //
    // Steering algorithm, a basic flocking algorith that can and should be optimised
    //================================================================================================================
    [BurstCompile]
    private struct BoidSteeringJob : IJobNativeMultiHashMapMergedSharedKeyIndices
    {
        // Random number generator
        public Unity.Mathematics.Random random;
        // DeltaTime since last frame
        [ReadOnly] public float DeltaTime;

        // Data representing the boids status needed for the algorithm. It's ReadOnly because we only read it and don't want any unecessary locks
        [ReadOnly] public NativeArray<UsedData> inData;
        // This is the forces applied to the boids by the steering algorithm
        [WriteOnly] public NativeArray<float3> outforce;

        public void ComputeMovement(int index)
        {
            // Boid data
            float3 position = inData[index].pos;
            quaternion rotation = new quaternion(inData[index].rot.x, inData[index].rot.y, inData[index].rot.z, inData[index].rot.w);

            float3 forward  = math.mul(rotation, new float3(0.0f, 0.0f, 1.0f));
            float3 up       = math.mul(rotation, new float3(0.0f, 1.0f, 0.0f));

            // flocking forces
            float3 avgHeading = float3.zero;
            float3 towardsForce = float3.zero;
            float3 outwardsForce = float3.zero;

            uint inRangeCount = 0;
            
            // Loop through all boids
            for (var j = 0; j < inData.Length; j++)
            {
                // Skip if it's our current boid
                if (j == index)
                    continue;

                // Other boid transform
                float3 other_position = inData[j].pos;
                quaternion other_rotation = inData[j].rot;
                
                // If the boid is in range we compute the forces
                if (math.distance(other_position, position) < 30.0f)
                {
                    float3 other_forward  = math.mul(other_rotation, new float3(0.0f, 0.0f, 1.0f));
                    float3 other_up       = math.mul(other_rotation, new float3(0.0f, 1.0f, 0.0f));
                    
                    avgHeading      += other_forward;
                    towardsForce    += other_position;
                    outwardsForce   += other_position - position;

                    inRangeCount++;
                }
            }

            // If we have boids in range we normalize the forces
            if (inRangeCount > 0)
            {
                avgHeading      = math.normalize(avgHeading / inRangeCount);
                towardsForce    = math.normalize((towardsForce / inRangeCount) - position);
                outwardsForce   = math.normalize(outwardsForce / inRangeCount * -1.0f); 
            }

            // Rotate in a random direction
            forward = math.mul(
                quaternion.Euler(
                    random.NextFloat(-10.0f, 10.0f),
                    random.NextFloat(-10.0f, 10.0f),
                    random.NextFloat(-10.0f, 10.0f)
                ),
                forward
            );

            // Apply forces factors
            float3 force = (
                (forward * 0.25f) + 
                (avgHeading * 0.4f) + 
                (towardsForce * 0.2f) + 
                (outwardsForce * 0.15f)
            );

            // Apply a force towards (0, 0, 0) if the boid is too far
            if (math.distance(float3.zero, position) > 1000.0f)
            {
                force += math.normalize(-position);
            }

            // Write force into outforce
            outforce[index] = math.normalize(force);
        } 

        // This is where we can optimise a lot but I didn't get to it yet
        public void ExecuteFirst(int index)
        {
            ComputeMovement(index);
        }
        // This is where we can optimise a lot but I didn't get to it yet
        public void ExecuteNext(int cellIndex, int index)
        {
            ComputeMovement(index);
        }
    }

    //================================================================================================================
    // ApplyValues
    //
    // Apply the forces the steering job returned
    //================================================================================================================
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

    //================================================================================================================
    // MoveForward
    //
    // Simply move forward on frame without forces
    //================================================================================================================
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

//====================================================================================================================
// System Variables
//====================================================================================================================

    // Random number generator
    private Unity.Mathematics.Random _random;

    // Group of entities corresponding to our boid entities
    private ComponentGroup _BoidGroup;

    // Hashmap for the IJobNativeMultiHashMapMergedSharedKeyIndices to dispatch entities according to their world position
    // Check this video for more info: https://www.youtube.com/watch?v=p65Yt20pw0g&feature=youtu.be&t=2329
    NativeMultiHashMap<int, int> _hashMap;

    // Data needed by the flocking algorithm in one struct
    struct UsedData
    {
        public float3 pos;
        public float4 rot;
    }
    
    // Data Copied at the begining and sent to the algorithm
    NativeArray<UsedData> _inData;
    // Data computed by the algorithm
    NativeArray<float3> _outforce;

    // Variables holding the steeringjob data to be caried over multiple frames
    float lastSteeringStart = -1.0f;
    BoidSteeringJob steeringJob;
    JobHandle steeringJobHandle;

//====================================================================================================================
// System Methods
//====================================================================================================================

    //================================================================================================================
    // OnCreateManager()
    //
    // Init _BoidGroup and random number generator
    //================================================================================================================
    protected override void OnCreateManager()
    {
        // Cached access to a set of ComponentData based on a specific query
        _BoidGroup = GetComponentGroup(typeof(Position), typeof(Rotation), typeof(BoidData));

        _random = new Unity.Mathematics.Random(404);
    }

    //================================================================================================================
    // OnStartRunning()
    //
    // Init the arrays the need the boidCount
    //================================================================================================================
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

    //================================================================================================================
    // OnUpdate()
    //
    // Once for every frame
    //================================================================================================================
    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        // If no steering job is currently running
        if (lastSteeringStart <= 0.0f)
        {
            // Copy the data into our array
            _inData.Clear();
            var copyDataJob = new CopyDataJob()
            {
                inData = _inData
            };
            // Schedule the copy job tu run after inputDeps
            var copyDataJobHandle = copyDataJob.ScheduleGroup(_BoidGroup, inputDeps);

            // Dispatch entities into cells for the steeringjob
            _hashMap.Clear();
            var hashPositionJob = new HashPositionJob()
            {
                hashMap = _hashMap.ToConcurrent()
            };
            // Schedule the hash job tu run after inputDeps
            var hashPositionJobHandle = hashPositionJob.ScheduleGroup(_BoidGroup, inputDeps);

            // Create a job handle that combines the copy and hash taht runs concurently
            JobHandle initJobHandle = JobHandle.CombineDependencies(copyDataJobHandle, hashPositionJobHandle);

            // The big job that runs accros multiple frames this job is the main bottleneck
            _outforce.Clear();
            steeringJob = new BoidSteeringJob()
            {
                random = new Unity.Mathematics.Random(_random.NextUInt()),
                DeltaTime = Time.deltaTime,
                inData = _inData,
                outforce = _outforce

            };
            // Schedule the steering job tu start only after copy and hash is done because we need to reed their output
            steeringJobHandle = steeringJob.Schedule(_hashMap, 64, initJobHandle);

            // Save the time we scheduled the job
            lastSteeringStart = Time.realtimeSinceStartup;

            // On this frame we want to move forward we can schedule this job only depending on inputDeps as it is independent from all the oother jobs
            MoveForward moveJob = new MoveForward()
            {
                DeltaTime = Time.deltaTime
            };
            JobHandle moveJobHandle = moveJob.ScheduleGroup(_BoidGroup, inputDeps);

            // We only return the movejob handle because the steeringJob will run over multiple frames
            _BoidGroup.AddDependency(moveJobHandle);
            return moveJobHandle;
        }
        // If the steeringJob has run for the time we allowed it
        else if (Time.realtimeSinceStartup - lastSteeringStart > 0.5f)
        {
            // Combine dependencies to wait for the steering job completion
            JobHandle initJobHandle = JobHandle.CombineDependencies(inputDeps, steeringJobHandle);

            // Apply the outforces
            ApplyValues applyValues = new ApplyValues()
            {
                DeltaTime = Time.deltaTime,
                outforce = _outforce
            };
            JobHandle applyValuesJobHandle = applyValues.ScheduleGroup(_BoidGroup, initJobHandle);

            // Indicate that no steering job will be running next frame
            lastSteeringStart = -1.0f;

            _BoidGroup.AddDependency(applyValuesJobHandle);
            return applyValuesJobHandle;
        }
        // Frames waiting for the steering job tick
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

    //================================================================================================================
    // OnStopRunning()
    //
    // Dispose of the memory we allocated on a Persistent allocation
    //================================================================================================================
    protected override void OnStopRunning()
    {
        _hashMap.Dispose();

        _inData.Dispose();
        _outforce.Dispose();
    }
}