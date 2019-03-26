/*
    This struct represents the data needed for boids
    There isn't much here
 */
using System;
using Unity.Entities;
using Unity.Transforms;
using Unity.Mathematics;
using UnityEngine;

[Serializable]
public struct BoidData : IComponentData
{
    public float Speed;
}

