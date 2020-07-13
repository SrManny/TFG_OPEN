﻿using Unity.Entities;
using Unity.Mathematics;

[System.Serializable]
public struct SMBCollider : IComponentData
{
    public float3 position;
    public float3 right;
    public float3 up;
    public float2 scale;
}

public class SMBColliderComponent : ComponentDataWrapper<SMBCollider> { }
