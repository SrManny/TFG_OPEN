using Unity.Entities;
using UnityEngine;
using Unity.Mathematics;

[System.Serializable]
public struct SMBDestination : IComponentData
{
    public float3 origin;
    public float3 destination;
    public float3 destinations2;
    public int finished;
}

public class SMBDestinationComponent : ComponentDataWrapper<SMBDestination> { }
