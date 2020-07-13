using Unity.Entities;
using Unity.Mathematics;

[System.Serializable]
public struct SMBVelocity : IComponentData
{
    public float3 Value;
}

public class SMBVelocityComponent : ComponentDataWrapper<SMBVelocity> { }
