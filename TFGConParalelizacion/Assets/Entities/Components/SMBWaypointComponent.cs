using Unity.Entities;
using Unity.Mathematics;

[System.Serializable]
public struct SMBWaypoint : IComponentData
{
    public float3 start;
    public float3 end;
}

public class SMBWaypointComponent : ComponentDataWrapper<SMBWaypoint> { }
