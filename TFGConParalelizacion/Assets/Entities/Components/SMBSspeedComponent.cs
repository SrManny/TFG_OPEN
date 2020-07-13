using Unity.Entities;
using UnityEngine;
using Unity.Mathematics;

[System.Serializable]
public struct SMBSspeed : IComponentData
{
    public float Sspeed;
}

public class SMBSspeedComponent : ComponentDataWrapper<SMBSspeed> { }
