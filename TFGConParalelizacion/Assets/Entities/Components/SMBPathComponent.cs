using Unity.Entities;
using Unity.Mathematics;
using Unity.Collections;

[System.Serializable]
public struct SMBPath : IComponentData
{
    public int indexIni;
    public int indexFin;
    public float3 NextPoint;
    public int Firsttriangle;
    public int recalculate;
    public float3 fordwarDir;
    //public float sSpeed;
    //public NativeFixedLengthAttribute ola
    //public NativeArray<int> path;
    //public nati
    //public path;
}

public class SMBPathComponent : ComponentDataWrapper<SMBPath> { }
