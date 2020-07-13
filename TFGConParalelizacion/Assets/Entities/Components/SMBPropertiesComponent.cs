using Unity.Entities;
using UnityEngine;
using Unity.Mathematics;

[System.Serializable]
public struct SMBProperties : ISharedComponentData
{
    public float radius;
    public float smoothingRadius;
    public float smoothingRadiusSq;

    public float mass;

    public float restDensity;
    public float viscosity;
    public float gravityMult;

    public float drag;
}

public class SMBPropertiesComponent : SharedComponentDataWrapper<SMBProperties> { }
