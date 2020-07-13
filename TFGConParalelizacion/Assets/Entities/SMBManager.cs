using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Collections;
using System.Collections.Generic;
public class SMBManager : MonoBehaviour
{

    // Import
    [Header("Import")]
    [SerializeField] private GameObject smbMarioPrefab;
    private List<SMBProperties> uniqueTypes = new List<SMBProperties>(10);
    [SerializeField] private GameObject sphColliderPrefab;
    private EntityManager manager;
    AStar astar;
    static public List<int> AllPaths = new List<int>();
    // Properties
    [Header("Properties")]
    [SerializeField] private int amount;
    [SerializeField] private GameObject Spawn;
    [SerializeField] private Transform _destination;// _destinationp, _destinationpp;
    [SerializeField] private GameObject Spawn2;
    [SerializeField] private Transform _destination2;// _destinationp2, _destinationpp2;


    private void Start()
    {
        // Imoprt
        manager = World.Active.GetOrCreateManager<EntityManager>();
        // Setup
        AddColliders();
        AddParticles(amount);
    }



    private void AddParticles(int _amount)
    {
        _amount = _amount*2;
        NativeArray<Entity> entities = new NativeArray<Entity>(_amount, Allocator.Temp);
        manager.Instantiate(smbMarioPrefab, entities);
        
        float bas = 2;
        for (int ii = 0; ii < _amount/2; ii++)
        {
            int i = ii * 2;
            int j = i + 1;
            //Position position = new Position { Value = new float3(Spawn.transform.position.x + i % 15 + UnityEngine.Random.Range(-0.1f, 0.1f), bas + (i / 15 / 15) * 1.1f, Spawn.transform.position.z + (i / 15) % 15) + UnityEngine.Random.Range(-0.1f, 0.1f) };
            Position position = new Position { Value = new float3(Spawn.transform.position.x + i % 10 + UnityEngine.Random.Range(-0.1f, 0.1f), bas + (i / 10 / 10) * 1.1f, Spawn.transform.position.z + (i / 10) % 10) + UnityEngine.Random.Range(-0.1f, 0.1f) };
            //Position position2 = new Position { Value = new float3(Spawn2.transform.position.x + j % 15 + UnityEngine.Random.Range(-0.1f, 0.1f), bas + (j / 15 / 15) * 1.1f, Spawn2.transform.position.z + (j / 15) % 15) + UnityEngine.Random.Range(-0.1f, 0.1f) };
            Position position2 = new Position { Value = new float3(Spawn2.transform.position.x + j % 10 + UnityEngine.Random.Range(-0.1f, 0.1f), bas + (j / 10 / 10) * 1.1f, Spawn2.transform.position.z + (j / 10) % 10) + UnityEngine.Random.Range(-0.1f, 0.1f) };

            manager.SetComponentData(entities[i], position);
            //manager.SetComponentData(entities[j], new SMBPath { indexIni = index, indexFin = 0 + index /*path = new NativeArray<int>(1, Allocator.Temp)*/ });
            manager.SetComponentData(entities[i], new SMBDestination { finished = 0, destination = new float3(_destination.position.x, bas, _destination.position.z), destinations2 = new float3(_destination.position.x, bas, _destination.position.z) });
            manager.SetComponentData(entities[i], new SMBSspeed { Sspeed = 7f});
            manager.SetComponentData(entities[j], position2);
            manager.SetComponentData(entities[j], new SMBDestination { destination = new float3(_destination2.position.x, bas, _destination2.position.z) });
            manager.SetComponentData(entities[j], new SMBSspeed { Sspeed = 7f });
        
        }

        entities.Dispose();
    }



    private void AddColliders()
    {
        // Find all colliders
        GameObject[] colliders = GameObject.FindGameObjectsWithTag("SMBCollider");
        // Turn them into entities
        NativeArray<Entity> entities = new NativeArray<Entity>(colliders.Length, Allocator.Temp);
        manager.Instantiate(sphColliderPrefab, entities);

        // Set data
        for (int i = 0; i < colliders.Length; i++)
        {
            manager.SetComponentData(entities[i], new SMBCollider
            {
                position = colliders[i].transform.position,
                right = colliders[i].transform.right,
                up = new float3(0, 0, -1),
                scale = new float2(2000,2000)
            });
        }

        // Done
        entities.Dispose();
    }
}
