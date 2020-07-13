using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class SpawnUnity : MonoBehaviour
{
    public Transform goal;
    public GameObject sphere;
    public Transform Spawn;
    public int amount;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Return))
        {
            for (int i = 0; i < amount; ++i)
            {
                GameObject sphereAux = Instantiate(sphere, new Vector3(Spawn.transform.position.x + Random.Range(-7, 7), Spawn.transform.position.y, Spawn.transform.position.z + Random.Range(-7, 7)), Quaternion.identity);
                NavMeshAgent agent = sphereAux.GetComponent<NavMeshAgent>();
                agent.destination = goal.position;
            }
        }
    }
}
