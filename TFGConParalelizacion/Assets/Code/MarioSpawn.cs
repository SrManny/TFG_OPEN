using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class MarioSpawn : MonoBehaviour
{
    public GameObject Mariobros, spawn;
    public Transform destination;
    GameObject ListOfMarios;
    public float Zvision, directionW;
    Dictionary<int, Node> Graph;
    NavMeshTriangulation triangulization;
    Mesh path;
    int count = 0;
    public Material material;
    AStar astar;

    // Start is called before the first frame update
    void Start()
    {
        triangulization = NavMesh.CalculateTriangulation();
        path = GetComponent<MeshFilter>().mesh;
        Graph = new Dictionary<int, Node>();
        ListOfMarios = new GameObject();
        ListOfMarios.transform.parent = this.transform;
        ListOfMarios.transform.rotation = Quaternion.identity;
        spawn.transform.parent = ListOfMarios.transform;
        destination.parent = ListOfMarios.transform;
        ListOfMarios.name = "ListOfMarios";
        Zvision = 1f;
        astar = new AStar();
        GraphData g = GetComponent<GraphData>();
        g.Graph = astar.Graph;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Return))
        {
            for (int i = 0; i < 1; ++i) Spawn();
        }
        else if (Input.GetKeyDown(KeyCode.Delete))
        {
            foreach (Transform child in ListOfMarios.transform)
            {
                if (child.gameObject.tag == "Mario") GameObject.Destroy(child.gameObject);
            }
            Mesh ola = this.GetComponent<MeshFilter>().mesh;
        }
    }

    private void Spawn()
    {
        GameObject MarioSpawned = Instantiate(Mariobros, new Vector3(spawn.transform.position.x + Random.Range(-15, 12.5f), spawn.transform.position.y, spawn.transform.position.z + Random.Range(-15 , 10)), Quaternion.identity);
        MarioSpawned.transform.parent = ListOfMarios.transform;
        MarioSpawned.name = " " + count;
        MarioMove script = MarioSpawned.GetComponent<MarioMove>();
        script.directionW = directionW;
        script.astar = astar;
        script.Graph = astar.Graph;
        BoxCollider bc = MarioSpawned.GetComponentInChildren<BoxCollider>();
        Vector3 size = bc.size;
        bc.size = new Vector3(size.x,size.y,size.z*Zvision);
        Vector3 center = bc.center;
        center.z = center.z + (bc.size.z-size.z)/2f; bc.center = center;
        script._destination = destination.transform;
        ++count;
    }

}
