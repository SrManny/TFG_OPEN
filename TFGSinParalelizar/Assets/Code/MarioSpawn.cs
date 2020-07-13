using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class MarioSpawn : MonoBehaviour
{
    public GameObject Mariobros, spawn, spawn2;
    public Transform destination, destination2;
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
        //path.Clear();
        Graph = new Dictionary<int, Node>();
        ListOfMarios = new GameObject();
        ListOfMarios.transform.parent = this.transform;
        ListOfMarios.transform.rotation = Quaternion.identity;
        spawn.transform.parent = ListOfMarios.transform;
        spawn2.transform.parent = ListOfMarios.transform;
        destination.parent = ListOfMarios.transform;
        destination2.parent = ListOfMarios.transform;
        ListOfMarios.name = "ListOfMarios";
        astar = new AStar();
        GraphData g = GetComponent<GraphData>();
        g.Graph = astar.Graph;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Return))
        {
            for (int i = 0; i < 2; ++i)
            {
                if (i % 2 == 0) Spawn(spawn, destination);
                else Spawn(spawn2, destination2);
            }
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

    private void Spawn(GameObject spawn, Transform destination)
    {
        //GameObject MarioSpawned = Instantiate(Mariobros, new Vector3(spawn.transform.position.x + Random.Range(-15, 12.5f), spawn.transform.position.y, spawn.transform.position.z + Random.Range(-15 , 10)), Quaternion.identity);
        GameObject MarioSpawned = Instantiate(Mariobros, new Vector3(spawn.transform.position.x, spawn.transform.position.y, spawn.transform.position.z), Quaternion.identity);
        MarioSpawned.transform.parent = ListOfMarios.transform;
        MarioSpawned.name = " " + count;
        MarioMove script = MarioSpawned.GetComponent<MarioMove>();
        script.directionW = directionW;
        script.astar = astar;
        script.Graph = astar.Graph;
        BoxCollider bc = MarioSpawned.GetComponentInChildren<BoxCollider>();
        Vector3 size = bc.size;
        bc.size = new Vector3(size.x,size.y,size.z*Zvision);
        Vector3 auxxx = MarioSpawned.transform.position;
        auxxx.z = MarioSpawned.transform.position.z + size.z * Zvision + bc.transform.localPosition.z * MarioSpawned.transform.localScale.z- 1.125f/4f;  //- MarioSpawned.transform.localScale.z/2f;
        
        Debug.DrawLine(MarioSpawned.transform.position, auxxx,Color.black);
        Vector3 center = bc.center;
        center.z = center.z + (bc.size.z-size.z)/2f; bc.center = center;
        script._destination = destination.transform;
        script.funnelMAX = Vector3.Distance(MarioSpawned.transform.position, auxxx);
        ++count;
        //script.Graph = astar.Graph;
    }

    private void Spawn2()
    {
        //GameObject MarioSpawned = Instantiate(Mariobros, new Vector3(spawn2.transform.position.x + Random.Range(-15, 12.5f), spawn2.transform.position.y, spawn2.transform.position.z + Random.Range(-15, 10)), Quaternion.identity);
        GameObject MarioSpawned = Instantiate(Mariobros, new Vector3(spawn2.transform.position.x, spawn2.transform.position.y, spawn2.transform.position.z), Quaternion.identity);
        //Debug.DrawLine(MarioSpawned.transform.position,new Vector3(0,0,0), Color.black);
        MarioSpawned.transform.parent = ListOfMarios.transform;
        MarioSpawned.name = " " + count;
        MarioMove script = MarioSpawned.GetComponent<MarioMove>();
        script.directionW = directionW;
        script.astar = astar;
        script.Graph = astar.Graph;
        BoxCollider bc = MarioSpawned.GetComponentInChildren<BoxCollider>();
        Vector3 size = bc.size;
        bc.size = new Vector3(size.x, size.y, size.z * Zvision);
        Vector3 auxxx = MarioSpawned.transform.position;

        auxxx.z = MarioSpawned.transform.position.z + size.z * Zvision + bc.transform.localPosition.z * MarioSpawned.transform.localScale.z - 1.125f/4f;// - MarioSpawned.transform.localScale.z/2f;
        Vector3 center = bc.center;
        center.z = center.z + (bc.size.z - size.z) / 2f; bc.center = center;
        script._destination = destination2.transform;
        script.funnelMAX = Vector3.Distance(MarioSpawned.transform.position, auxxx);
        ++count;
        //script.Graph = astar.Graph;
    }

}
