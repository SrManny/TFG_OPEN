using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GraphData : MonoBehaviour
{
    public AStar astar;
    public GraphGenerator g;
    public Dictionary<int, Node> Graph;
    
    // Start is called before the first frame update
    void Start()
    {
    }
    
    /*public List<int> trianglePathFromTo(Vector3 origin, Vector3 destination)
    {
        astar.setOrigin(origin);
        astar.setDestination(destination);
        return astar.trianglePath2();
    }

    public List<Vector3> trianglePath(Vector3 origin, Vector3 destination)
    {

        astar.setOrigin(origin);
        astar.setDestination(destination);
        return astar.trianglePath();
    }*/

    public Vector3 getClosestWayPoint(int current, Vector3 position, out int pointID, int neighBorID, string tag)
    {
        Vector3 aux = Graph[current].Neighbors[neighBorID].getClosestWayPoint(position, out pointID);
        return aux;
    }

    public Vector3 getProyectedWayPoint(int current, Vector3 position, int neighBorID)
    {
        Graph[current].NeighborsDic[neighBorID].increaseOccupation();
        Graph[neighBorID].NeighborsDic[current].increaseOccupation();
        return Graph[current].NeighborsDic[neighBorID].getProyectedWayPoint(position);
        //Debug.Log(tag + " " + Graph[current].Neighbors[neighBorID].isFree(pointID));
    }
    public bool isTooBusy(int current, int neighBorID)
    {
        return Graph[current].NeighborsDic[neighBorID].isTooBusy();
    }

    /*public List<int> recalculatePath(int currentTriangle, int neiID, Vector3 destination)
    {
        astar.modifyCost(currentTriangle, neiID, Graph[currentTriangle].NeighborsDic[neiID].occupation);
        return astar.recalculatePath(Graph[currentTriangle].Getbarycenter(), destination);
    }*/

    public void liberateOccupation(int triangle, int neighBorID)
    {
        Graph[triangle].NeighborsDic[neighBorID].decreaseOccupation();
        Graph[neighBorID].NeighborsDic[triangle].decreaseOccupation();
    }

    public bool anyFreeWayPoint(int triangle, int neighBorID)
    {
        
        return Graph[triangle].Neighbors[neighBorID].anyFreeWayPoint();
    }
    public bool anyNeighborFree(int triangle)
    {
        return Graph[triangle].anyNeighborFree();
    }

    public List<int> Neighbors(int id)
    {
        List<int> aux = new List<int>();
        Dictionary<int, Neighbor> neighbors = Graph[id].NeighborsDic;
        foreach(KeyValuePair<int, Neighbor> entry in neighbors)
        {
            aux.Add(entry.Key);
        }
        return aux;
    }
    public Vector3 triangleBaricenter(int currentTriangle)
    {
        return Graph[currentTriangle].Getbarycenter();
    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
