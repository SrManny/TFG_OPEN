using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
    // Start is called before the first frame update
    public int id;
    public Vector3 actualPosition, potencialPosition;
    public List<Vector3> triangle;
    public List<Neighbor> Neighbors;
    public Dictionary<int, Neighbor> NeighborsDic;
    public Neighbor next;
    public Vector3 barycenter;


    public Node(int id, List<Vector3> triangle)
    {
        this.id = id;
        this.triangle = triangle;
        this.barycenter = (triangle[0] + triangle[1] + triangle[2])/3;
        Neighbors = new List<Neighbor>();
        NeighborsDic = new Dictionary<int, Neighbor>();
        next = null;
    }

    public void AddNeigh(Neighbor nei)
    {
        Neighbors.Add(nei);
        NeighborsDic[nei.NeighborID()] = nei;
    }
    
    public void AddNeighBors(Dictionary<int, Neighbor> neighbors)
    {
        foreach(KeyValuePair<int, Neighbor> entry in neighbors)
        {
            NeighborsDic[entry.Key] = entry.Value;
        }
    }

    public  Vector3 Getbarycenter()
    {
        return barycenter;
    }

    public bool anyNeighborFree()
    {
        foreach (Neighbor nei in Neighbors)
        {
            if (nei.anyFreeWayPoint()) return true;
        }
        return false;
    }

    public void incrementCostOfNei(int neiID)
    {
        Neighbors[neiID].ModifiedCost = true;
    }
}
