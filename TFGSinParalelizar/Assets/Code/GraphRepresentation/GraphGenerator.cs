using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class GraphGenerator 
{
    NavMeshTriangulation triangulization;
    public Dictionary<int, Node> Graph;
    int[] indices;
    Vector3[] vertices;
    // Start is called before the first frame update
    public GraphGenerator(NavMeshTriangulation triangulization)
    {
        this.triangulization = triangulization;
        Graph = new Dictionary<int, Node>();
        indices = triangulization.indices;
        vertices = triangulization.vertices;
        GenerateGraph();
    }

    private void GenerateGraph()
    {
        for (int i = 0; i < indices.Length; i += 3)
        {
            List<Vector3> tmp = new List<Vector3>();
            int i1 = indices[i];
            int i2 = indices[i + 1];
            int i3 = indices[i + 2];
            tmp.Add(vertices[i1]);
            tmp.Add(vertices[i2]);
            tmp.Add(vertices[i3]);
            Node node = new Node(i / 3, tmp);
            Dictionary<int ,Neighbor> neighbors = FindNeighbors(i + 3, tmp);
            
            if (!Graph.ContainsKey(i / 3)) Graph.Add(i / 3, node);
            Graph[(i / 3)].AddNeighBors(neighbors);
            foreach (KeyValuePair<int, Neighbor> entry in neighbors)
            {
                if (!Graph.ContainsKey(entry.Key)) Graph.Add(entry.Key, entry.Value.neighbor);
                Graph[entry.Key].AddNeigh(new Neighbor(Graph[(i / 3)], entry.Value.isConnedtedByVertex(), entry.Value.getAdjPoints()));
            }
        }
    }

    private Dictionary<int, Neighbor> FindNeighbors(int index, List<Vector3> triangle2)
    {
        Dictionary<int, Neighbor> neighbors = new Dictionary<int, Neighbor>();
        for (int i = index; i < indices.Length; i += 3)
        {
            List<Vector3> triangle1 = new List<Vector3>();
            int i1 = indices[i];
            int i2 = indices[i + 1];
            int i3 = indices[i + 2];
            triangle1.Add(vertices[i1]);
            triangle1.Add(vertices[i2]);
            triangle1.Add(vertices[i3]);
            List<Vector3> AdjPoints = vertexShared(triangle1, triangle2, i/3, index/3);
            if (AdjPoints.Count > 0)
            {
                
                Node node = new Node(i / 3, triangle1);
                neighbors[node.id] = new Neighbor(node, false, AdjPoints);
            }
        }
        return neighbors;
    }

    private List<Vector3> vertexShared(List<Vector3> triangle1, List<Vector3> triangle2, int x, int index)
    {
        int count = 0;
        List<Vector3> points = new List<Vector3>();
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                
                if ((Mathf.Abs(triangle2[i].x - triangle1[j].x) < 0.0005) && (Mathf.Abs(triangle2[i].z - triangle1[j].z) < 0.0005))
                {
                    points.Add(triangle2[i]);
                    ++count;
                }
                if (count == 2) return points;
            }
        }
        return new List<Vector3>();
    }
}
