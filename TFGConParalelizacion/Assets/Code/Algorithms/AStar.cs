using System;
using System.Collections;
using Unity.Mathematics;
using System.Collections.Generic;
using System.Collections.Specialized;
using UnityEngine;
using UnityEngine.AI;

public class AStar : MonoBehaviour
{
    
    public Dictionary<int, Node> Graph;
    public Dictionary<int, Node> invertedPath;
    public SortedDictionary<int, float> gScore, fScore;
    public HashSet<int> closedSet, openSet;
    public List<SMBWaypoint> WayPointsPath;
    Vector3 origin, destination;
    public NavMeshTriangulation triangulation;
    Material mat;
    public static int count = 0;
    public AStar()
    {
        this.triangulation = NavMesh.CalculateTriangulation();
        GraphGenerator graphG = new GraphGenerator(triangulation);
        WayPointsPath = new List<SMBWaypoint>();
        this.Graph = graphG.Graph;
        closedSet = new HashSet<int>();
        openSet = new HashSet<int>();
        gScore = new SortedDictionary<int, float>();
        fScore = new SortedDictionary<int, float>();
        invertedPath = new Dictionary<int, Node>();
        foreach (KeyValuePair<int, Node> node in Graph)
        {
            gScore[node.Key] = float.PositiveInfinity;
            fScore[node.Key] = float.PositiveInfinity;
        }
    }

    static double area(int x1, int y1, int x2,
                       int y2, int x3, int y3)
    {
        return System.Math.Abs((x1 * (y2 - y3) +
                         x2 * (y3 - y1) +
                         x3 * (y1 - y2)) / 2.0);
    }

    private float heuristicCost(Vector3 A)
    {
        return Vector3.Distance(A, destination) ;
    }

    private int getTheMinimum()
    {
        int res = -1;
        float min = float.PositiveInfinity;
        foreach(int id in openSet)
        {
            if (fScore[id] < min)
            {
                min = fScore[id];
                res = id;
            }
        }
        return res;
    }

    private int initialNode()
    {
        Vector2 position = new Vector2(origin.x, origin.z);
        foreach (KeyValuePair<int, Node> node in Graph)
        {
            //if (intersect(node.Value.triangle, position)) return node.Key;
            Ray r = new Ray(new Vector3(origin.x, 100, origin.z), new Vector3(0, -1, 0));
            if (Intersect(node.Value.triangle[0], node.Value.triangle[1], node.Value.triangle[2], r)) return node.Key;
        }
        return 0;
    }

    public bool Intersect(Vector3 p1, Vector3 p2, Vector3 p3, Ray ray)
    {
        // Vectors from p1 to p2/p3 (edges)
        Vector3 e1, e2;

        Vector3 p, q, t;
        float det, invDet, u, v;


        //Find vectors for two edges sharing vertex/point p1
        e1 = p2 - p1;
        e2 = p3 - p1;

        // calculating determinant 
        p = Vector3.Cross(ray.direction, e2);

        //Calculate determinat
        det = Vector3.Dot(e1, p);

        //if determinant is near zero, ray lies in plane of triangle otherwise not
        if (det > -Mathf.Epsilon && det < Mathf.Epsilon) { return false; }
        invDet = 1.0f / det;

        //calculate distance from p1 to ray origin
        t = ray.origin - p1;

        //Calculate u parameter
        u = Vector3.Dot(t, p) * invDet;

        //Check for ray hit
        if (u < 0 || u > 1) { return false; }

        //Prepare to test v parameter
        q = Vector3.Cross(t, e1);

        //Calculate v parameter
        v = Vector3.Dot(ray.direction, q) * invDet;

        //Check for ray hit
        if (v < 0 || u + v > 1) { return false; }

        if ((Vector3.Dot(e2, q) * invDet) > Mathf.Epsilon)
        {
            //ray does intersect
            return true;
        }

        // No hit at all
        return false;
    }

    private int LastNode()
    {
        Vector2 position = new Vector2(destination.x, destination.z);
        foreach (KeyValuePair<int, Node> node in Graph)
        {

            //if (intersect(node.Value.triangle, position)) return node.Key;
            Ray r = new Ray(new Vector3(destination.x, 100, destination.z), new Vector3(0, -1, 0));
            if (Intersect(node.Value.triangle[0], node.Value.triangle[1], node.Value.triangle[2], r)) return node.Key;
        }
        return -1;
    }

    private bool intersect(List<Vector3> triangle, Vector2 position)
    {
        int x1 = (int)triangle[0].x;
        int x2 = (int)triangle[1].x;
        int x3 = (int)triangle[2].x;
        int z1 = (int)triangle[0].z;
        int z2 = (int)triangle[1].z;
        int z3 = (int)triangle[2].z;
        int x = (int)position.x;
        int z = (int)position.y;

        /* Calculate area of triangle ABC */
        double A = area(x1, z1, x2, z2, x3, z3);

        /* Calculate area of triangle PBC */
        double A1 = area(x, z, x2, z2, x3, z3);

        /* Calculate area of triangle PAC */
        double A2 = area(x1, z1, x, z, x3, z3);

        /* Calculate area of triangle PAB */
        double A3 = area(x1, z1, x2, z2, x, z);

        /* Check if sum of A1, A2 and A3 is same as A */
        return (A == A1 + A2 + A3);
    }
    public void setOrigin(Vector3 origin)
    {
        this.origin = origin;
    }

    public void setDestination(Vector3 destination)
    {
        this.destination = destination;
    }
    

    public List<Vector3> reconstructTrianglePath(int current)
    {
        List<Vector3> trianglePath = new List<Vector3>();
        
        trianglePath.AddRange(Graph[current].triangle);
        while (invertedPath.ContainsKey(current))
        {
            current = invertedPath[current].id;
            trianglePath.InsertRange(0,Graph[current].triangle);
        }
        return trianglePath;
    }

    public void cleanStructures()
    {
        closedSet = new HashSet<int>();
        openSet = new HashSet<int>();
        WayPointsPath.Clear();
        gScore = new SortedDictionary<int, float>();
        fScore = new SortedDictionary<int, float>();
        invertedPath = new Dictionary<int, Node>();
        foreach (KeyValuePair<int, Node> node in Graph)
        {
            gScore[node.Key] = float.PositiveInfinity;
            fScore[node.Key] = float.PositiveInfinity;
        }
    }

    public List<int> reconstructTrianglePath2(int current)
    {
        List<int> trianglePath = new List<int>();
        trianglePath.Add(current);
        WayPointsPath.Add(new SMBWaypoint { start = destination, end = destination });
        int old = current;
        while (invertedPath.ContainsKey(current))
        {
            current = invertedPath[current].id;
            trianglePath.Insert(0, current);
            
            List<Vector3> adjPoints = Graph[old].NeighborsDic[current].getAdjPoints();
            WayPointsPath.Insert(0, new SMBWaypoint { start = adjPoints[0], end = adjPoints[1] });
            old = current;
       }
        return trianglePath;
    }
    public List<SMBWaypoint> getWayPoints()
    {
        return WayPointsPath;
    }
    /*public List<Vector3> IntersectedTriangles(int triangleA, int triangleB)
    {
        List<Neighbor> neighbors = Graph[triangleA].Neighbors;
        List<Vector3> aux = new List<Vector3>();
        List<Vector2> segment;
        segment.Add(new Vector2());
        segment.Add(new Vector2());
        foreach (Neighbor nei in neighbors)
        {
            if (nei.NeighborID() != triangleB && triangleIntersectSegment(nei.triangle, Vector2)) aux.AddRange(nei.triangle());
        }
        return aux;
    }*/

    public float distanceBetweenNodes(int act, Neighbor nei)
    {
        float penalitation = nei.occupation;
        if (1 > penalitation) penalitation = 1;
        return Vector3.Distance(Graph[act].Getbarycenter(), nei.neighbor.Getbarycenter());
        
    }
    public List<Vector3> trianglePath()
    {
        int start = initialNode();
        Debug.Log(Graph[0].barycenter);
        Debug.Log(origin);
        int last = LastNode();
        Debug.Log(last);
        Graph[start].actualPosition = origin;
        openSet.Add(start);
        gScore[start] = 0;
        fScore[start] = heuristicCost(Graph[start].actualPosition);
        int count = 0;
        while (openSet.Count > 0)
        {
            int current = getTheMinimum();
            if (current == last)
            {
                return reconstructTrianglePath(current);
            }
            openSet.Remove(current);
            closedSet.Add(current);
            Dictionary<int, Neighbor> neighbors = Graph[current].NeighborsDic;
            foreach (KeyValuePair<int, Neighbor> entry in neighbors)
            {
                Neighbor neigh = entry.Value;
                int neighID = neigh.NeighborID();
                if (closedSet.Contains(neighID)) continue;

                float potencialNewScore = gScore[current] + distanceBetweenNodes(current, neigh);
                if (!openSet.Contains(neighID)) openSet.Add(neighID);
                else if (potencialNewScore >= gScore[neighID]) continue;
                // This path is the best until now. Record it!
                invertedPath[neighID] = Graph[current];
                gScore[neighID] = potencialNewScore;
                Graph[neighID].actualPosition = Graph[neighID].potencialPosition;
                fScore[neighID] = gScore[neighID] + heuristicCost(Graph[neighID].barycenter);
            }
            ++count;
        }
        return new List<Vector3>();
    }

    public List<int> trianglePath2()
    {
        int start = initialNode();
       
        int last = LastNode();
        Graph[start].actualPosition = origin;
        openSet.Add(start);
        gScore[start] = 0;
        fScore[start] = heuristicCost(Graph[start].actualPosition);
        int count = 0;
        while (openSet.Count > 0)
        {
            int current = getTheMinimum();
            if (current == last)
            {
                return reconstructTrianglePath2(current);
            }
            openSet.Remove(current);
            closedSet.Add(current);
            Dictionary<int,Neighbor> neighbors = Graph[current].NeighborsDic;
            foreach (KeyValuePair<int, Neighbor> entry in neighbors)
            {
                Neighbor neigh = entry.Value;
                int neighID = neigh.NeighborID();
                if (closedSet.Contains(neighID)) continue;
                float potencialNewScore = gScore[current] + distanceBetweenNodes(current, neigh);
                if (!openSet.Contains(neighID)) openSet.Add(neighID);
                else if (potencialNewScore >= gScore[neighID]) continue;
                // This path is the best until now. Record it!
                invertedPath[neighID] = Graph[current];
                gScore[neighID] = potencialNewScore;
                Graph[neighID].actualPosition = Graph[neighID].potencialPosition;
                fScore[neighID] = gScore[neighID] + heuristicCost(Graph[neighID].barycenter);
            }
            ++count;
        }
        return new List<int>();
    }

    public List<int> recalculatePath(Vector3 origin, Vector3 destination)
    {
        setOrigin(origin);
        setDestination(destination);
        closedSet = new HashSet<int>();
        openSet = new HashSet<int>();
        gScore = new SortedDictionary<int, float>();
        fScore = new SortedDictionary<int, float>();
        invertedPath = new Dictionary<int, Node>();
        foreach (KeyValuePair<int, Node> node in Graph)
        {
            gScore[node.Key] = float.PositiveInfinity;
            fScore[node.Key] = float.PositiveInfinity;
        }
        return trianglePath2();
    }

    public void modifyCost(int current, int neiID, int occupation)
    {
        Graph[current].NeighborsDic[neiID].occupation = occupation;
        Graph[neiID].NeighborsDic[current].occupation = occupation;
    }

    public void paintGraph()
    {
        List<Color> colors = new List<Color>();
        List<int> triangles = new List<int>();
        int count = 0;
        foreach(KeyValuePair<int, Node> entry in Graph)
        {
            DrawLine(entry.Value.triangle[0], entry.Value.triangle[1], Color.white);
            DrawLine(entry.Value.triangle[1], entry.Value.triangle[2], Color.white);
            DrawLine(entry.Value.triangle[2], entry.Value.triangle[0], Color.white);
            triangles.Add(count);
            triangles.Add(count + 1);
            triangles.Add(count + 2);
            count += 3;
        }
    }
    void DrawLine(Vector3 start, Vector3 end, Color color)
    {
        GameObject myLine = new GameObject();
        myLine.name = "Line";
       // myLine.transform.parent = ListLines.transform;
        myLine.transform.position = start;
        myLine.AddComponent<LineRenderer>();
        LineRenderer lr = myLine.GetComponent<LineRenderer>();
        lr.material = mat;
        lr.startColor = color;
        lr.endColor = color;
        lr.startWidth = 0.5f;
        lr.endWidth = 0.5f;
        lr.SetPosition(0, start);
        lr.SetPosition(1, end);
    }
    public void paintTriangleNeigh(int id, Color color1, Color color2)
    {
        DrawLine(Graph[id].triangle[0], Graph[id].triangle[1], color1);
        DrawLine(Graph[id].triangle[1], Graph[id].triangle[2], color1);
        DrawLine(Graph[id].triangle[2], Graph[id].triangle[0], color1);
        Dictionary<int, Neighbor> nei = Graph[id].NeighborsDic;
        foreach(KeyValuePair<int, Neighbor> entry in nei)
        {
            DrawLine(entry.Value.neighbor.triangle[0], entry.Value.neighbor.triangle[1], color2);
            DrawLine(entry.Value.neighbor.triangle[1], entry.Value.neighbor.triangle[2], color2);
            DrawLine(entry.Value.neighbor.triangle[2], entry.Value.neighbor.triangle[0], color2);
        }
    }
}
