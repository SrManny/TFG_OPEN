using System;
using System.Collections;
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
    Vector3 origin, destination;
    public NavMeshTriangulation triangulation;
    Material mat;
    public AStar()
    {
        this.triangulation = NavMesh.CalculateTriangulation();
        GraphGenerator graphG = new GraphGenerator(triangulation);
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
        return -1;
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

    public void cleanStructures()
    {
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

    public List<Vector3> reconstructTrianglePath(int current)
    {
        List<Vector3> trianglePath = new List<Vector3>();
        trianglePath.AddRange(Graph[current].triangle);
         while (invertedPath.ContainsKey(current))
        {
            current = invertedPath[current].id;
            trianglePath.InsertRange(0,Graph[current].triangle);
            //if (invertedPath.ContainsKey(invertedPath[current].id)) trianglePath.InsertRange(0, IntersectedTriangles(current, invertedPath[current].id));
            //Debug.Log(current + "con un coste gScore " + gScore[current] + "y el coso de fScore " + fScore[current] + " Y ADEMAS QUIERO SABER PORQUE PUNTO PASO " + Graph[current].actualPosition);
        }
        return trianglePath;
    }

    public List<int> reconstructTrianglePath2(int current)
    {
        List<int> trianglePath = new List<int>();
        trianglePath.Add(current);
        //if (invertedPath.ContainsKey(invertedPath[current].id)) trianglePath.AddRange(IntersectedTriangles(current, invertedPath[current].id));
        //Debug.Log(current + "con un coste gScore " + gScore[current] + "y el coso de fScore " + fScore[current] + " Y ADEMAS QUIERO SABER PORQUE PUNTO PASO " + Graph[current].actualPosition);
        while (invertedPath.ContainsKey(current))
        {
            current = invertedPath[current].id;
            trianglePath.Insert(0, current);
            //if (invertedPath.ContainsKey(invertedPath[current].id)) trianglePath.InsertRange(0, IntersectedTriangles(current, invertedPath[current].id));
            //Debug.Log(current + "con un coste gScore " + gScore[current] + "y el coso de fScore " + fScore[current] + " Y ADEMAS QUIERO SABER PORQUE PUNTO PASO " + Graph[current].actualPosition);
        }
        return trianglePath;
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
    
            float x = Graph[act].actualPosition.x;
            float y = Graph[act].actualPosition.z;
            float x1 = nei.getAdjPoints()[0].x;
            float y1 = nei.getAdjPoints()[0].z;
            float x2 = nei.getAdjPoints()[1].x;
            float y2 = nei.getAdjPoints()[1].z;

            float A = x - x1;
            float B = y - y1;
            float C = x2 - x1;
            float D = y2 - y1;

            float dot = A * C + B * D;
            float len_sq = C * C + D * D;
            float param = -1;
            if (len_sq != 0) //in case of 0 length line
                param = dot / len_sq;

            float xx, yy;

            if (param < 0)
            {
                xx = x1;
                yy = y1;
            }
            else if (param > 1)
            {
                xx = x2;
                yy = y2;
            }
            else
            {
                xx = x1 + param * C;
                yy = y1 + param * D;
            }

            var dx = x - xx;
            var dy = y - yy;
            Vector3 aux = new Vector3(xx, 20.1f, yy);
            //nei.neighbor.potencialPosition = aux;
            Graph[nei.NeighborID()].potencialPosition = aux;
            return (float) Math.Sqrt(dx * dx + dy * dy)*penalitation;
    }
    public List<Vector3> trianglePath()
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
            //Debug.Log("De momento tenemos que estamos mirando " + current);
            //foreach (int k in openSet) Debug.Log(gScore[k] + " del nodo " + k);
            if (current == last)
            {
                return reconstructTrianglePath(current);//reconstructTrianglePath(current);
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
                fScore[neighID] = gScore[neighID] + heuristicCost(Graph[neighID].actualPosition);
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
        //La pos
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
                fScore[neighID] = gScore[neighID] + heuristicCost(Graph[neighID].actualPosition);
            }
            ++count;
        }
        Debug.Log(count);
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
        lr.startWidth = 0.25f;
        lr.endWidth = 0.25f;
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
