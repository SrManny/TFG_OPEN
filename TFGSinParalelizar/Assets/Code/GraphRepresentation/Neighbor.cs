using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Neighbor
{
    public Node neighbor;
    public bool byVertex, ModifiedCost;
    public float K;
    public List<Vector3> AdjPoints;
    public Dictionary<int, Vector3> WayPoints;
    public HashSet<int> FreeWayPoints;
    float radius = 0.6f;
    public int occupation = 0;
    Vector2 StartPoint, EndPoint;

    public Neighbor(Node neighbor, bool byVertex, List<Vector3> AdjPoints)
    {
        this.neighbor = neighbor;
        this.byVertex = byVertex;
        this.AdjPoints = AdjPoints;
        this.WayPoints = new Dictionary<int, Vector3>();
        this.FreeWayPoints = new HashSet<int>();
        StartPoint = new Vector2(AdjPoints[0].x, AdjPoints[0].z);
        EndPoint = new Vector2(AdjPoints[1].x, AdjPoints[1].z);
        K = Vector2.Distance(StartPoint, EndPoint) / (2 * radius);
        ModifiedCost = false;
    }
    private void generateWayPoint()
    {
        StartPoint = new Vector2(AdjPoints[0].x, AdjPoints[0].z);
        EndPoint = new Vector2(AdjPoints[1].x, AdjPoints[1].z);
        K = Vector2.Distance(StartPoint, EndPoint)/ (2*radius);
        for (int i = 0; i <= K; ++i)
        {
            WayPoints.Add(i, new Vector3(StartPoint.x + (i / K) * (EndPoint.x - StartPoint.x), AdjPoints[0].y, (StartPoint.y + (i / K) * (EndPoint.y - StartPoint.y))));
            FreeWayPoints.Add(i);
        }

    }
    public bool isConnedtedByVertex()
    {
        return byVertex;
    }

    public List<Vector3> getAdjPoints()
    {
        return AdjPoints;
    }

    public int NeighborID()
    {
        return neighbor.id;
    }

    public List<Vector3> triangle()
    {
        return neighbor.triangle;
    }

    public override bool Equals(object obj)
    {
        if (obj == null) return false;
        Neighbor objAsNeighbor = obj as Neighbor;
        if (objAsNeighbor == null) return false;
        else return Equals(objAsNeighbor);
    }

    public bool Equals(Neighbor other)
    {
        if (other == null) return false;
        return (this.neighbor.id.Equals(other.neighbor.id));
    }

    public Vector3 getClosestWayPoint(Vector3 position, out int pointID)
    {
        int index = -1;
        float minDist = float.PositiveInfinity;
        for (int i = 0; i < WayPoints.Count; ++i)
        {
            if (FreeWayPoints.Contains(i))
            {
                float aux = Vector3.Distance(position, WayPoints[i]);
                if (minDist > aux)
                {
                    minDist = aux;
                    index = i;
                }
            }
        }
        this.FreeWayPoints.Remove(index);
        pointID = index;
        return WayPoints[index];
    }

    public void liberateWayPoint(int pointID)
    {
        this.FreeWayPoints.Add(pointID);
    }
    public float distanceToEdge(Vector3 position)
    {
        return 0;
    }
    public Vector3 getProyectedWayPoint(Vector3 position)
    {
        float x = position.x;
        float y = position.z;
        float x1 = AdjPoints[0].x;
        float y1 = AdjPoints[0].z;
        float x2 = AdjPoints[1].x;
        float y2 = AdjPoints[1].z;

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
            xx = x2;
            yy = y2;
        }
        else if (param > 1)
        {
            xx = x1;
            yy = y1;
        }
        else
        {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }
        return new Vector3(xx, position.y, yy);
    }
    public bool isFree(int id)
    {
        return this.FreeWayPoints.Contains(id);
    }

    public void increaseOccupation()
    {
        ++occupation;
    }

    public void decreaseOccupation()
    {
        --occupation;
    }
    public bool isTooBusy()
    {
        return false;//occupation/3f > Vector2.Distance(StartPoint, EndPoint) / (2 * radius);
    }
    public bool anyFreeWayPoint()
    {
        return (this.FreeWayPoints.Count > 0);
    }

}
