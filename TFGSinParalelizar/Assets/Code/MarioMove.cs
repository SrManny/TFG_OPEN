using System.Collections;
using System.Collections.Generic;

using UnityEngine;
using UnityEngine.AI;


public class MarioMove : MonoBehaviour
{
    //[SerializeField]

    public Transform _destination;
    public float funnelMAX, Mindistance;
    public GameObject ListLines, parent, LinesOfPath;
    public List<Collider> hitColliders;
    public Animator _animator;
    public Material material, mat;
    //public int ola = 0, ocu;
    public bool desvio = false, dentrito = false;
    public Vector3 origin, center;
    NavMeshAgent _navMeshAgent;
    public string neighBorID = ".";
    public int longitudFunel = 2;
    public NavMeshTriangulation triangulization;
    public Dictionary<int, Node> Graph;
    public int NumTri = 19, indexPath = 0;
    List<Vector3> trianglePath;
    public AStar astar;
    public Vector3 finalDir;
    bool MarioHasDestination, Lastmovement, FirsTriangle, stopped;
    public Vector3 NextPoint;
    public float angless, DistanceToNext = 0;
    int recalculate = 0;
    public int currentTriangle, vecinoID;
    public List<int> vecinosID;
    public List<int> trianglePathID;
    public int ClosestMarioIndex, countCol = 0;
    FunelScript col;
    public double A, total;
    public float Speed;//Don't touch this
    float MaxSpeed;//This is the maximum speed that the object will achieve
    float Acceleration;//How fast will object reach a maximum speed
    float Deceleration;//How fast will object reach a speed of 0
    GraphData script;
    bool m_Started;
    public float firsty;
    public float directionW;

    // Start is called before the first frame update
    void Start()
    {
        m_Started = true;
        _navMeshAgent = this.GetComponent<NavMeshAgent>();
        _animator = this.GetComponent<Animator>();
        ListLines = new GameObject();
        LinesOfPath = new GameObject();
        LinesOfPath.name = "LinesOfPath";
        ListLines.transform.parent = this.transform.parent;
        ListLines.name = "ListLines";
        finalDir = new Vector3();
        MarioHasDestination = false;
        Lastmovement = false;
        FirsTriangle = true;
        stopped = false;
        NextPoint = new Vector3();
        Speed = 0;
        MaxSpeed = 7;
        Acceleration = 10;
        Deceleration = 10;
        currentTriangle = -1;
        LinesOfPath.transform.parent = transform;
        script = GetComponentInParent<GraphData>();
        origin = transform.position;
        col = GetComponentInChildren<FunelScript>();
        firsty = 0f;
    }

    private void SetDestination()
    {
        if (_destination != null)
        {
            Vector3 targetVector = _destination.transform.position;
            _navMeshAgent.SetDestination(targetVector);
        }
    }

    private void SetDestination2()
    {
        if (_destination != null)
        {
            Vector3 targetVector = _destination.transform.position;
            MarioHasDestination = true;
            astar.cleanStructures();
            astar.setOrigin(transform.position);
            astar.setDestination(_destination.position);
            trianglePathID = astar.trianglePath2();
            currentTriangle = trianglePathID[indexPath];
        }
    }


    public void paintPath(List<Vector3> trianglePath, Color color1)
    {
     
        List<Color> colors = new List<Color>();
        List<int> triangles = new List<int>();
        int count = 0;
        for (int j = 0; j < trianglePath.Count; j += 3)
        {
            colors.Add(color1);
            colors.Add(color1);
            colors.Add(color1);
            DrawLine(trianglePath[j], trianglePath[j + 1], Color.white);
            DrawLine(trianglePath[j+1], trianglePath[j + 2], Color.white);
            DrawLine(trianglePath[j+2], trianglePath[j], Color.white);
            triangles.Add(count);
            triangles.Add(count + 1);
            triangles.Add(count + 2);
            count += 3;
        }
    }

    private int initialNode()
    {
        Vector2 position = new Vector2(transform.position.x, transform.position.z);
        foreach (KeyValuePair<int, Node> node in Graph)
        {

            Ray r = new Ray(new Vector3(position.x, 100, position.y), new Vector3(0, -1, 0));
            if (intersect(node.Value.triangle, r)) return node.Key;
        }
        return -1;
    }
    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            
            SetDestination2();
            //Graphics.DrawMesh(path, Vector3.zero, Quaternion.identity, material, 0);
            //Debug.Log(triangulization.areas);
        }
        else if (Input.GetKeyDown(KeyCode.B))
        {
            astar.paintTriangleNeigh(initialNode(), Color.black, Color.red);
        }
        else if (Input.GetKeyDown(KeyCode.V))
        {
            Debug.Log(this.transform.position);
            Debug.Log(_destination.position);
            astar.setOrigin(this.transform.position);
            astar.setDestination(_destination.position);
            paintPath(astar.trianglePath(), Color.red);
        }
        else if (Input.GetKeyDown(KeyCode.C))
        {
            astar.paintGraph();
        }
        if (MarioHasDestination) GoToNextPoint();
        _animator.SetFloat("Speed", Speed*1.5f / MaxSpeed);
    }

    private void GoToNextPoint()
    {
        
        if (Lastmovement && Vector3.Distance(NextPoint, transform.position) < 2)
        {
            MarioHasDestination = false;
        }
        else if (Lastmovement && Vector3.Distance(NextPoint, transform.position) >= 2)
        {
            move();
        }
        else if ((FirsTriangle == true || Vector3.Distance(NextPoint, transform.position) < 3) || Graph[trianglePathID[indexPath-1]].NeighborsDic[trianglePathID[indexPath]].distanceToEdge(transform.position)< 0)
        {
            AsignNewPoint();
        }
        else if (!Lastmovement && recalculate > 8)
        {
            recalculate = 0;
            recalculateWayPoint(transform.position);
        }
        else
        {
            move();
            ++recalculate;

        }
    }

    private void AsignNewPoint()
    {
        
        
        if (indexPath + 1 < trianglePathID.Count)
        {
            vecinoID = trianglePathID[indexPath + 1];
            vecinosID = script.Neighbors(currentTriangle);

            if (script.isTooBusy(currentTriangle, trianglePathID[indexPath+1]))
            {
                bool anyFree = false;
                foreach(int id in vecinosID)
                {
                    if (!script.isTooBusy(currentTriangle, id))
                    {
                        anyFree = true;
                        break;
                    }
                }
                //Recalcular Path
                /*if (anyFree)
                {
                    desvio = true;
                    int nextTriangle = trianglePathID[indexPath + 1];
                    if (!FirsTriangle)
                    {
                        currentTriangle = trianglePathID[indexPath - 1];
                        nextTriangle = trianglePathID[indexPath];
                        script.liberateOccupation(trianglePathID[indexPath - 1], trianglePathID[indexPath]);
                    }
                    FirsTriangle = true;
                    trianglePathID = script.recalculatePath(currentTriangle, nextTriangle, _destination.position);
                    indexPath = 0;
                    currentTriangle = trianglePathID[indexPath];
                    DistanceToNext = Vector3.Distance(script.triangleBaricenter(currentTriangle), _destination.position);
                }*/
            }
            else
            {
                desvio = false;
                if (!FirsTriangle) script.liberateOccupation(trianglePathID[indexPath - 1], trianglePathID[indexPath]);
                NextPoint = script.getProyectedWayPoint(currentTriangle, transform.position, trianglePathID[indexPath + 1]);
                //DrawLinesOfPath(transform.position, NextPoint, Color.grey);
                FirsTriangle = false;
                ++indexPath;
                currentTriangle = trianglePathID[indexPath];
                DistanceToNext = Vector3.Distance(script.triangleBaricenter(currentTriangle), _destination.position);
                stopped = false;
            }
        }
        else
        {
            script.liberateOccupation(trianglePathID[indexPath - 1], trianglePathID[indexPath]);
            Lastmovement = true;
            NextPoint = _destination.transform.position;
        }
    }

    public void recalculateWayPoint(Vector3 aux)
    {
        //Destroy(LinesOfPath.transform.GetChild(LinesOfPath.transform.childCount - 1).gameObject);
        script.liberateOccupation(trianglePathID[indexPath - 1], trianglePathID[indexPath]);
        NextPoint = script.getProyectedWayPoint(trianglePathID[indexPath - 1], aux, trianglePathID[indexPath]);
        //Debug.DrawLine(transform.position, NextPoint, Color.black);
        //DrawLinesOfPath(transform.position, NextPoint, Color.grey);
        stopped = false;
    }
    
    private void move()
    {
        Mindistance = Mathf.Infinity;
        hitColliders = col.GetColliders();  //Physics.OverlapBox(new Vector3(transform.position.x, transform.position.y + transform.position.y / 10, transform.position.z + transform.localScale.z / longitudFunel), new Vector3(transform.localScale.x * 0.25f*0.5f, transform.localScale.y * 0.5f*0.5f, 0.5f*transform.localScale.z / longitudFunel), Quaternion.identity);
        ClosestMarioIndex = -1;
        string name = "";
        Vector3 ClosestPoint = new Vector3();
        int i = 0, j = 0;
        Vector3 oldPosition = transform.position;
        //Check when there is a new collider coming into contact with the box
        if (!Lastmovement)
        {
            while (i < hitColliders.Count)
            {
                if (hitColliders[i].tag == "Mario" || hitColliders[i].tag == "Tower")
                {
                    Vector3 auxClosestPoint = hitColliders[i].ClosestPoint(transform.position);
                    float Aux = Vector3.Distance(transform.position, hitColliders[i].gameObject.transform.position);
                    if (Mindistance > Aux)//Vector3.Distance(transform.position, auxClosestPoint))
                    {
                        Mindistance = Aux;
                        ClosestMarioIndex = i;
                        ClosestPoint = auxClosestPoint;
                        name = hitColliders[i].name;
                    }
                    ++j;
                }
                ++i;
            }
        }
        if (j != 0)
        {
            Vector3 normal = new Vector3();
            Vector3 tangent1 = new Vector3();
            Vector3 tangent2 = new Vector3();
            Vector3 finalDirection = new Vector3();// = Vector3.Normalize(NextPoint-transform.position)*(1f-directionW)*10f + transform.position;
            float step = Speed * Time.deltaTime;
            //float r;
            float dot = Vector3.Dot(transform.forward, hitColliders[ClosestMarioIndex].gameObject.transform.forward);
            float ratio = (1 - ((Mindistance - 2.5f) / funnelMAX));
            if (hitColliders[ClosestMarioIndex].gameObject.tag == "Tower")
            {
                bool movement = false;
                normal = -Vector3.Normalize((ClosestPoint - transform.position));
                normal.y = normal.y - normal.y; //hitColliders[ClosestMarioIndex].gameObject.GetComponent<BoxCollider>().ClosestPoint(transform.position)
                if (Vector3.Distance(transform.position, ClosestPoint) > 1f)
                {
                    tangent1 = Vector3.Cross(normal, Vector3.down);
                    tangent2 = Vector3.Cross(normal, Vector3.up);
                    float aux1 = Vector3.Angle(tangent1, transform.forward);
                    float aux2 = Vector3.Angle(tangent2, transform.forward);
                    if (aux1 < aux2) normal = tangent1;
                    else normal = tangent2;
                }
                Vector3 aux = Vector3.MoveTowards(transform.position, finalDirection + Vector3.Normalize(normal) * directionW * 10, step * 1.5f);
                Ray r = new Ray(new Vector3(aux.x, 100, aux.z), new Vector3(0, -1, 0));
                if (indexPath > 1) dentrito = intersect(Graph[trianglePathID[indexPath - 2]].triangle, r);
                else dentrito = false;
                if (dentrito || intersect(Graph[trianglePathID[indexPath - 1]].triangle, r) || intersect(Graph[trianglePathID[indexPath]].triangle, r))
                {
                    transform.position = new Vector3(aux.x, firsty, aux.z);
                    Quaternion targetRotation = Quaternion.LookRotation(new Vector3(transform.position.x - oldPosition.x, 0, transform.position.z - oldPosition.z));
                    transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, 10 * 1 * Time.deltaTime);
                    movement = true;
                }
                r = new Ray(new Vector3(transform.position.x, 100, transform.position.z), new Vector3(0, -1, 0));
                if (indexPath > 1) dentrito = intersect(Graph[trianglePathID[indexPath - 2]].triangle, r);
                else dentrito = false;
                ++countCol;
                if (!movement && (!dentrito || !intersect(Graph[trianglePathID[indexPath - 1]].triangle, r) || !intersect(Graph[trianglePathID[indexPath]].triangle, r)))
                {
                    transform.position = Vector3.MoveTowards(transform.position, NextPoint, step);
                    Quaternion targetRotation = Quaternion.LookRotation(new Vector3(transform.position.x - oldPosition.x, 0, transform.position.z - oldPosition.z));
                    transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, 10 * 1 * Time.deltaTime);
                }//if (intersect(Graph[trianglePathID[indexPathAux]].triangle, new Vector2(aux.x, aux.z)) || intersect(Graph[trianglePathID[indexPathAux + 1]].triangle, new Vector2(aux.x, aux.z)))
                 //{
                 /*transform.position = new Vector3(aux.x, firsty, aux.z);
                     Quaternion targetRotation = Quaternion.LookRotation(transform.position - oldPosition);
                     transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, 5 * 1 * Time.deltaTime);*/
                 //}
                Debug.DrawRay(transform.position, finalDirection - transform.position + Vector3.Normalize(normal) * directionW * 10, Color.black);
            }
            else if (dot > 1)
            {
                Debug.Log("El dot vale " + dot);
                Debug.Log("El forward mio vale " + transform.forward);
                Debug.Log("El forward vecino vale " + hitColliders[ClosestMarioIndex].gameObject.transform.forward);
                if (j < 3 && ratio > 0.4f)
                {
                    if (Speed - 2 * Time.deltaTime > 3) Speed = Speed - 2 * Time.deltaTime;
                    else if (Speed <= 3) Speed = Speed + 3 * Time.deltaTime; ;
                }
                else if (j>=3)
                {
                    if (Speed - 2 * Time.deltaTime > 2) Speed = Speed - 2 * Time.deltaTime;
                    else if (Speed <= 2) Speed = Speed + 3 * Time.deltaTime; 
                }
                transform.position = Vector3.MoveTowards(transform.position, NextPoint, step);
                Quaternion targetRotation = Quaternion.LookRotation(transform.position - oldPosition);
                transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, 0.05f);
            }
            else if (hitColliders[ClosestMarioIndex].gameObject.tag == "Mario")// && dot < 0.5)
            {
                if (j > 1)
                {
                    if (Speed - 4 * Time.deltaTime > 3) Speed = Speed - 4 * Time.deltaTime;
                    else if (Speed <= 3) Speed = Speed + 2 * Time.deltaTime;
                }
                else { if (Speed + Time.deltaTime < MaxSpeed) Speed = Speed + Time.deltaTime; }
                center = hitColliders[ClosestMarioIndex].gameObject.transform.position;
                normal = Vector3.Normalize((transform.position - center));
                normal.y = normal.y - normal.y;
                tangent1 = Vector3.Cross(normal, Vector3.down);
                tangent2 = Vector3.Cross(normal, Vector3.up);
                float aux1 = Vector3.Angle(tangent1, transform.forward);
                float aux2 = Vector3.Angle(tangent2, transform.forward);
                Vector3 aux = new Vector3();
                if (aux1 > aux2) tangent1 = tangent2;
                if (ratio < 0) ratio = 0;
                else if (ratio > 1) ratio = 1;
                directionW = ratio;
                finalDirection = Vector3.Normalize(NextPoint - transform.position) * (1f - directionW) * 10f + transform.position;
                if (name.CompareTo(neighBorID) != 0 && (finalDir.x != 0 || finalDir.y != 0 || finalDir.z != 0)) finalDir = finalDir;
                else finalDir = Vector3.Normalize(tangent1) * directionW * 10f;

                Vector3 otherfinalDir = hitColliders[ClosestMarioIndex].gameObject.GetComponent<MarioMove>().finalDir;
                if (finalDir.x * otherfinalDir.x > 0 && finalDir.z * otherfinalDir.z > 0)
                {
                    
                    finalDir = -1f * finalDir;
                }
                finalDirection = finalDirection + finalDir;
                aux = Vector3.MoveTowards(transform.position, finalDirection, step);
                Ray r = new Ray(new Vector3(aux.x, 100, aux.z), new Vector3(0, -1, 0));
                if (indexPath > 1) dentrito = intersect(Graph[trianglePathID[indexPath - 2]].triangle, r);
                else dentrito = false;
                bool movement = false;
                ++countCol;
                if (dentrito || intersect(Graph[trianglePathID[indexPath - 1]].triangle, r) || intersect(Graph[trianglePathID[indexPath]].triangle, r))
                {
                    transform.position = new Vector3(aux.x, firsty, aux.z);
                    Quaternion targetRotation = Quaternion.LookRotation(new Vector3(transform.position.x - oldPosition.x, 0, transform.position.z - oldPosition.z));
                    transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, 0.02f);
                    movement = true;
                }
                r = new Ray(new Vector3(transform.position.x, 100, transform.position.z), new Vector3(0, -1, 0));
                if (indexPath > 1) dentrito = intersect(Graph[trianglePathID[indexPath - 2]].triangle, r);
                else dentrito = false;
                ++countCol;
                if (!movement && (!dentrito || !intersect(Graph[trianglePathID[indexPath - 1]].triangle, r) || !intersect(Graph[trianglePathID[indexPath]].triangle, r)))
                {
                    transform.position = Vector3.MoveTowards(transform.position, NextPoint, step);
                    Quaternion targetRotation = Quaternion.LookRotation(new Vector3(transform.position.x - oldPosition.x, 0, transform.position.z - oldPosition.z));
                    transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, 0.02f);//10 * 1 * Time.deltaTime);
                }
               neighBorID = name;
                //Debug.DrawRay(transform.position, finalDirection - transform.position, Color.black);

            }
            recalculateWayPoint(transform.position);
        }
        else
        {
            neighBorID = ".";
            finalDir = new Vector3();
            if (Speed + 3 * Time.deltaTime < MaxSpeed) Speed = Speed + 3 * Time.deltaTime;
            float step = Speed * Time.deltaTime;
            transform.position = Vector3.MoveTowards(transform.position, NextPoint, step);
            Quaternion targetRotation = Quaternion.LookRotation(transform.position - oldPosition);
            transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, 0.05f);
        }
    }
    void DrawLine(Vector3 start, Vector3 end, Color color)
    {
        GameObject myLine = new GameObject();
        myLine.name = "Line";
        myLine.transform.parent = ListLines.transform;
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

    void DrawLinesOfPath(Vector3 start, Vector3 end, Color color)
    {
        GameObject myLine = new GameObject();
        myLine.name = "LineOfPointAsigned";
        myLine.transform.parent = LinesOfPath.transform;
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


    public bool intersect(List<Vector3> triangle, Ray ray)
    {
        Vector3 p1 = triangle[0];
        Vector3 p2 = triangle[1];
        Vector3 p3 = triangle[2];
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

    private bool Intersect(List<Vector3> triangle, Vector2 position)
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
        A = area(x1, z1, x2, z2, x3, z3);

        /* Calculate area of triangle PBC */
        double A1 = area(x, z, x2, z2, x3, z3);

        /* Calculate area of triangle PAC */
        double A2 = area(x1, z1, x, z, x3, z3);

        /* Calculate area of triangle PAB */
        double A3 = area(x1, z1, x2, z2, x, z);
        total = A1 + A2 + A3;
        /* Check if sum of A1, A2 and A3 is same as A */
        return (A == A1 + A2 + A3);
    }
    static double area(int x1, int y1, int x2,
                      int y2, int x3, int y3)
    {
        return System.Math.Abs((x1 * (y2 - y3) +
                         x2 * (y3 - y1) +
                         x3 * (y1 - y2)) / 2.0);
    }

    /*void OnDrawGizmos()
    {
        GL.PushMatrix();
        Gizmos.color = Color.red;
        Matrix4x4 rotationMatrix = Matrix4x4.TRS(new Vector3(transform.position.x, transform.position.y+100000, transform.position.z + transform.localScale.z / longitudFunel), Quaternion.Euler(0,0,0), new Vector3());
        //GL.MultMatrix(rotationMatrix);
        Vector3 position1 = new Vector3(transform.position.x, transform.position.y + transform.position.y / 10, transform.position.z + transform.localScale.z / longitudFunel);
        Vector3 v = position1 - transform.position; //the relative vector from P2 to P1.
        float angles = Vector3.SignedAngle(v,new Vector3 (transform.forward.x, 0, transform.forward.z), Vector3.up);
        v = Quaternion.Euler(0, angles, 0) * v; //rotatate
        v = transform.position + v;
        //Check that it is being run in Play Mode, so it doesn't try to draw this in Editor mode
        Vector3 size = new Vector3(transform.localScale.x * 0.25f, transform.localScale.y * 0.5f, transform.localScale.z / longitudFunel);
        if (m_Started)
            //Draw a cube where the OverlapBox is (positioned where your GameObject is as well as a size)
            Gizmos.DrawWireCube(v, size);
        GL.PopMatrix();
    }*/
}

