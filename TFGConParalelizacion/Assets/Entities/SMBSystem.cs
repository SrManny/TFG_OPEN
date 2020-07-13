using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Transforms;
using Unity.Mathematics;
using Unity.Entities;
using Unity.Burst;
using Unity.Jobs;
using System.Collections;

[UpdateBefore(typeof(TransformSystem))]
public class SMBSystem : JobComponentSystem
{

    //MOVER LOS ELEMENTOS DE DONDE ESTEN AL PUNTO FINAL
    private ComponentGroup SMBCharacterGroup;
    private ComponentGroup SMBColliderGroup;

    private Transform cameraTransform;
    private AStar astar = new AStar();
    Dictionary<int, Node> Graph; 
    public bool first = true;
    private List<SMBProperties> uniqueTypes = new List<SMBProperties>(10);
    private List<PreviousParticle> previousParticles = new List<PreviousParticle>();

    private List<int> Allpaths = new List<int>();
    private List<SMBWaypoint> wayPointsPath = new List<SMBWaypoint>();
    //Las posiciones en el espacio
    private static readonly int[] cellOffsetTable =
    {
        1, 1, 1, 1, 1, 0, 1, 1, -1, 1, 0, 1, 1, 0, 0, 1, 0, -1, 1, -1, 1, 1, -1, 0, 1, -1, -1,
        0, 1, 1, 0, 1, 0, 0, 1, -1, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, -1, 1, 0, -1, 0, 0, -1, -1,
        -1, 1, 1, -1, 1, 0, -1, 1, -1, -1, 0, 1, -1, 0, 0, -1, 0, -1, -1, -1, 1, -1, -1, 0, -1, -1, -1
    };



    private struct PreviousParticle
    {
        public NativeMultiHashMap<int, int> hashMap;
        public NativeArray<Position> particlesPosition;
        public NativeArray<SMBVelocity> particlesVelocity;
        public NativeArray<SMBDestination> particlesDestination;
        public NativeArray<SMBSspeed> particlesSspeed;
        public NativeArray<SMBPath> particlesindexPaths;
        public NativeArray<float3> particlesForces;
        public NativeArray<float> particlesPressure;
        public NativeArray<float> particlesDensity;
        public NativeArray<int> particleIndices;

        public NativeArray<int> cellOffsetTable;
        public NativeArray<SMBCollider> copyColliders;
    }

    public struct WayPoint
    {
        public float3 start;
        public float3 end;
    }

    public void Start()
    {
        Debug.Log(Graph[0].id);
        Debug.Log(Graph[1].id);
    }

    [BurstCompile]
    private struct ComputePaths : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Position> positions;
        [ReadOnly] public SMBDestination smbdestination;
        public NativeList<int> Allpaths;
        public AStar astar;

        public void Execute(int index)
        {
            astar.setOrigin(positions[index].Value);
            astar.setDestination(smbdestination.destination);
            int[] path = astar.trianglePath2().ToArray();
            NativeArray<int> aux = new NativeArray<int>();
            NativeArray<int>.Copy(path, aux);
            Allpaths.AddRange(aux);

        }
    }


    [BurstCompile]
    private struct HashPositions : IJobParallelFor
    {
        [ReadOnly] public float cellRadius;

        public NativeArray<Position> positions;
        public NativeMultiHashMap<int, int>.Concurrent hashMap;

        public void Execute(int index)
        {
            float3 position = positions[index].Value;
            int hash = GridHash.Hash(position, cellRadius);
            hashMap.Add(hash, index);

            positions[index] = new Position { Value = position };
        }
    }



    [BurstCompile]
    private struct MergeParticles : IJobNativeMultiHashMapMergedSharedKeyIndices
    {
        public NativeArray<int> particleIndices;



        public void ExecuteFirst(int index)
        {
            particleIndices[index] = index;
        }


        public void ExecuteNext(int cellIndex, int index)
        {
            particleIndices[index] = cellIndex;
        }
    }



    [BurstCompile]
    private struct ComputeDensityPressure : IJobParallelFor
    {
        [ReadOnly] public NativeMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<int> cellOffsetTable;
        [ReadOnly] public NativeArray<Position> particlesPosition;
        [ReadOnly] public SMBProperties settings;

        public NativeArray<float> densities;
        public NativeArray<float> pressures;

        private const float PI = 3.14159274F;
        private const float GAS_CONST = 2000.0f;



        public void Execute(int index)
        {
            // Cache
            int particleCount = particlesPosition.Length;
            float3 position = particlesPosition[index].Value;
            float density = 0.0f;
            int i, hash, j;
            int3 gridOffset;
            int3 gridPosition = GridHash.Quantize(position, settings.radius);
            bool found;

            // Find neighbors
            for (int oi = 0; oi < 27; oi++)
            {
                i = oi * 3;
                gridOffset = new int3(cellOffsetTable[i], cellOffsetTable[i + 1], cellOffsetTable[i + 2]);
                hash = GridHash.Hash(gridPosition + gridOffset);
                NativeMultiHashMapIterator<int> iterator;
                found = hashMap.TryGetFirstValue(hash, out j, out iterator);
                while (found)
                {
                    // Neighbor found, get density
                    float3 rij = particlesPosition[j].Value - position;
                    float r2 = math.lengthsq(rij);

                    if (r2 < settings.smoothingRadiusSq)
                    {
                        density += settings.mass * (315.0f / (64.0f * PI * math.pow(settings.smoothingRadius, 9.0f))) * math.pow(settings.smoothingRadiusSq - r2, 3.0f);
                    }

                    // Next neighbor
                    found = hashMap.TryGetNextValue(out j, ref iterator);
                }
            }

            // Apply density and compute/apply pressure
            densities[index] = density;
            pressures[index] = GAS_CONST * (density - settings.restDensity);
        }
    }



    [BurstCompile]
    private struct ComputeForces : IJobParallelFor
    {
        [ReadOnly] public NativeMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<int> cellOffsetTable;
        [ReadOnly] public NativeArray<Position> particlesPosition;
        [ReadOnly] public NativeArray<SMBVelocity> particlesVelocity;
        [ReadOnly] public NativeArray<float> particlesPressure;
        [ReadOnly] public NativeArray<float> particlesDensity;
        [ReadOnly] public SMBProperties settings;

        public NativeArray<float3> particlesForces;

        private const float PI = 3.14159274F;



        public void Execute(int index)
        {
            // Cache
            int particleCount = particlesPosition.Length;
            float3 position = particlesPosition[index].Value;
            float3 velocity = particlesVelocity[index].Value;
            float pressure = particlesPressure[index];
            float density = particlesDensity[index];
            float3 forcePressure = new float3(0, 0, 0);
            float3 forceViscosity = new float3(0, 0, 0);
            int i, hash, j;
            int3 gridOffset;
            int3 gridPosition = GridHash.Quantize(position, settings.radius);
            bool found;

            // Physics
            // Find neighbors
            for (int oi = 0; oi < 27; oi++)
            {
                i = oi * 3;
                gridOffset = new int3(cellOffsetTable[i], cellOffsetTable[i + 1], cellOffsetTable[i + 2]);
                hash = GridHash.Hash(gridPosition + gridOffset);
                NativeMultiHashMapIterator<int> iterator;
                found = hashMap.TryGetFirstValue(hash, out j, out iterator);
                while (found)
                {
                    // Neighbor found, get density
                    if (index == j)
                    {
                        found = hashMap.TryGetNextValue(out j, ref iterator);
                        continue;
                    }

                    float3 rij = particlesPosition[j].Value - position;
                    float r2 = math.lengthsq(rij);
                    float r = math.sqrt(r2);

                    if (r < settings.smoothingRadius)
                    {
                        forcePressure += -math.normalize(rij) * settings.mass * (2.0f * pressure) / (2.0f * density) * (-45.0f / (PI * math.pow(settings.smoothingRadius, 6.0f))) * math.pow(settings.smoothingRadius - r, 2.0f);

                        forceViscosity += settings.viscosity * settings.mass * (particlesVelocity[j].Value - velocity) / density * (45.0f / (PI * math.pow(settings.smoothingRadius, 6.0f))) * (settings.smoothingRadius - r);
                    }

                    // Next neighbor
                    found = hashMap.TryGetNextValue(out j, ref iterator);
                }
            }

            // Gravity
            float3 forceGravity = new float3(0.0f, -9.81f, 0.0f) * density * settings.gravityMult;

            // Apply
            particlesForces[index] = forcePressure + forceViscosity*0.005f + forceGravity*5f;
        }
    }


    [BurstCompile]
    private struct ComputeNewPoint : IJobParallelFor
    {
        
        [ReadOnly] public NativeArray<Position> particlesPosition;
        
        [ReadOnly] public NativeArray<SMBWaypoint> waypoints;
        public NativeArray<SMBDestination> particlesDestination;
        public NativeArray<SMBPath> indexPaths;

        private const float PI = 3.14159274F;
        private static Vector3 getProyectedWayPoint(Vector3 position, Vector3 start, Vector3 end)
        {
            float x = position.x;
            float y = position.z;
            float x1 = start.x;
            float y1 = start.z;
            float x2 = end.x;
            float y2 = end.z;

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

        public void Execute(int index)
        {
            int FirsTriangle = indexPaths[index].Firsttriangle;
            Vector3 NextPoint = indexPaths[index].NextPoint;
            float3 fordwarDir = particlesPosition[index].Value;
            int indexPath = indexPaths[index].indexIni;
            int Length = indexPaths[index].indexFin;
            int recalculate = indexPaths[index].recalculate;
            if ((FirsTriangle == -1 || FirsTriangle == 1 || Vector3.Distance(NextPoint, particlesPosition[index].Value) < 4))
            {
                if (FirsTriangle == -1)
                {
                    NextPoint = particlesPosition[index].Value;
                }
                else {
                    if (indexPath + 1 < Length)
                    {
                        if (FirsTriangle != 1)  NextPoint = getProyectedWayPoint(particlesPosition[index].Value, waypoints[indexPath].start, waypoints[indexPath].end);
                        else NextPoint = getProyectedWayPoint(particlesPosition[index].Value, waypoints[indexPath].start, waypoints[indexPath].end);
                        fordwarDir = (new float3(NextPoint)) - fordwarDir;
                        FirsTriangle = 0;
                        indexPath = indexPath + 1;
                    }
                    else 
                    {
                        /*if (particlesDestination[index].finished < 2)*/ NextPoint = particlesDestination[index].destination;
                        //else NextPoint = particlesDestination[index].destinations2;
                        fordwarDir = (new float3(NextPoint)) - fordwarDir;
                        /*if (Vector3.Distance(NextPoint, particlesPosition[index].Value) < 4 && particlesDestination[index].finished != 2)
                        {
                            float3 dest2 = particlesDestination[index].destinations2;
                            int finished = 1;
                            particlesDestination[index] = new SMBDestination { destination = NextPoint, destinations2 = dest2, finished = finished };
                        }*/
                    }
                }

            }
            indexPaths[index] = new SMBPath { indexIni = indexPath, indexFin = Length, Firsttriangle = FirsTriangle, NextPoint = NextPoint , fordwarDir = fordwarDir, recalculate = recalculate};
        }
    }

    [BurstCompile]
    private struct RecomputeNewPoint : IJobParallelFor
    {
        public NativeArray<SMBPath> indexPaths;
        [ReadOnly] public NativeArray<Position> particlesPosition;
        [ReadOnly] public NativeArray<SMBWaypoint> waypoints;

        private static Vector3 getProyectedWayPoint(Vector3 position, Vector3 start, Vector3 end)
        {
            float x = position.x;
            float y = position.z;
            float x1 = start.x;
            float y1 = start.z;
            float x2 = end.x;
            float y2 = end.z;

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

        public void Execute(int index)
        {
            int FirsTriangle = indexPaths[index].Firsttriangle;
            Vector3 NextPoint = indexPaths[index].NextPoint;
            float3 fordwarDir = indexPaths[index].fordwarDir;
            int indexPath = indexPaths[index].indexIni;
            int Length = indexPaths[index].indexFin;
            int recalculate = indexPaths[index].recalculate;
            if (recalculate == 0 && FirsTriangle != -1)
            {
                if (indexPath + 1 < Length)
                {
                    Unity.Mathematics.Random rand = new Unity.Mathematics.Random(1);
                    recalculate = rand.NextInt(10)+5;
                    NextPoint = getProyectedWayPoint(particlesPosition[index].Value, waypoints[indexPath-1].start, waypoints[indexPath-1].end);
                    FirsTriangle = 0;
                }

            }
            else if (recalculate>-1) recalculate = recalculate - 1;
            indexPaths[index] = new SMBPath { indexIni = indexPath, indexFin = Length, Firsttriangle = FirsTriangle, NextPoint = NextPoint, recalculate = recalculate, fordwarDir = fordwarDir};
        }
    }

    [BurstCompile]
    private struct ComputePosition : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Position> particlesPosition;
        [ReadOnly] public NativeMultiHashMap<int, int> hashMap;
        [ReadOnly] public NativeArray<int> cellOffsetTable;
        [ReadOnly] public SMBProperties settings;
        public NativeArray<SMBSspeed> particlesSspeed;
        [ReadOnly] public NativeArray<SMBVelocity> particlesVelocity;
        [ReadOnly] public NativeArray<SMBDestination> particlesDestination;
        private const float PI = 3.14159274F;
        [ReadOnly] public NativeArray<SMBPath> indexPaths;
        public NativeArray<Position> finalPosition;


        public void Execute(int index)
        {
            float sSpeed = particlesSspeed[index].Sspeed;
            float step = sSpeed * 0.03f, r;
            bool found;
            Vector3 aux = particlesVelocity[index].Value, center, normal, tangent1, tangent2;
            float Speed = aux.magnitude;
            float3 position = particlesPosition[index].Value;
            float3 velocity = particlesVelocity[index].Value;
            float Acceleration = 5;
            float MaxSpeed = 10;
            float3 NextPoint = indexPaths[index].NextPoint;
            float3 forward = Vector3.Normalize(indexPaths[index].fordwarDir);
            
            int i, hash, j; float angle;
            int3 gridOffset;
            int3 gridPosition = GridHash.Quantize(position, settings.radius);
            float3 closestParticle = new float3();
            float3 otherForward = new float3();
            float minDist = Mathf.Infinity;
            // Find neighbors
             for (int oi = 0; oi < 5; oi++)
            {
                i = oi * 3;
                float scale = 3;
               
                if (oi == 3) scale = 2;
                else if (oi == 4) scale = 0.5f;
                gridOffset = new int3((int)(forward.x * scale), (int)(forward.y * scale), (int)(forward.z * scale));
                hash = GridHash.Hash(gridPosition + gridOffset);
                if (oi == 1)
                {
                    Vector3 left = Vector3.Normalize(Vector3.Cross(forward, Vector3.up));
                    gridPosition = GridHash.Quantize(position + new float3(left), settings.radius);
                    
                    hash = GridHash.Hash(gridPosition + gridOffset);
                }
                else if (oi == 2)
                {
                    Vector3 right = Vector3.Normalize(Vector3.Cross(forward, Vector3.down));
                    gridPosition = GridHash.Quantize(position + new float3(right), settings.radius);
                    
                    hash = GridHash.Hash(gridPosition + gridOffset);
                }
                NativeMultiHashMapIterator<int> iterator;
                found = hashMap.TryGetFirstValue(hash, out j, out iterator);
                while (found)
                {
                    if (index == j)
                    {
                        found = hashMap.TryGetNextValue(out j, ref iterator);
                        continue;
                    }

                    // Neighbor found
                    float3 rij = particlesPosition[j].Value - position;
                    float r2 = math.lengthsq(rij);

                    if (r2 < minDist)
                    {
                        otherForward = indexPaths[j].fordwarDir;
                        float3 auxDir = particlesPosition[j].Value - position;
                        angle = Mathf.Abs(Vector3.Angle(forward, Vector3.Normalize(auxDir)));
                        minDist = r2;
                        closestParticle = particlesPosition[j].Value;
                    }

                    // Next neighbor
                    found = hashMap.TryGetNextValue(out j, ref iterator);
                }
               
            }
            
            Vector3 oldPosition = particlesPosition[index].Value;

            
            if (minDist < Mathf.Infinity)
            {
                r = settings.radius;
                float dot = Vector3.Dot(forward, otherForward);
                center = closestParticle;
                if (dot > 0.8)
                {
                    /*if (j < 3 && ratio > 0.4f)
                    {
                        if (Speed - 3 * Time.deltaTime > 3) Speed = Speed - 3 * Time.deltaTime;
                        else if (Speed <= 3) Speed = Speed + 3 * Time.deltaTime; ;
                    }
                    else if (j >= 3)
                    {
                        if (Speed - 3 * Time.deltaTime > 2) Speed = Speed - 3 * Time.deltaTime;
                        else if (Speed <= 2) Speed = Speed + 3 * Time.deltaTime;
                    }*/
                    if (sSpeed - 0.5f > 4) sSpeed = sSpeed - 0.5f;
                    else if (Speed <= 4) sSpeed = sSpeed + 0.5f;
                    step = sSpeed * 0.03f;
                    
                    position = Vector3.MoveTowards(oldPosition, NextPoint, step);
                }
                else if (dot < -0.25f)
                {
                    if (Speed < 7) sSpeed = sSpeed + 0.5f;
                    step = sSpeed * 0.03f;
                    normal = Vector3.Normalize(oldPosition - center);
                    normal.y = normal.y - normal.y;
                    tangent1 = Vector3.Cross(normal, Vector3.down);
                    tangent2 = Vector3.Cross(normal, Vector3.up);
                    Vector3 auxNextPoint = NextPoint;
                    Vector3 Fordwarddirection = auxNextPoint - oldPosition;
                    float aux1 = Vector3.Angle(tangent1, Fordwarddirection);
                    float aux2 = Vector3.Angle(tangent2, Fordwarddirection);
                    float directionW = 0.75f;
                    Vector3 finalDirection = Vector3.Normalize(Fordwarddirection) * (1f - directionW) * 10f + oldPosition;
                    
                    Unity.Mathematics.Random rand = new Unity.Mathematics.Random(1);
                    Vector3 maxtang;
                    if (aux2 < aux1) maxtang = tangent1;
                    else maxtang = tangent2;
                    if (aux2 < aux1) tangent1 = tangent2;
                    if (rand.NextInt(4) == 1) tangent1 = maxtang;
                    position = Vector3.MoveTowards(oldPosition, finalDirection + Vector3.Normalize(tangent1) * directionW * 10f, step);
                }
                else {
                    if (Speed < 7) sSpeed = sSpeed + 0.5f;
                    step = sSpeed * 0.03f;
                    position = Vector3.MoveTowards(oldPosition, NextPoint, step);
                }
            }
            else
            {
                if (Speed < 7) sSpeed = sSpeed + 0.5f;
                step = sSpeed * 0.03f;
                position = Vector3.MoveTowards(oldPosition, NextPoint, step);
            }
          
            particlesSspeed[index] = new SMBSspeed { Sspeed = sSpeed };
            finalPosition[index] = new Position { Value = position };
        }
    }



    [BurstCompile]
    private struct Integrate : IJobParallelFor
    {
        [ReadOnly] public NativeArray<float3> particlesForces;
        [ReadOnly] public NativeArray<float> particlesDensity;

        public NativeArray<Position> particlesPosition;
        public NativeArray<SMBVelocity> particlesVelocity;

        private const float DT = 0.0008f;



        public void Execute(int index)
        {
            // Cache
            float3 velocity = particlesVelocity[index].Value;
            float3 position = particlesPosition[index].Value;

            // Process
            velocity += DT * particlesForces[index] / particlesDensity[index];
            position += DT * velocity;

            // Apply
            particlesVelocity[index] = new SMBVelocity { Value = velocity };
            particlesPosition[index] = new Position { Value = position };
        }
    }



    [BurstCompile]
    private struct ComputeColliders : IJobParallelFor
    {
        [ReadOnly] public SMBProperties settings;
        [ReadOnly] public NativeArray<SMBCollider> copyColliders;

        public NativeArray<Position> particlesPosition;
        public NativeArray<SMBVelocity> particlesVelocity;

        private const float BOUND_DAMPING = -0.5f;



        private static bool Intersect(SMBCollider collider, float3 position, float radius, out float3 penetrationNormal, out float3 penetrationPosition, out float penetrationLength)
        {
            float3 colliderProjection = collider.position - position;

            penetrationNormal = math.cross(collider.right, collider.up);
            penetrationLength = math.abs(math.dot(colliderProjection, penetrationNormal)) - (radius / 2.0f);
            penetrationPosition = collider.position - colliderProjection;

            return penetrationLength < 0.0f
                && math.abs(math.dot(colliderProjection, collider.right)) < collider.scale.x
                && math.abs(math.dot(colliderProjection, collider.up)) < collider.scale.y;
        }



        private static Vector3 DampVelocity(SMBCollider collider, float3 velocity, float3 penetrationNormal, float drag)
        {
            float3 newVelocity = math.dot(velocity, penetrationNormal) * penetrationNormal * BOUND_DAMPING
                                + math.dot(velocity, collider.right) * collider.right * drag
                                + math.dot(velocity, collider.up) * collider.up * drag;
            newVelocity = math.dot(newVelocity, new float3(0, 0, 1)) * new float3(0, 0, 1)
                        + math.dot(newVelocity, new float3(1, 0, 0)) * new float3(1, 0, 0)
                        + math.dot(newVelocity, new float3(0, 1, 0)) * new float3(0, 1, 0);
            return newVelocity;
        }



        public void Execute(int index)
        {
            // Cache
            int colliderCount = copyColliders.Length;
            float3 position = particlesPosition[index].Value;
            float3 velocity = particlesVelocity[index].Value;

            // Process
            for (int i = 0; i < colliderCount; i++)
            {
                float3 penetrationNormal;
                float3 penetrationPosition;
                float penetrationLength;
                if (Intersect(copyColliders[i], position, settings.radius, out penetrationNormal, out penetrationPosition, out penetrationLength))
                {
                    velocity = DampVelocity(copyColliders[i], velocity, penetrationNormal, 1.0f - settings.drag);
                    position = penetrationPosition - penetrationNormal * math.abs(penetrationLength);
                }
            }

            // Apply
            particlesVelocity[index] = new SMBVelocity { Value = velocity };
            particlesPosition[index] = new Position { Value = position };
        }
    }



    [BurstCompile]
    private struct ApplyPositions : IJobParallelFor
    {
        [ReadOnly] public NativeArray<Position> particlesPosition;
        [ReadOnly] public NativeArray<SMBVelocity> particlesVelocity;
        [ReadOnly] public NativeArray<SMBPath> particlesindexPaths;
        [ReadOnly] public NativeArray<SMBSspeed> particlesSspeed;
        //[ReadOnly] public NativeArray<SMBDestination> particlesDestination;

        public ComponentDataArray<Position> positions;
        public ComponentDataArray<SMBVelocity> velocities;
        public ComponentDataArray<SMBPath> indexPaths;
        public ComponentDataArray<SMBSspeed> SMBSspeeds;
        //public ComponentDataArray<SMBDestination> SMBdestinations;

        public void Execute(int index)
        {
            // Apply to components
            positions[index] = new Position { Value = particlesPosition[index].Value };
            velocities[index] = particlesVelocity[index];
            indexPaths[index] = particlesindexPaths[index];
            SMBSspeeds[index] = particlesSspeed[index];
            //SMBdestinations[index] = particlesDestination[index];
        }
    }



    protected override void OnCreateManager()
    {
        // Import
        SMBCharacterGroup = GetComponentGroup(ComponentType.ReadOnly(typeof(SMBProperties)), typeof(Position), typeof(SMBVelocity), typeof(SMBDestination), typeof(SMBSspeed), typeof(SMBPath));
        SMBColliderGroup = GetComponentGroup(ComponentType.ReadOnly(typeof(SMBCollider)));
        Graph = astar.Graph;
    }


    //La data del sphereParticle es compartiiiida
    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        if (cameraTransform == null)
            cameraTransform = GameObject.Find("Main Camera").transform;
        EntityManager.GetAllUniqueSharedComponentData(uniqueTypes);
        ComponentDataArray<SMBCollider> colliders = SMBColliderGroup.GetComponentDataArray<SMBCollider>();
        int colliderCount = colliders.Length;

        for (int typeIndex = 1; typeIndex < uniqueTypes.Count; typeIndex++)
        {
            // Get the current chunk setting
            SMBProperties settings = uniqueTypes[typeIndex];
            //SMBDestination smbdestination = _destination[typeIndex];
            SMBCharacterGroup.SetFilter(settings);

            // Cache the data
            ComponentDataArray<Position> positions = SMBCharacterGroup.GetComponentDataArray<Position>();
            ComponentDataArray<SMBVelocity> velocities = SMBCharacterGroup.GetComponentDataArray<SMBVelocity>();
            ComponentDataArray<SMBDestination> SMBdestinations = SMBCharacterGroup.GetComponentDataArray<SMBDestination>();
            ComponentDataArray<SMBSspeed> SMBSspeeds = SMBCharacterGroup.GetComponentDataArray<SMBSspeed>();
            ComponentDataArray<SMBPath> indexPaths = SMBCharacterGroup.GetComponentDataArray<SMBPath>();

            int cacheIndex = typeIndex - 1;
            int particleCount = positions.Length;

            NativeMultiHashMap<int, int> hashMap = new NativeMultiHashMap<int, int>(particleCount, Allocator.TempJob);
            NativeArray<Position> particlesPosition = new NativeArray<Position>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            NativeArray<Position> finalposition = new NativeArray<Position>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            NativeArray<SMBVelocity> particlesVelocity = new NativeArray<SMBVelocity>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            NativeArray<SMBDestination> particlesDestination = new NativeArray<SMBDestination>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            NativeArray<SMBSspeed> particlesSspeed = new NativeArray<SMBSspeed>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            NativeArray<SMBPath> particlesindexPaths = new NativeArray<SMBPath>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            NativeArray<float3> particlesForces = new NativeArray<float3>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            NativeArray<float> particlesPressure = new NativeArray<float>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            NativeArray<float> particlesDensity = new NativeArray<float>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);
            NativeArray<int> particleIndices = new NativeArray<int>(particleCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);

            NativeArray<int> cellOffsetTableNative = new NativeArray<int>(cellOffsetTable, Allocator.TempJob);
            NativeArray<SMBCollider> copyColliders = new NativeArray<SMBCollider>(colliderCount, Allocator.TempJob, NativeArrayOptions.UninitializedMemory);



            // Add new or dispose previous particle chunks
            PreviousParticle nextParticles = new PreviousParticle
            {
                hashMap = hashMap,
                particlesPosition = particlesPosition,
                particlesVelocity = particlesVelocity,
                particlesDestination = particlesDestination,
                particlesSspeed = particlesSspeed,
                particlesindexPaths = particlesindexPaths,
                particlesForces = particlesForces,
                particlesPressure = particlesPressure,
                particlesDensity = particlesDensity,
                particleIndices = particleIndices,
                cellOffsetTable = cellOffsetTableNative,
                copyColliders = copyColliders
            };

            if (cacheIndex > previousParticles.Count - 1)
            {
                previousParticles.Add(nextParticles);
            }
            else
            {
                previousParticles[cacheIndex].hashMap.Dispose();
                previousParticles[cacheIndex].particlesPosition.Dispose();
                previousParticles[cacheIndex].particlesVelocity.Dispose();
                previousParticles[cacheIndex].particlesDestination.Dispose();
                previousParticles[cacheIndex].particlesSspeed.Dispose();
                previousParticles[cacheIndex].particlesindexPaths.Dispose();
                previousParticles[cacheIndex].particlesForces.Dispose();
                previousParticles[cacheIndex].particlesPressure.Dispose();
                previousParticles[cacheIndex].particlesDensity.Dispose();
                previousParticles[cacheIndex].particleIndices.Dispose();
                previousParticles[cacheIndex].cellOffsetTable.Dispose();
                previousParticles[cacheIndex].copyColliders.Dispose();
            }
            previousParticles[cacheIndex] = nextParticles;



            // Copy the component data to native arrays
            CopyComponentData<Position> particlesPositionJob = new CopyComponentData<Position> { Source = positions, Results = particlesPosition };
            JobHandle particlesPositionJobHandle = particlesPositionJob.Schedule(particleCount, 64, inputDeps);

            CopyComponentData<SMBVelocity> particlesVelocityJob = new CopyComponentData<SMBVelocity> { Source = velocities, Results = particlesVelocity };
            JobHandle particlesVelocityJobHandle = particlesVelocityJob.Schedule(particleCount, 64, inputDeps);

            CopyComponentData<SMBDestination> particlesDestinationJob = new CopyComponentData<SMBDestination> { Source = SMBdestinations, Results = particlesDestination };
            JobHandle particlesDestinationJobHandle = particlesDestinationJob.Schedule(particleCount, 64, inputDeps);

            CopyComponentData<SMBSspeed> particlesSspeedJob = new CopyComponentData<SMBSspeed> { Source = SMBSspeeds, Results = particlesSspeed };
            JobHandle particlesSspeedJobHandle = particlesSspeedJob.Schedule(particleCount, 64, inputDeps);

            CopyComponentData<SMBCollider> copyCollidersJob = new CopyComponentData<SMBCollider> { Source = colliders, Results = copyColliders };
            JobHandle copyCollidersJobHandle = copyCollidersJob.Schedule(colliderCount, 64, inputDeps);

            MemsetNativeArray<float> particlesPressureJob = new MemsetNativeArray<float> { Source = particlesPressure, Value = 0.0f };
            JobHandle particlesPressureJobHandle = particlesPressureJob.Schedule(particleCount, 64, inputDeps);

            MemsetNativeArray<float> particlesDensityJob = new MemsetNativeArray<float> { Source = particlesDensity, Value = 0.0f };
            JobHandle particlesDensityJobHandle = particlesDensityJob.Schedule(particleCount, 64, inputDeps);

            MemsetNativeArray<int> particleIndicesJob = new MemsetNativeArray<int> { Source = particleIndices, Value = 0 };
            JobHandle particleIndicesJobHandle = particleIndicesJob.Schedule(particleCount, 64, inputDeps);

            MemsetNativeArray<float3> particlesForcesJob = new MemsetNativeArray<float3> { Source = particlesForces, Value = new float3(0, 0, 0) };
            JobHandle particlesForcesJobHandle = particlesForcesJob.Schedule(particleCount, 64, inputDeps);
            
            MemsetNativeArray<Position> finalpositionJob = new MemsetNativeArray<Position> { Source = finalposition, Value = new Position { Value = new float3() } };
            JobHandle finalpositionJobHandle = finalpositionJob.Schedule(particleCount, 64, inputDeps);

            //JobHandle computepathsJobHandle = particlesPositionJobHandle;


            if (first)
            {
                int index = 0, firsTriangle = 1;
                for (int i = 0; i < particleCount; ++i)
                {
                    astar.cleanStructures();
                    astar.setOrigin(positions[i].Value);
                    astar.setDestination(SMBdestinations[i].destination);
                    astar.trianglePath2();
                    //Allpaths.AddRange(astar.trianglePath2());
                    wayPointsPath.AddRange(astar.getWayPoints());

                    int aux = wayPointsPath.Count;
                    if (aux - index == 0)
                    {
                        firsTriangle = -1;
                    }
                    else firsTriangle = 1;
                    Unity.Mathematics.Random ola = new Unity.Mathematics.Random(1);
                    indexPaths[i] = new SMBPath { indexIni = index, indexFin = aux, NextPoint = new float3(), Firsttriangle = firsTriangle, recalculate = ola.NextInt(15), fordwarDir = new float3() };
                    index = aux;
                }
                first = false;
            }
            //Una vez llegado al destino ir a otro, funciona pero va muuuuy lento
            /*else
            {
                int index = 0, firsTriangle = 1, aux = 0;
                int diff = 0;
                for (int i = 0; i < particleCount; ++i)
                {
                    index = indexPaths[i].indexIni + diff;
                    aux = indexPaths[i].indexFin + diff;
                    float3 NextPoint = indexPaths[i].NextPoint, fordwarDir = indexPaths[i].fordwarDir;
                    int recalculate = indexPaths[i].recalculate;
                    firsTriangle = indexPaths[i].Firsttriangle;
                    if (SMBdestinations[i].finished == 1)
                    {
                        firsTriangle = 1;
                        astar.cleanStructures();
                        astar.setOrigin(positions[i].Value);
                        astar.setDestination(SMBdestinations[i].destinations2);
                        astar.trianglePath2();
                        int count = 0;
                        if (i == 0)
                        {
                            wayPointsPath.RemoveRange(0, indexPaths[i].indexFin);
                            index = 0;
                            count = indexPaths[i].indexFin;
                        }
                        else
                        {
                            index = indexPaths[i - 1].indexFin;
                            count = indexPaths[i].indexFin + diff - indexPaths[i - 1].indexFin;
                            wayPointsPath.RemoveRange(indexPaths[i - 1].indexFin, count);
                        }
                        List<SMBWaypoint> wayaux = astar.getWayPoints();
                        wayPointsPath.InsertRange(index, wayaux);
                        
                        aux = wayaux.Count;
                        
                        indexPaths[i] = new SMBPath { indexIni = index, indexFin = aux + index, NextPoint = new float3(), Firsttriangle = firsTriangle, recalculate = recalculate, fordwarDir = new float3() };
                        SMBdestinations[i] = new SMBDestination {finished = 2, destinations2 = SMBdestinations[i].destinations2, destination = SMBdestinations[i].destination };
                        diff += aux - count;
                    }
                    else indexPaths[i] = new SMBPath { indexIni = index, indexFin = aux, NextPoint = NextPoint, Firsttriangle = firsTriangle, recalculate = recalculate, fordwarDir = fordwarDir };
                }
            }*/

         
            NativeArray<SMBWaypoint> NwayPointspaths = new NativeArray<SMBWaypoint>(wayPointsPath.Count, Allocator.TempJob);
            //MemsetNativeArray<SMBWaypoint> waypointsJob = new MemsetNativeArray<SMBWaypoint> { Source = NwayPointspaths, Value = new SMBWaypoint { } };
            
            //NativeArray<int>.Copy(Allpaths.ToArray(), paths);
            NativeArray<SMBWaypoint>.Copy(wayPointsPath.ToArray(), NwayPointspaths, wayPointsPath.Count);
            //CopyComponentData<SMBDestination> particlesDestinationJob = new CopyComponentData<SMBDestination> { Source = SMBdestinations, Results = particlesDestination };
            //JobHandle particlesDestinationJobHandle = particlesDestinationJob.Schedule(particleCount, 64, inputDeps);

            CopyComponentData<SMBPath> particlesIndexPathJob = new CopyComponentData<SMBPath> { Source = indexPaths, Results = particlesindexPaths };
            JobHandle particlesIndexPathJobHandle = particlesIndexPathJob.Schedule(particleCount, 64, inputDeps);
            // Put positions into a hashMap
            HashPositions hashPositionsJob = new HashPositions
            {
                positions = particlesPosition,
                hashMap = hashMap.ToConcurrent(),
                cellRadius = settings.radius
            };
            JobHandle hashPositionsJobHandle = hashPositionsJob.Schedule(particleCount, 64, particlesPositionJobHandle);

            JobHandle mergedPositionIndicesJobHandle = JobHandle.CombineDependencies(hashPositionsJobHandle, particleIndicesJobHandle);

            MergeParticles mergeParticlesJob = new MergeParticles
            {
                particleIndices = particleIndices
            };
            JobHandle mergeParticlesJobHandle = mergeParticlesJob.Schedule(hashMap, 64, mergedPositionIndicesJobHandle);

            JobHandle mergedMergedParticlesDensityPressure = JobHandle.CombineDependencies(mergeParticlesJobHandle, particlesPressureJobHandle, particlesDensityJobHandle);

            // Compute density pressure
            ComputeDensityPressure computeDensityPressureJob = new ComputeDensityPressure
            {
                particlesPosition = particlesPosition,
                densities = particlesDensity,
                pressures = particlesPressure,
                hashMap = hashMap,
                cellOffsetTable = cellOffsetTableNative,
                settings = settings
            };
            JobHandle computeDensityPressureJobHandle = computeDensityPressureJob.Schedule(particleCount, 64, mergedMergedParticlesDensityPressure);

            JobHandle mergeComputeDensityPressureVelocityForces = JobHandle.CombineDependencies(computeDensityPressureJobHandle, particlesForcesJobHandle, particlesVelocityJobHandle);

            // Compute forces
            ComputeForces computeForcesJob = new ComputeForces
            {
                particlesPosition = particlesPosition,
                particlesVelocity = particlesVelocity,
                particlesForces = particlesForces,
                particlesPressure = particlesPressure,
                particlesDensity = particlesDensity,
                cellOffsetTable = cellOffsetTableNative,
                hashMap = hashMap,
                settings = settings
            };
            JobHandle computeForcesJobHandle = computeForcesJob.Schedule(particleCount, 64, mergeComputeDensityPressureVelocityForces);

            // Integrate
            Integrate integrateJob = new Integrate
            {
                particlesPosition = particlesPosition,
                particlesVelocity = particlesVelocity,
                particlesDensity = particlesDensity,
                particlesForces = particlesForces
            };
            JobHandle integrateJobHandle = integrateJob.Schedule(particleCount, 64, computeForcesJobHandle);
            
            JobHandle mergedIntegrateCollider = JobHandle.CombineDependencies(integrateJobHandle, copyCollidersJobHandle);
            //JobHandle mergedIntegrateCollider = JobHandle.CombineDependencies(particlesPositionJobHandle, particlesVelocityJobHandle, copyCollidersJobHandle);
            // Compute Colliders
            ComputeColliders computeCollidersJob = new ComputeColliders
            {
                particlesPosition = particlesPosition,
                particlesVelocity = particlesVelocity,
                copyColliders = copyColliders,
                settings = settings
            };
            JobHandle computeCollidersJobHandle = computeCollidersJob.Schedule(particleCount, 64, mergedIntegrateCollider);
            JobHandle allReady = JobHandle.CombineDependencies(computeCollidersJobHandle, particlesIndexPathJobHandle, particlesDestinationJobHandle);

            ComputeNewPoint computeNewPointJob = new ComputeNewPoint
            {
                particlesPosition = particlesPosition,
                waypoints = NwayPointspaths,
                indexPaths = particlesindexPaths,
                particlesDestination = particlesDestination

            };
            JobHandle computeNewPointJobHandle = computeNewPointJob.Schedule(particleCount, 64, allReady);


            computeNewPointJobHandle = JobHandle.CombineDependencies(computeNewPointJobHandle, finalpositionJobHandle);

            RecomputeNewPoint RecomputeNewPointJob = new RecomputeNewPoint
            {
                particlesPosition = particlesPosition,
                waypoints = NwayPointspaths,
                indexPaths = particlesindexPaths

            };
            JobHandle RecomputeNewPointJobHandle = RecomputeNewPointJob.Schedule(particleCount, 64, computeNewPointJobHandle);
            JobHandle preparedToComputePositions = JobHandle.CombineDependencies(RecomputeNewPointJobHandle, particlesSspeedJobHandle);
            ComputePosition computePositionJob = new ComputePosition
            {
                particlesPosition = particlesPosition,
                particlesDestination = particlesDestination,
                particlesSspeed = particlesSspeed,
                particlesVelocity = particlesVelocity,
                indexPaths = particlesindexPaths,
                hashMap = hashMap,
                settings = settings,
                cellOffsetTable = cellOffsetTableNative,
                finalPosition = finalposition
            };
            JobHandle comptePositionJobHandle = computePositionJob.Schedule(particleCount, 64, preparedToComputePositions);

         
            // Apply positions
            ApplyPositions applyPositionsJob = new ApplyPositions
            {
                particlesPosition = finalposition,
                particlesVelocity = particlesVelocity,
                particlesindexPaths = particlesindexPaths,
                //particlesDestination = particlesDestination,
                particlesSspeed = particlesSspeed,
                positions = positions,
                velocities = velocities,
                indexPaths = indexPaths,
                SMBSspeeds = SMBSspeeds,
                //SMBdestinations = SMBdestinations,
            };
            JobHandle applyPositionsJobHandle = applyPositionsJob.Schedule(particleCount, 64, comptePositionJobHandle);

            inputDeps = applyPositionsJobHandle;
            inputDeps.Complete();
            NwayPointspaths.Dispose();
            finalposition.Dispose();
        }

        // Done
        uniqueTypes.Clear();

        return inputDeps;
    }



    protected override void OnStopRunning()
    {
        for (int i = 0; i < previousParticles.Count; i++)
        {
            previousParticles[i].hashMap.Dispose();
            previousParticles[i].particlesPosition.Dispose();
            previousParticles[i].particlesVelocity.Dispose();
            previousParticles[i].particlesDestination.Dispose();
            previousParticles[i].particlesSspeed.Dispose();
            previousParticles[i].particlesindexPaths.Dispose();
            previousParticles[i].particlesForces.Dispose();
            previousParticles[i].particlesPressure.Dispose();
            previousParticles[i].particlesDensity.Dispose();
            previousParticles[i].particleIndices.Dispose();
            previousParticles[i].cellOffsetTable.Dispose();
            previousParticles[i].copyColliders.Dispose();
        }

        previousParticles.Clear();
    }
}
