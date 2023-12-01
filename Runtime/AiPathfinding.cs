using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using Clipper2Lib;
using EarClipperLib;
using FunnelAlgorithm;
using UnityEngine.AI;

namespace com.heax3.pathfinding_unity
{
    public class AiPathfinding
    {
        List<MapTriangle> _triangles;
        public AiPathfinding(List<GameObject> mapGameObjects, List<GameObject> obstacleGameObjects = null)
        {
            PathsD solution = CreateMapFromColliders(mapGameObjects);

            List<List<Vector3m>> holes = null;
            if (obstacleGameObjects != null && obstacleGameObjects.Count > 0)
            {
                holes = GetObstaclePointsForTriangulateFromMap(obstacleGameObjects);
            }

            List<Vector3m> areaForTriangulate = GetPointsForTriangulateFromMap(solution);

            _triangles = TriangulateMap(areaForTriangulate, holes);
        }

        public AiPathfinding(NavMeshTriangulation navMeshTriangulation)
        {
            int k = 0;

            List<MapTriangle> mapTriangles = new List<MapTriangle>();
            MapTriangle mapTriangle = null;
            LineRenderer anglelineRenderer = null;

            for (var i = 0; i < navMeshTriangulation.indices.Length; i += 3)
            {
                var t0 = navMeshTriangulation.indices[i];
                var t1 = navMeshTriangulation.indices[i + 1];
                var t2 = navMeshTriangulation.indices[i + 2];


/*                anglelineRenderer = VisualUtil.GeneratePathLine(Color.green);
                anglelineRenderer.positionCount = 3;*/
                mapTriangle = new MapTriangle();

                Vector3 v0 = navMeshTriangulation.vertices[t0];
                Vector3 v1 = navMeshTriangulation.vertices[t1];
                Vector3 v2 = navMeshTriangulation.vertices[t2];

/*                anglelineRenderer.SetPosition(0, v0);
                anglelineRenderer.SetPosition(1, v1);
                anglelineRenderer.SetPosition(2, v2);*/

                mapTriangle.AddVertices(new Vector3((float)Math.Round(v0.x, 3), (float)Math.Round(v0.y, 3), (float)Math.Round(v0.z, 3)));
                mapTriangle.AddVertices(new Vector3((float)Math.Round(v1.x, 3), (float)Math.Round(v1.y, 3), (float)Math.Round(v1.z, 3)));
                mapTriangle.AddVertices(new Vector3((float)Math.Round(v2.x, 3), (float)Math.Round(v2.y, 3), (float)Math.Round(v2.z, 3)));

                /*                mapTriangle.AddVertices(new Vector3(v0.x, 1f, v0.z));
                                mapTriangle.AddVertices(new Vector3(v1.x, 1f, v1.z));
                                mapTriangle.AddVertices(new Vector3(v2.x, 1f, v2.z));*/

/*                GameObject go2 = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                go2.transform.position = new Vector3(mapTriangle.Center.x, 8, mapTriangle.Center.z);
                go2.transform.localScale = go2.transform.localScale * 0.4f;
*/
                mapTriangles.Add(mapTriangle);
            }

            foreach (var currentMapTriangle in mapTriangles)
            {
                currentMapTriangle.LinkedMapTriangles.Clear();

                foreach (var edge in currentMapTriangle.Edges)
                {
                    MapTriangle foundMapTriangle = mapTriangles.FirstOrDefault

                         (t => (t != currentMapTriangle && t.AB.Equals(edge))
                         || (t != currentMapTriangle && t.BC.Equals(edge))
                         || (t != currentMapTriangle && t.CA.Equals(edge)));

                    if (foundMapTriangle != null)
                    {
                        if (!currentMapTriangle.LinkedMapTriangles.Contains(foundMapTriangle))
                        {
                            currentMapTriangle.LinkedMapTriangles.Add(foundMapTriangle);
                        }

                    }
                }
            }

            _triangles = mapTriangles;
        }

        public Vector3[] CreatePath(Vector3 startPathPoint, Vector3 targetPathPoint)
        {

            Vector2 startPathPoint2D = new Vector2(startPathPoint.x, startPathPoint.z);
            Vector2 targetPathPoint2D = new Vector2(targetPathPoint.x, targetPathPoint.z);

            startPathPoint = new Vector3(startPathPoint.x, startPathPoint.y, startPathPoint.z);
            targetPathPoint = new Vector3(targetPathPoint.x, targetPathPoint.y, targetPathPoint.z);

            MapTriangle startTriangle = GetTriangleByPoint(_triangles, startPathPoint);

            Debug.Log("START Triangle " + startTriangle.ToString());

            MapTriangle targetTriangle = GetTriangleByPoint(_triangles, targetPathPoint);

            Debug.Log("TARGET Triangle " + targetTriangle.ToString());

            if (startTriangle == targetTriangle)
            {
/*                       LineRenderer anglelineRenderer = VisualUtil.GeneratePathLine(Color.black);
                       anglelineRenderer.positionCount = 2;
                       anglelineRenderer.SetPosition(0, new Vector3(startPathPoint.x, startPathPoint.y + 0.9f, startPathPoint.z));
                       anglelineRenderer.SetPosition(1, new Vector3(targetPathPoint.x, targetPathPoint.y + 0.9f, targetPathPoint.z));
*/
                return new Vector3[2] { startPathPoint, targetPathPoint };
            }

            AStarVector2Float targetClosestTriangleVertices = GetClosestVerticesToPoint(targetTriangle, startPathPoint2D, targetPathPoint2D);

            Debug.Log("Target Triangle Vertice" + targetClosestTriangleVertices);

            AStarVector2Float startClosestTriangleVertices = GetClosestVerticesToPoint(startTriangle, targetPathPoint2D, startPathPoint2D);

            Debug.Log("Start Triangle Vertice" + startClosestTriangleVertices);

            var graphNodesWithNeighboards = NeighboardNodeUtility.GetNeighboardNode(startClosestTriangleVertices, _triangles);

            IReadOnlyCollection<AStarVector2Float> aStarPath = AStarPathResult(startClosestTriangleVertices, targetClosestTriangleVertices, graphNodesWithNeighboards);

            List<MapTriangle> pathTriangles = NodeMapTriangleUtility.GenerateNodeMapTriangle(
                _triangles,
                aStarPath,
                targetTriangle,
                startTriangle,
                targetPathPoint,
                startPathPoint);

            FunnelAlgorithmPath funnelAlgorithmPath = GetClearPath(pathTriangles);


            return funnelAlgorithmPath.Positions;
        }

/*        public Vector3[] CreatePath2(Vector3 startPathPoint, Vector3 targetPathPoint)
        {

            Vector2 startPathPoint2D = new Vector2(startPathPoint.x, startPathPoint.z);
            Vector2 targetPathPoint2D = new Vector2(targetPathPoint.x, targetPathPoint.z);

            startPathPoint = new Vector3(startPathPoint.x, 1f, startPathPoint.z);
            targetPathPoint = new Vector3(targetPathPoint.x, 1f, targetPathPoint.z);

            MapTriangle startTriangle = GetTriangleByPoint(_triangles, startPathPoint);

            Debug.Log("START Triangle " + startTriangle.ToString());

            MapTriangle targetTriangle = GetTriangleByPoint(_triangles, targetPathPoint);

            Debug.Log("TARGET Triangle " + targetTriangle.ToString());

            if (startTriangle == targetTriangle)
            {
                return new Vector3[2] { startPathPoint, targetPathPoint };
            }

            AStarVector2Float targetClosestTriangleVertices = new AStarVector2Float(targetTriangle.Center.x, targetTriangle.Center.z);  
            //GetClosestVerticesToPoint(targetTriangle, startPathPoint2D, targetPathPoint2D);

            Debug.Log("Target Triangle Vertice" + targetClosestTriangleVertices);

            AStarVector2Float startClosestTriangleVertices = new AStarVector2Float(startTriangle.Center.x, startTriangle.Center.z); 
            //GetClosestVerticesToPoint(startTriangle, targetPathPoint2D, startPathPoint2D);

            Debug.Log("Start Triangle Vertice" + startClosestTriangleVertices);

            var graphNodesWithNeighboards = NeighboardCenterNodeUtility.GetNeighboardNode(startTriangle, _triangles);

            IReadOnlyCollection<AStarVector2Float> aStarPath = AStarPathResult(startClosestTriangleVertices, targetClosestTriangleVertices, graphNodesWithNeighboards);

            List<MapTriangle> pathTriangles = NodeMapTriangleUtility.GenerateNodeMapTriangle(
                _triangles,
                aStarPath,
                targetTriangle,
                startTriangle,
                targetPathPoint,
                startPathPoint);

            FunnelAlgorithmPath funnelAlgorithmPath = GetClearPath(pathTriangles);


            return funnelAlgorithmPath.Positions;
        }
*/

        private FunnelAlgorithmPath GetClearPath(List<MapTriangle> pathTriangles)
        {
            List<Triangle> funnelTriangles = new List<Triangle>();
            foreach (var tr in pathTriangles)
            {
                Triangle funnelTriangle = new Triangle(tr.Vertices[0], tr.Vertices[1], tr.Vertices[2]);
                funnelTriangles.Add(funnelTriangle);
            }


            var pathfinder = new FunnelPathfinder();
            var path = pathfinder.FindPath(funnelTriangles);


            LineRenderer anglelineRenderer = VisualUtil.GeneratePathLine(Color.black);
            anglelineRenderer.positionCount = path.Positions.Length;

            int i = 0;
            foreach (var p in path.Positions)
            {
                Debug.Log("FINAL PATH =>" + p);
                anglelineRenderer.SetPosition(i, new Vector3(p.x, p.y + 0.4f, p.z));
                i++;
            }

            return path;
        }

        private IReadOnlyCollection<AStarVector2Float> AStarPathResult(AStarVector2Float start,
            AStarVector2Float target,
            Dictionary<AStarVector2Float, List<AStarVector2Float>> graphNodesWithNeighboards)
        {

            AStarPath pathController = new AStarPath(graphNodesWithNeighboards);

            pathController.Calculate(start, target, null, out var path);

            Debug.Log("PATH POINT COUNT " + path.Count);

/*            LineRenderer pathLineRenderer = VisualUtil.GeneratePathLine(Color.red);
            int i = 0;
            pathLineRenderer.positionCount = path.Count + 1;

            pathLineRenderer.SetPosition(i, new Vector3(target.X, target.Y, target.Z));

            foreach (var p in path)
            {
                i++;
                pathLineRenderer.SetPosition(i, new Vector3(p.X, p.Y, p.Z));

            }*/

            return path;
        }

        private AStarVector2Float GetClosestVerticesToPoint(MapTriangle targetTriangle, Vector2 point, Vector2 pointInTargetTriangle)
        {
            float dist = float.MaxValue;
            Vector3 minVec = Vector3.zero;
            foreach (var vec in targetTriangle.Vertices)
            {
                float curDist = Vector2.Distance(new Vector2(vec.x, vec.z), new Vector2(point.x, point.y));

                curDist += Vector2.Distance(new Vector2(vec.x, vec.z), new Vector2(pointInTargetTriangle.x, pointInTargetTriangle.y));

                if (curDist < dist)
                {
                    dist = curDist;
                    minVec = vec;
                }
            }

          //  int targetRoundedX = (int)Math.Round(minVec.x, MidpointRounding.AwayFromZero);
         //   int targetRoundedZ = (int)Math.Round(minVec.z, MidpointRounding.AwayFromZero);
            AStarVector2Float target = new AStarVector2Float(minVec.x, minVec.y, minVec.z);

            return target;
        }

        private MapTriangle GetTriangleByPoint(List<MapTriangle> mapTriangles, Vector3 point)
        {

            MapTriangle foundTriangle = null;
            foreach (var tr in mapTriangles)
            {
                if (tr.PointInTriangle(point))
                {
                    foundTriangle = tr;
                    break;
                }
            }
            return foundTriangle;
        }

        private PathsD CreateMapFromColliders(List<GameObject> mapGameObjects)
        {
            // List<GameObject> mapGameObjects = GameObject.FindGameObjectsWithTag("map").ToList();
            PathsD subjects = new();
            PathsD solution;
            foreach (var go in mapGameObjects)
            {
                BoxCollider box = go.GetComponent<BoxCollider>();

                if (box != null)
                {
                    Vector3[] vertices = new Vector3[4];

                    vertices[0] = box.transform.TransformPoint(box.center + new Vector3(-box.size.x, box.size.y, -box.size.z) * 0.5f);
                    vertices[1] = box.transform.TransformPoint(box.center + new Vector3(box.size.x, box.size.y, -box.size.z) * 0.5f);
                    vertices[2] = box.transform.TransformPoint(box.center + new Vector3(box.size.x, box.size.y, box.size.z) * 0.5f);
                    vertices[3] = box.transform.TransformPoint(box.center + new Vector3(-box.size.x, box.size.y, box.size.z) * 0.5f);

                    PointD[] points = new PointD[]
                    {
                        new PointD(vertices[0].x, vertices[0].z),
                        new PointD(vertices[1].x, vertices[1].z),
                        new PointD(vertices[2].x, vertices[2].z),
                        new PointD(vertices[3].x, vertices[3].z)
                    };

                    PathD path = new PathD(points);
                    subjects.Add(path);
                }

                MeshCollider mesh = go.GetComponent<MeshCollider>();

                if (mesh != null)
                {
                    var bounds = FindBoundryEdges(mesh.sharedMesh.triangles);

                    List<int> withoutItem1 = mesh.sharedMesh.triangles.ToList();

                    foreach (var b in bounds)
                    {   
                        withoutItem1.RemoveAll(k => k == b.Item1);
                        withoutItem1.RemoveAll(k => k == b.Item2);
                    }

                    Debug.Log(mesh.sharedMesh.triangles.Length + " " + withoutItem1.Count);

                    bounds = FindBoundryEdges(withoutItem1.ToArray());

                    Vector3[] vertices = mesh.sharedMesh.vertices;
              //      int[] triangles = mesh.sharedMesh.triangles;

                    //     int trianglesCount = triangles.Length / 3;
                    PathD path = new PathD();

                    var firstEdge = bounds[0];

                    List<int> clockWiseIndexes = new List<int>();

                    clockWiseIndexes.Add(firstEdge.Item1);
                    clockWiseIndexes.Add(firstEdge.Item2);

                    int nextIndex = firstEdge.Item2;
                    (int, int) currentEdge = firstEdge;
                    int iteration = 0;
                    while (true)
                    {
                       var edge =  bounds.FirstOrDefault(i => i != currentEdge && (i.Item1 == nextIndex || i.Item2 == nextIndex));

                        if (edge.Item1 == 0 && edge.Item2 == 0)
                        {
                            break;
                        }

                        currentEdge = edge;
                        nextIndex = edge.Item1 == nextIndex ? edge.Item2 : edge.Item1;
                        clockWiseIndexes.Add(nextIndex);
                        iteration++;

                        bounds.Remove(edge);
                    }

                    List<Vector3> worldVertices = new List<Vector3>();
                    foreach (var vv in clockWiseIndexes)
                    {
                        Vector3 worldBound1 = go.transform.TransformPoint(vertices[vv]);
                        worldVertices.Add(worldBound1);
                    }

                    List<Vector3> worldVertices2 = new List<Vector3>();
                    List<Vector3> ignoredWorldVertices2 = new List<Vector3>();

                    for (int i=0; i < worldVertices.Count; i++)
                    {
                        Vector3 point = worldVertices[i];
                        
                        Vector3 pointWithTheSameXZ = worldVertices.FirstOrDefault(p => 
                        p.NearestEqualX(point) 
                        && p.y != point.y 
                        && p.NearestEqualZ(point));

                       
                        if (pointWithTheSameXZ.x == 0 && pointWithTheSameXZ.z == 0)
                        {
                            worldVertices2.Add(point);
                            continue;
                        }

/*                        if (ignoredWorldVertices2.Contains(point))
                        {
                            worldVertices2.Add(point);
                            continue;
                        }

                        ignoredWorldVertices2.Add(pointWithTheSameXZ);*/

                        if (i+1 >= worldVertices.Count)
                        {
                            continue;
                        }

                        Vector2 point2D = new Vector2(point.x, point.z);
                        Vector2 nextPoint = new Vector2(worldVertices[i + 1].x, worldVertices[i + 1].z);
                        Vector2 prevPoint = new Vector2(worldVertices[i - 1].x, worldVertices[i - 1].z);
                        Vector2 nextDirection = (nextPoint - point2D).normalized;
                        Vector2 prevDirection = (prevPoint - point2D).normalized;
                        float angle = Vector2.SignedAngle(prevDirection, nextDirection);
                        if(angle < 0)
                        {
                            angle = 360 + angle;
                        }
                        Vector2 nextPointMultiplier = point2D + nextDirection * 0.3f;
                        Vector2 movedPoint = CalculatePosition(point2D, nextPointMultiplier, angle/2f, true);
                        Debug.Log(point +" "+ movedPoint +" "+ angle);
                        worldVertices2.Add(new Vector3(movedPoint.x, point.y, movedPoint.y));
                    }


                    foreach (var worldVert in worldVertices2)
                    {
                        path.Add(new PointD((double)worldVert.x, (double)worldVert.z, (long) worldVert.y));

                        GameObject go2 = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        go2.transform.position = new Vector3(worldVert.x, worldVert.y, worldVert.z);
                        go2.transform.localScale = go2.transform.localScale * 0.4f;
                    }

                    //    var path2 = Clipper.TrimCollinear(path,true);
                    //      Debug.Log(path.Count + " " + path2.Count);

                    subjects.Add(path);

                }
            }

          // FillRule fillrule = FillRule.NonZero;
        //   solution = Clipper.Union(subjects, fillrule);

            return subjects;
        }

        private List<Vector3m> GetPointsForTriangulateFromMap(PathsD solution)
        {
            if (solution == null || solution.Count <= 0)
            {
                return null;
            }
            //TODO


            PathD solutionPath = solution.OrderByDescending(p => p.Count).FirstOrDefault();


/*            for (int k = 0; k < solution.Count; k++)
            {
                Path64 solutionPath1 = solution[k];
                LineRenderer clipperUnionLineRenderer1 = VisualUtil.GeneratePathLine(Color.cyan);
                clipperUnionLineRenderer1.positionCount = solutionPath1.Count;
                int s = 0;
                foreach (var p in solutionPath1)
                {
                    clipperUnionLineRenderer1.SetPosition(s, new Vector3(p.X, 8f, p.Y));
                    s++;
                }
            }*/

            if (solutionPath == null || solutionPath.Count <= 0)
            {
                return null;
            }
            Debug.Log("Solution vertices count before simplify " + solutionPath.Count);

            solutionPath = Clipper.SimplifyPath(solutionPath, 0, true);

            Debug.Log("Solution vertices count after simplify " + solutionPath.Count);

            LineRenderer clipperUnionLineRenderer = VisualUtil.GeneratePathLine(Color.cyan);
            clipperUnionLineRenderer.positionCount = solutionPath.Count;
            int i = 0;

            List<Vector3m> pointsForEarClipping = new List<Vector3m>();

            foreach (var p in solutionPath)
            {
                clipperUnionLineRenderer.SetPosition(i, new Vector3((float)p.x, 8+0.2f, (float)p.y));
                i++;

                pointsForEarClipping.Add(new Vector3m(p.x, 1, p.y));
            }

            return pointsForEarClipping;
        }

        private List<MapTriangle> TriangulateMap(List<Vector3m> pointsForEarClipping, List<List<Vector3m>> holes)
        {
            EarClipping earClipping = new EarClipping();
            earClipping.SetPoints(pointsForEarClipping, holes);
            earClipping.Triangulate();
            var res = earClipping.Result;

            int k = 0;

            List<MapTriangle> mapTriangles = new List<MapTriangle>();
            MapTriangle mapTriangle = null;
            LineRenderer anglelineRenderer = null;
            foreach (var p in res)
            {
                if (k == 0)
                {
                    anglelineRenderer = VisualUtil.GeneratePathLine(Color.green);
                    anglelineRenderer.positionCount = 3;
                    mapTriangle = new MapTriangle();
                }

                Vector3 curVector = new Vector3((float)p.X.ToDouble(), (float)p.Y.ToDouble(), (float)p.Z.ToDouble());
                anglelineRenderer.SetPosition(k, new Vector3((float)p.X.ToDouble(), (float)p.Y.ToDouble(), (float)p.Z.ToDouble()));
                mapTriangle.AddVertices(curVector);
                k++;
                if (k % 3 == 0)
                {
                    mapTriangles.Add(mapTriangle);
                    k = 0;
                }
            }

            return mapTriangles;
        }

        private List<List<Vector3m>> GetObstaclePointsForTriangulateFromMap(List<GameObject> obstacleGameObjects)
        {

            List<List<Vector3m>> obstaclePointsForEarClipping = new List<List<Vector3m>>();
            foreach (var go in obstacleGameObjects)
            {
                BoxCollider box = go.GetComponent<BoxCollider>();

                if (box != null)
                {
                    Vector3[] vertices = new Vector3[4];

                    vertices[0] = box.transform.TransformPoint(box.center + new Vector3(-box.size.x, box.size.y, -box.size.z) * 0.5f);
                    vertices[1] = box.transform.TransformPoint(box.center + new Vector3(box.size.x, box.size.y, -box.size.z) * 0.5f);
                    vertices[2] = box.transform.TransformPoint(box.center + new Vector3(box.size.x, box.size.y, box.size.z) * 0.5f);
                    vertices[3] = box.transform.TransformPoint(box.center + new Vector3(-box.size.x, box.size.y, box.size.z) * 0.5f);

/*                    Vector3 v0 = new Vector3(
                        (float) Math.Round(vertices[0].x, MidpointRounding.AwayFromZero),
                        1, 
                        (float)Math.Round(vertices[0].z, MidpointRounding.AwayFromZero));

                    Vector3 v1 = new Vector3(
                        (float)Math.Round(vertices[1].x, MidpointRounding.AwayFromZero), 
                        1,
                        (float)Math.Round(vertices[1].z, MidpointRounding.AwayFromZero));

                    Vector3 v2 = new Vector3(
                        (float)Math.Round(vertices[2].x, MidpointRounding.AwayFromZero),
                        1,
                        (float)Math.Round(vertices[2].z, MidpointRounding.AwayFromZero));

                    Vector3 v3 = new Vector3(
                        (float)Math.Round(vertices[3].x, MidpointRounding.AwayFromZero),
                        1,
                        (float)Math.Round(vertices[3].z, MidpointRounding.AwayFromZero));*/

                    List<Vector3m> vector3s = new List<Vector3m>()
                    {
                        new Vector3m(vertices[0].x, 1, vertices[0].z),
                        new Vector3m(vertices[1].x, 1, vertices[1].z),
                        new Vector3m(vertices[2].x, 1, vertices[2].z),
                        new Vector3m(vertices[3].x, 1, vertices[3].z)
                    };

                    foreach (var t in vertices)
                    {
                        GameObject go1 = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        go1.transform.position = new Vector3(t.x, t.y, t.z);
                        go1.transform.localScale = Vector3.one * 0.8f;

                    }

                    vector3s.Reverse();
                    obstaclePointsForEarClipping.Add(vector3s);
                }
            }

            return obstaclePointsForEarClipping;
        }


        private List<(int, int)> FindBoundryEdges(int[] triangles)
        {

            var edges = new Dictionary<(int, int), bool>();

            for (var i = 0; i < triangles.Length; i += 3)
            {
                var t0 = triangles[i];
                var t1 = triangles[i + 1];
                var t2 = triangles[i + 2];

                var e0 = t0 < t1 ? (t0, t1) : (t1, t0);
                var e1 = t1 < t2 ? (t1, t2) : (t2, t1);
                var e2 = t2 < t0 ? (t2, t0) : (t0, t2);

                edges[e0] = !edges.ContainsKey(e0);
                edges[e1] = !edges.ContainsKey(e1);
                edges[e2] = !edges.ContainsKey(e2);
            }

            return edges.Where(e => e.Value).Select(e => e.Key).ToList();
        }

        private  Vector2 PerpendicularClockwise( Vector2 vector2)
        {
            return new Vector2(vector2.y, -vector2.x);
        }

        private  Vector2 PerpendicularCounterClockwise( Vector2 vector2)
        {
            return new Vector2(-vector2.y, vector2.x);
        }

        private Vector2 CalculatePosition(Vector2 targetPos, Vector2 position, float angle, bool clockWise = false)
        {
            var angleInRadians = clockWise ? -angle * Mathf.Deg2Rad : angle * Mathf.Deg2Rad;

            var cosOfAngle = Mathf.Cos(angleInRadians);
            var sinOfAngle = Mathf.Sin(angleInRadians);

            var initialVector = new Vector2(cosOfAngle, sinOfAngle);
            var perpendicularVectorToInitial = new Vector2(-sinOfAngle, cosOfAngle);

            var newPositionX = (position.x - targetPos.x) * initialVector;
            var newPositionY = (position.y - targetPos.y) * perpendicularVectorToInitial;
            var finalPosition = newPositionX + newPositionY + targetPos;

            return finalPosition;
        }

        private double GetAngle(Vector2 a, Vector2 b)
        {

            var dot = a.x * b.x + a.y * b.y;
            var cross = a.x * b.y - a.y * b.x;

            double angle = Math.Acos(dot);

            var deg = (180 / Math.PI) * angle;

            double angle2 = Math.Atan2(cross, dot);

            var deg2 = (180 / Math.PI) * angle2;
            Debug.Log(deg + " "+ deg2);

        //    double angle = Math.Atan2(b.y, b.x) - Math.Atan2(a.y, a.x);
            return deg;
        }
    }
}
