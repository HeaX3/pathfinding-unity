using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using Clipper2Lib;
using EarClipperLib;
using FunnelAlgorithm;

namespace com.heax3.pathfinding_unity
{
    public class AiPathfinding
    {
        List<MapTriangle> _triangles;
        public AiPathfinding(List<GameObject> mapGameObjects, List<GameObject> obstacleGameObjects = null)
        {
            Paths64 solution = CreateMapFromColliders(mapGameObjects);

            List<List<Vector3m>> holes = null;
            if (obstacleGameObjects != null && obstacleGameObjects.Count > 0)
            {
                holes = GetObstaclePointsForTriangulateFromMap(obstacleGameObjects);
            }

            List<Vector3m> areaForTriangulate = GetPointsForTriangulateFromMap(solution);

            _triangles = TriangulateMap(areaForTriangulate, holes);
        }

        public Vector3[] CreatePath(Vector3 startPathPoint, Vector3 targetPathPoint)
        {

            Vector2 startPathPoint2D = new Vector2(startPathPoint.x, startPathPoint.z);
            Vector2 targetPathPoint2D = new Vector2(targetPathPoint.x, targetPathPoint.z);

            startPathPoint = new Vector3(startPathPoint.x, 1, startPathPoint.z);
            targetPathPoint = new Vector3(targetPathPoint.x, 1, targetPathPoint.z);

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
                anglelineRenderer.SetPosition(i, new Vector3(p.x, p.y + 0.9f, p.z));
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

            LineRenderer pathLineRenderer = VisualUtil.GeneratePathLine(Color.red);
            int i = 0;
            pathLineRenderer.positionCount = path.Count + 1;

            pathLineRenderer.SetPosition(i, new Vector3(target.X, 1.2f, target.Y));

            foreach (var p in path)
            {
                i++;
                pathLineRenderer.SetPosition(i, new Vector3(p.X, 1.3f, p.Y));

            }

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
            AStarVector2Float target = new AStarVector2Float(minVec.x, minVec.z);

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

        private Paths64 CreateMapFromColliders(List<GameObject> mapGameObjects)
        {
            // List<GameObject> mapGameObjects = GameObject.FindGameObjectsWithTag("map").ToList();
            Paths64 subjects = new();
            Paths64 solution;
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

                    Point64[] points = new Point64[]
                    {
                        new Point64(vertices[0].x, vertices[0].z),
                        new Point64(vertices[1].x, vertices[1].z),
                        new Point64(vertices[2].x, vertices[2].z),
                        new Point64(vertices[3].x, vertices[3].z)
                    };

                    Path64 path = new Path64(points);
                    subjects.Add(path);
                }

                MeshCollider mesh = go.GetComponent<MeshCollider>();

     
                if (mesh != null)
                {

                    Vector3[] vertices = mesh.sharedMesh.vertices;
                    int[] triangles = mesh.sharedMesh.triangles;

                    int trianglesCount = triangles.Length / 3;

                    for (int i = 0; i < trianglesCount; i++)
                    {
                        Vector3 p0 = vertices[triangles[i * 3 + 0]];
                        Vector3 p1 = vertices[triangles[i * 3 + 1]];
                        Vector3 p2 = vertices[triangles[i * 3 + 2]];

                        Vector3 p0World = go.transform.TransformPoint(p0);
                        Vector3 p1World = go.transform.TransformPoint(p1);
                        Vector3 p2World = go.transform.TransformPoint(p2);

                        Point64[] points = new Point64[]
                        {
                            new Point64(p0World.x, p0World.z),
                            new Point64(p1World.x, p1World.z),
                            new Point64(p2World.x, p2World.z)
                        };

                        Path64 path = new Path64(points);
                        subjects.Add(path);
                    }
                }
            }

            FillRule fillrule = FillRule.NonZero;
            solution = Clipper.Union(subjects, fillrule);

            return solution;
        }

        private List<Vector3m> GetPointsForTriangulateFromMap(Paths64 solution)
        {
            if (solution == null || solution.Count <= 0)
            {
                return null;
            }
            //TODO


            Path64 solutionPath = solution.OrderByDescending(p => p.Count).FirstOrDefault();


/*            for(int k=0; k < 3; k++)
            {
                Path64 solutionPath1 = solution[k];
                LineRenderer clipperUnionLineRenderer1 = VisualUtil.GeneratePathLine(Color.cyan);
                clipperUnionLineRenderer1.positionCount = solutionPath1.Count;
                int s = 0;
                foreach (var p in solutionPath1)
                {
                    clipperUnionLineRenderer1.SetPosition(s, new Vector3(p.X, 5f, p.Y));
                    s++;
                }
            }*/

            if (solutionPath == null || solutionPath.Count <= 0)
            {
                return null;
            }
            Debug.Log("Solution vertices count before simplify " + solutionPath.Count);

            solutionPath = Clipper.SimplifyPath(solutionPath, 1, true);

            Debug.Log("Solution vertices count after simplify " + solutionPath.Count);

            LineRenderer clipperUnionLineRenderer = VisualUtil.GeneratePathLine(Color.cyan);
            clipperUnionLineRenderer.positionCount = solutionPath.Count;
            int i = 0;

            List<Vector3m> pointsForEarClipping = new List<Vector3m>();

            foreach (var p in solutionPath)
            {
                clipperUnionLineRenderer.SetPosition(i, new Vector3(p.X, 1f, p.Y));
                i++;

                pointsForEarClipping.Add(new Vector3m(p.X, 1, p.Y));
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


    }
}
