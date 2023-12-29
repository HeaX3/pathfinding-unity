using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using FunnelAlgorithm;
using UnityEngine.AI;

namespace com.heax3.pathfinding_unity
{
    public class AiPathfinding
    {
        List<MapTriangle> _triangles;

        public AiPathfinding(NavMeshTriangulation navMeshTriangulation)
        {
            int k = 0;
            List<MapTriangle> mapTriangles = new List<MapTriangle>();
            MapTriangle mapTriangle = null;

            for (var i = 0; i < navMeshTriangulation.indices.Length; i += 3)
            {
                var t0 = navMeshTriangulation.indices[i];
                var t1 = navMeshTriangulation.indices[i + 1];
                var t2 = navMeshTriangulation.indices[i + 2];

                mapTriangle = new MapTriangle();

                Vector3 v0 = navMeshTriangulation.vertices[t0];
                Vector3 v1 = navMeshTriangulation.vertices[t1];
                Vector3 v2 = navMeshTriangulation.vertices[t2];

                float v0x = (float)Math.Round(v0.x, 3);
                float v0y = (float)Math.Round(v0.y, 3);
                float v0z = (float)Math.Round(v0.z, 3);

                float v1x = (float)Math.Round(v1.x, 3);
                float v1y = (float)Math.Round(v1.y, 3);
                float v1z = (float)Math.Round(v1.z, 3);

                float v2x = (float)Math.Round(v2.x, 3);
                float v2y = (float)Math.Round(v2.y, 3);
                float v2z = (float)Math.Round(v2.z, 3);

                mapTriangle.AddVertices(new Vector3(v0x, v0y, v0z));
                mapTriangle.AddVertices(new Vector3(v1x, v1y, v1z));
                mapTriangle.AddVertices(new Vector3(v2x, v2y, v2z));

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

            if (startTriangle == null)
            {
                Debug.Log("==> START POINT OUT OF MAP");
                return Array.Empty<Vector3>();
            }

            MapTriangle targetTriangle = GetTriangleByPoint(_triangles, targetPathPoint);

            if (targetTriangle == null)
            {
                Debug.Log("==> TARGET POINT OUT OF MAP");
                return Array.Empty<Vector3>();
            }

            if (startTriangle == targetTriangle)
            {
                return new Vector3[2] { startPathPoint, targetPathPoint };
            }

            AStarVector2Float startClosestTriangleVertices = GetClosestVerticesToPoint(startTriangle, targetPathPoint2D, startPathPoint2D);

            var graphNodesWithNeighboards = NeighboardNodeUtility.GetNeighboardNode(startClosestTriangleVertices, targetTriangle, targetPathPoint, _triangles);

            var targetPoint2dFloat = new AStarVector2Float(targetPathPoint.x, targetPathPoint.y, targetPathPoint.z);

            IReadOnlyCollection<AStarVector2Float> aStarPath = AStarPathResult(startClosestTriangleVertices, targetPoint2dFloat, graphNodesWithNeighboards);

            List<MapTriangle> pathTriangles = NodeMapTriangleUtility.GenerateNodeMapTriangle(_triangles, aStarPath, targetTriangle, startTriangle, targetPathPoint, startPathPoint);

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
            return path;
        }

        private IReadOnlyCollection<AStarVector2Float> AStarPathResult(AStarVector2Float start, AStarVector2Float target,
            Dictionary<AStarVector2Float, List<AStarVector2Float>> graphNodesWithNeighboards)
        {

            AStarPath pathController = new AStarPath(graphNodesWithNeighboards);
            pathController.Calculate(start, target, null, out var path);
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
    }
}
