using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using FunnelAlgorithm;

namespace com.heax3.pathfinding_unity
{
    public static class NodeMapTriangleUtility
    {
        private static List<MapTriangle> pathTriangles = new List<MapTriangle>();
        private static MapTriangle _startTriangle;
        public static List<MapTriangle> GenerateNodeMapTriangle(List<MapTriangle> triangles, IReadOnlyCollection<AStarVector2Float> aStarPath, MapTriangle targetTriangle, MapTriangle startTriangle, Vector3 targetInWorldPoint, Vector3 startInWorldPoint)
        {
            pathTriangles.Clear();
            _startTriangle = startTriangle;

            IReadOnlyCollection<AStarVector2Float> oppositePath = aStarPath.Reverse().ToArray();

            List<MapTriangle> allMapTriangles = new List<MapTriangle>();

            foreach (AStarVector2Float p in oppositePath)
            {
                List<MapTriangle> mapTriangles = triangles.Where(s => s.PointInTriangle(new Vector3(p.X, p.Y, p.Z))).ToList();
                allMapTriangles.AddRange(mapTriangles);
            }

            foreach (var currentMapTriangle in allMapTriangles)
            {
                currentMapTriangle.LinkedMapTriangles.Clear();

                foreach (var edge in currentMapTriangle.Edges)
                {
                    MapTriangle foundMapTriangle = allMapTriangles.FirstOrDefault

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

            MapTriangle firstPathElementTriangle = allMapTriangles.FirstOrDefault(f => f == startTriangle);

            PathElement newPathElement = new PathElement();
            newPathElement.CurrentMapTriangle = firstPathElementTriangle;
            newPathElement.LinkedMapTriangle = firstPathElementTriangle;
            newPathElement.LinkedElementPath = null;
            CheckMapTriangle(firstPathElementTriangle, targetTriangle, newPathElement);

            List<List<MapTriangle>> trianglesArray = new List<List<MapTriangle>>();
            List<MapTriangle> triangleArray = new List<MapTriangle>();
            foreach (var partPath in pathTriangles)
            {
                if (partPath == targetTriangle)
                {
                    triangleArray = new List<MapTriangle>();
                    trianglesArray.Add(triangleArray);
                }

                triangleArray.Add(partPath);
            }

            pathTriangles = trianglesArray.OrderBy(t => t.Count).FirstOrDefault();

            pathTriangles[0] = GetFinalTriangle(targetInWorldPoint);

            List<MapTriangle> pathTrianglesResult = new List<MapTriangle>();

            for (int i = 0; i < pathTriangles.Count; i++)
            {
                MapTriangle triangleInPath = pathTriangles[i];
                if (triangleInPath.PointInTriangle(startInWorldPoint))
                {
                    pathTrianglesResult.Add(triangleInPath);
                    break;
                }
                else
                {
                    pathTrianglesResult.Add(triangleInPath);
                }
            }

            pathTrianglesResult[pathTrianglesResult.Count - 1] = GetFirstTriangle(startInWorldPoint, pathTrianglesResult);

            pathTrianglesResult.Reverse();

            return pathTrianglesResult;
        }

        private static void CheckMapTriangle(MapTriangle currentMapTriangle, MapTriangle finalTriangle, PathElement prevnewPathElement)
        {
            List<MapTriangle> ignored = new List<MapTriangle>();
            PathElement pe = prevnewPathElement;
            while (pe.LinkedElementPath != null)
            {
                ignored.Add(pe.LinkedMapTriangle);
                pe = pe.LinkedElementPath;
            }

            foreach (var c in currentMapTriangle.LinkedMapTriangles)
            {
                if (ignored.Contains(c))
                {
                    continue;
                }

                PathElement newPathElement = new PathElement();
                newPathElement.CurrentMapTriangle = currentMapTriangle;
                newPathElement.LinkedMapTriangle = c;
                newPathElement.LinkedElementPath = prevnewPathElement;

                if (c == finalTriangle || c.ToString() == finalTriangle.ToString())
                {

                    PathElement p = newPathElement;
                    while (p.LinkedElementPath != null)
                    {
                        pathTriangles.Add(p.LinkedMapTriangle);
                        p = p.LinkedElementPath;
                    }

                    if (pathTriangles.Count == 1)
                    {
                        foreach (var fp in pathTriangles[0].Edges)
                        {
                            var foundEdge = _startTriangle.Edges.FirstOrDefault(r => r.Equals(fp));

                            if (foundEdge != null)
                            {
                                pathTriangles.Add(_startTriangle);
                                break;
                            }
                        }
                    }
                    break;
                }
                else
                {
                    CheckMapTriangle(c, finalTriangle, newPathElement);
                }
            }
        }

        private static MapTriangle GetFinalTriangle(Vector3 targetInWorld)
        {
            MapTriangle newTriangle = new MapTriangle();
            foreach (var fp in pathTriangles[0].Edges)
            {
                var foundEdge = pathTriangles[1].Edges.FirstOrDefault(r => r.Equals(fp));

                if (foundEdge != null)
                {
                    newTriangle.AddVertices(foundEdge.A);
                    newTriangle.AddVertices(foundEdge.B);
                    newTriangle.AddVertices(targetInWorld);
                }
            }

            return newTriangle;
        }

        private static MapTriangle GetFirstTriangle(Vector3 targetInWorld, List<MapTriangle> pathTrianglesResult)
        {
            MapTriangle newTriangle = new MapTriangle();
            foreach (var fp in pathTrianglesResult[pathTrianglesResult.Count - 1].Edges)
            {
                var foundEdge = pathTrianglesResult[pathTrianglesResult.Count - 1 - 1].Edges.FirstOrDefault(r => r.Equals(fp));

                if (foundEdge != null)
                {
                    newTriangle.AddVertices(foundEdge.A);
                    newTriangle.AddVertices(foundEdge.B);
                    newTriangle.AddVertices(targetInWorld);
                }
            }

            return newTriangle;
        }
    }
}
