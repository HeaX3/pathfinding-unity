using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using FunnelAlgorithm;

namespace com.heax3.pathfinding_unity
{
    public static class NeighboardNodeUtility
    {
        private static Dictionary<AStarVector2Float, List<AStarVector2Float>> _graphNodesWithNeighboards = new Dictionary<AStarVector2Float, List<AStarVector2Float>>();

        public static Dictionary<AStarVector2Float, List<AStarVector2Float>> GetNeighboardNode(AStarVector2Float startPath, MapTriangle targetTriangle, Vector3 targetPathPoint, List<MapTriangle> mapTriangles)
        {
            _graphNodesWithNeighboards.Clear();

            GetVertices(new Vector3(startPath.X, startPath.Y, startPath.Z), mapTriangles);

            foreach (var v in targetTriangle.Vertices)
            {
                AStarVector2Float currentVector2Float = new AStarVector2Float(v.x, v.y, v.z);

                if (_graphNodesWithNeighboards.ContainsKey(currentVector2Float))
                {
                    _graphNodesWithNeighboards[currentVector2Float].Add(new AStarVector2Float(targetPathPoint.x, targetPathPoint.y, targetPathPoint.z));
                }
            }
            return _graphNodesWithNeighboards;
        }

        private static void GetVertices(Vector3 start, List<MapTriangle> mapTriangles)
        {
            AStarVector2Float startVector2Float = new AStarVector2Float(start.x, start.y, start.z);
            List<AStarVector2Float> neighboards = new List<AStarVector2Float>();

            if (_graphNodesWithNeighboards.ContainsKey(startVector2Float))
            {
                return;
            }

            _graphNodesWithNeighboards.Add(startVector2Float, neighboards);

            foreach (var tr in mapTriangles)
            {
                Vector3 v1 = tr.Vertices[0];
                Vector3 v2 = tr.Vertices[1];
                Vector3 v3 = tr.Vertices[2];

                if (!v1.NearestEqualXZ(start)
                    && !v2.NearestEqualXZ(start)
                    && !v3.NearestEqualXZ(start))
                {
                    continue;
                }

                foreach (var v in tr.Vertices)
                {
                    AStarVector2Float currentVector2Float = new AStarVector2Float(v.x, v.y, v.z);

                    if (startVector2Float.Equals(currentVector2Float))
                    {
                        continue;
                    }

                    if (!neighboards.Contains(currentVector2Float))
                    {
                        neighboards.Add(currentVector2Float);
                    }

                    GetVertices(v, mapTriangles);
                }
            }
        }
    }
}
