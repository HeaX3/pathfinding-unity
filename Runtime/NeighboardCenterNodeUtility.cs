using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using FunnelAlgorithm;

namespace com.heax3.pathfinding_unity
{
    public static class NeighboardCenterNodeUtility
    {
        private static Dictionary<AStarVector2Float, List<AStarVector2Float>> _graphNodesWithNeighboards = new Dictionary<AStarVector2Float, List<AStarVector2Float>>();

        public static Dictionary<AStarVector2Float, List<AStarVector2Float>> GetNeighboardNode(MapTriangle startTriangle, List<MapTriangle> mapTriangles)
        {
            _graphNodesWithNeighboards.Clear();

            GetVertices(startTriangle, mapTriangles);

            return _graphNodesWithNeighboards;
        }

        private static void GetVertices(MapTriangle startTriangle, List<MapTriangle> mapTriangles)
        {
            AStarVector2Float startVector2Float = new AStarVector2Float(startTriangle.Center.x, startTriangle.Center.y, startTriangle.Center.z);

            List<AStarVector2Float> neighboards = new List<AStarVector2Float>();
            if (_graphNodesWithNeighboards.ContainsKey(startVector2Float))
            {
                return;
            }

            _graphNodesWithNeighboards.Add(startVector2Float, neighboards);
            foreach (var tr in startTriangle.LinkedMapTriangles)
            {
                AStarVector2Float currentVector2Float = new AStarVector2Float(tr.Center.x, tr.Center.y, tr.Center.z);

                if (startVector2Float.Equals(currentVector2Float))
                {
                    continue;
                }
                if (!neighboards.Contains(currentVector2Float))
                {
                    neighboards.Add(currentVector2Float);
                }
                GetVertices(tr, mapTriangles);
            }
        }
    }
}

