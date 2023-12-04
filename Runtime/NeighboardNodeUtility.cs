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
    public static class NeighboardNodeUtility
    {
       private static Dictionary<AStarVector2Float, List<AStarVector2Float>> _graphNodesWithNeighboards = new Dictionary<AStarVector2Float, List<AStarVector2Float>>();

        public static Dictionary<AStarVector2Float, List<AStarVector2Float>> GetNeighboardNode(AStarVector2Float startPath,
            MapTriangle targetTriangle,
            Vector3 targetPathPoint,
            List<MapTriangle> mapTriangles)
        {
            _graphNodesWithNeighboards.Clear();

            GetVertices(new Vector3(startPath.X, startPath.Y, startPath.Z), mapTriangles);

            foreach(var v in targetTriangle.Vertices)
            {
                AStarVector2Float currentVector2Float = new AStarVector2Float(v.x, v.y, v.z);

                if (_graphNodesWithNeighboards.ContainsKey(currentVector2Float))
                {
                    _graphNodesWithNeighboards[currentVector2Float].Add(new AStarVector2Float(targetPathPoint.x, targetPathPoint.y, targetPathPoint.z));
                }
            }

/*            foreach (var v in _graphNodesWithNeighboards)
            {
                foreach (var vv in v.Value)
                {
                    GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    go.transform.position = new Vector3(vv.X, vv.Y, vv.Z);
                    go.transform.localScale = go.transform.localScale * 0.8f;
                }
            }*/

            return _graphNodesWithNeighboards;
        }

        private static void GetVertices(Vector3 start, List<MapTriangle> mapTriangles)
        {
         //   int startRoundedX = (int)Math.Round(start.x, MidpointRounding.AwayFromZero);
        //    int startRoundedZ = (int)Math.Round(start.z, MidpointRounding.AwayFromZero);
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
                 //   int roundedX = (int)Math.Round(v.x, MidpointRounding.AwayFromZero);
                //    int roundedZ = (int)Math.Round(v.z, MidpointRounding.AwayFromZero);
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
