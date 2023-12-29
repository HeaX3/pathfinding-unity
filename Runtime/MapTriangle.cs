using FunnelAlgorithm;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace com.heax3.pathfinding_unity
{
    public class MapTriangle
    {
        public List<Vector3> Vertices;

        public List<Edge> Edges;

        public List<MapTriangle> LinkedMapTriangles;

        public Vector3 Center;

        [SerializeField]
        private Edge ab;
        public Edge AB { get { return ab; } private set { ab = value; } }
        [SerializeField]
        private Edge bc;
        public Edge BC { get { return bc; } private set { bc = value; } }
        [SerializeField]
        private Edge ca;
        public Edge CA { get { return ca; } private set { ca = value; } }

        public MapTriangle()
        {
            Vertices = new List<Vector3>();
            Edges = new List<Edge>();
            LinkedMapTriangles = new List<MapTriangle>();
        }

        public void AddVertices(Vector3 vert)
        {
            Vertices.Add(vert);

            if(Vertices.Count == 3)
            {
                AB = new Edge(Vertices[0], Vertices[1]);
                BC = new Edge(Vertices[1], Vertices[2]);
                CA = new Edge(Vertices[2], Vertices[0]);

                Edges.Add(AB);
                Edges.Add(BC);
                Edges.Add(CA);

                float centerX = (Vertices[0].x + Vertices[1].x + Vertices[2].x) / 3;
                float centerY = (Vertices[0].y + Vertices[1].y + Vertices[2].y) / 3;
                float centerZ = (Vertices[0].z + Vertices[1].z + Vertices[2].z) / 3;

                Center = new Vector3(centerX, centerY, centerZ);
            }
        }

        public string ToString()
        {
            return $"V1 {Vertices[0]} V2{Vertices[1]} V3{Vertices[2]}";
        }

        public bool PointInTriangle(Vector3 pt)
        {
            Vector3 v1 = Vertices[0];
            Vector3 v2 = Vertices[1];
            Vector3 v3 = Vertices[2];

            float d1, d2, d3;
            bool has_neg, has_pos;

            d1 = Sign(pt, v1, v2);
            d2 = Sign(pt, v2, v3);
            d3 = Sign(pt, v3, v1);

            has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

            return !(has_neg && has_pos);
        }
        float Sign(Vector3 p1, Vector3 p2, Vector3 p3)
        {
            return (p1.x - p3.x) * (p2.z - p3.z) - (p2.x - p3.x) * (p1.z - p3.z);
        }
    }

    public class PathElement
    {
        public PathElement LinkedElementPath;
        public MapTriangle CurrentMapTriangle;
        public MapTriangle LinkedMapTriangle;
    }


}
