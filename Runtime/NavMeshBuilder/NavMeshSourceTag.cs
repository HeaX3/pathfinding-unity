using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;


namespace com.heax3.pathfinding_unity
{
    [DefaultExecutionOrder(-200)]
    public class NavMeshSourceTag : MonoBehaviour
    {
        public static List<MeshFilter> m_Meshes = new List<MeshFilter>();
        public static List<Terrain> m_Terrains = new List<Terrain>();

        void OnEnable()
        {
            var m = GetComponent<MeshFilter>();
            if (m != null)
            {
                m_Meshes.Add(m);
            }

            var t = GetComponent<Terrain>();
            if (t != null)
            {
                m_Terrains.Add(t);
            }
        }

        void OnDisable()
        {
            var m = GetComponent<MeshFilter>();
            if (m != null)
            {
                m_Meshes.Remove(m);
            }

            var t = GetComponent<Terrain>();
            if (t != null)
            {
                m_Terrains.Remove(t);
            }
        }

        public static void Collect(ref List<NavMeshBuildSource> sources)
        {
            sources.Clear();

            for (var i = 0; i < m_Meshes.Count; ++i)
            {
                var mf = m_Meshes[i];
                if (mf == null) continue;

                var m = mf.sharedMesh;
                if (m == null) continue;

                var s = new NavMeshBuildSource();
                s.shape = NavMeshBuildSourceShape.Mesh;
                s.sourceObject = m;
                s.transform = mf.transform.localToWorldMatrix;
                s.area = 0;
                sources.Add(s);
            }

            for (var i = 0; i < m_Terrains.Count; ++i)
            {
                var t = m_Terrains[i];
                if (t == null) continue;

                var s = new NavMeshBuildSource();
                s.shape = NavMeshBuildSourceShape.Terrain;
                s.sourceObject = t.terrainData;
                s.transform = Matrix4x4.TRS(t.transform.position, Quaternion.identity, Vector3.one);
                s.area = 0;
                sources.Add(s);
            }
        }
    }
}
