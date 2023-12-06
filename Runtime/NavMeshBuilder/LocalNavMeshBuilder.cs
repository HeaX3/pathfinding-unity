using UnityEngine;
using UnityEngine.AI;
using System.Collections;
using System.Collections.Generic;
using NavMeshBuilder = UnityEngine.AI.NavMeshBuilder;
using com.heax3.pathfinding_unity;

namespace com.heax3.pathfinding_unity
{
    // Build and update a localized navmesh from the sources marked by NavMeshSourceTag
    [DefaultExecutionOrder(-102)]
    public class LocalNavMeshBuilder : MonoBehaviour
    {

        public Transform Tracked;
        public Vector3 Size = new Vector3(80.0f, 20.0f, 80.0f);

        NavMeshData navMesh;
        AsyncOperation operation;
        NavMeshDataInstance instance;
        List<NavMeshBuildSource> sources = new List<NavMeshBuildSource>();
        public NavMeshTriangulation triangles;


        public NavMeshTriangulation CollectResourcesForNavMesh(bool cleanNavMeshAfterBuild = false)
        {

            navMesh = new NavMeshData();
            instance = UnityEngine.AI.NavMesh.AddNavMeshData(navMesh);

            NavMeshTriangulation triangles = UpdateNavMesh(false);

            if (cleanNavMeshAfterBuild)
            {
                NavMesh.RemoveNavMeshData(instance);
            }

            return triangles;
        }

        void OnDisable()
        {
            instance.Remove();
        }

        private NavMeshTriangulation UpdateNavMesh(bool asyncUpdate = false)
        {
            NavMeshSourceTag.Collect(ref sources);
            var defaultBuildSettings = UnityEngine.AI.NavMesh.GetSettingsByID(0);

            Debug.Log(sources.Count);

            var bounds = QuantizedBounds();

            if (asyncUpdate)
                operation = NavMeshBuilder.UpdateNavMeshDataAsync(navMesh, defaultBuildSettings, sources, bounds);
            else
                NavMeshBuilder.UpdateNavMeshData(navMesh, defaultBuildSettings, sources, bounds);

            return UnityEngine.AI.NavMesh.CalculateTriangulation();

        }

        static Vector3 Quantize(Vector3 v, Vector3 quant)
        {
            float x = quant.x * Mathf.Floor(v.x / quant.x);
            float y = quant.y * Mathf.Floor(v.y / quant.y);
            float z = quant.z * Mathf.Floor(v.z / quant.z);
            return new Vector3(x, y, z);
        }

        Bounds QuantizedBounds()
        {
            // Quantize the bounds to update only when theres a 10% change in size
            var center = Tracked ? Tracked.position : transform.position;
            return new Bounds(Quantize(center, 0.1f * Size), Size);
        }

        void OnDrawGizmosSelected()
        {
            if (navMesh)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawWireCube(navMesh.sourceBounds.center, navMesh.sourceBounds.size);
            }

            Gizmos.color = Color.yellow;
            var bounds = QuantizedBounds();
            Gizmos.DrawWireCube(bounds.center, bounds.size);

            Gizmos.color = Color.green;
            var center = Tracked ? Tracked.position : transform.position;
            Gizmos.DrawWireCube(center, Size);
        }
    }
}
