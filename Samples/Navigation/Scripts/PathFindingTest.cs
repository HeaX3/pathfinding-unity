using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.EnhancedTouch;
using Clipper2Lib;
using EarClipperLib;
using FunnelAlgorithm;
using System.Threading.Tasks;
using System.Threading;
using com.heax3.pathfinding_unity;

namespace com.heax3.pathfinding_unity
{
    public class PathFindingTest : MonoBehaviour
    {
        [SerializeField]
        Camera camera;

        [SerializeField]
        LocalNavMeshBuilder localNavMeshBuilder;

        Vector3 startPath = new Vector3(-3.47f, 4.00f, -20.52f);

        public InputAction _touch;

        void Awake()
        {
            TouchSimulation.Enable();

            _touch.started += OnTouch;
            _touch.Enable();

            Debug.Log("Awake");
        }
        AiPathfinding aiPathfinding;
        void Start()
        {
            DateTime startD = DateTime.UtcNow;

            var triangles = localNavMeshBuilder.CollectResourcesForNavMesh();

            aiPathfinding = new AiPathfinding(triangles);

            TimeSpan span = DateTime.UtcNow - startD;
            Debug.Log($"GENERATE NAV MESH Total Milliseconds {span.TotalMilliseconds}");

            
        }

        void OnTouch(InputAction.CallbackContext context)
        {
            Vector3 touch = Mouse.current.position.ReadValue();
            Debug.Log("OnTouch");
            Ray ray = camera.ScreenPointToRay(new Vector3(touch.x, touch.y, 0));
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, Mathf.Infinity))
            {
                Debug.Log("H I T" + hit.point);

                CreatePathToHitPoint(hit.point);
            }
        }


        private void CreatePathToHitPoint(Vector3 targetInWorld)
        {
            //     List<GameObject> mapGameObjects = GameObject.FindGameObjectsWithTag("map").ToList();

            //      AiPathfinding aiPathfinding = new AiPathfinding(mapGameObjects);

            targetInWorld = new Vector3(targetInWorld.x, targetInWorld.y, targetInWorld.z);

            DateTime startD = DateTime.UtcNow;

            aiPathfinding.CreatePath(startPath, targetInWorld);
            TimeSpan span = DateTime.UtcNow - startD;
            Debug.Log($"GENERATE PATH Total Milliseconds: {span.TotalMilliseconds}");

/*            ThreadPool.QueueUserWorkItem((obj)=> 
            {
                aiPathfinding.CreatePath(startPath, targetInWorld);
                TimeSpan span = DateTime.UtcNow - startD;
                Debug.Log($"GENERATE PATH {span.TotalMilliseconds}");
            });*/

        }

    }
}
