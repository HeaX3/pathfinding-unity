using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace com.heax3.pathfinding_unity
{
    public static class Vector3Extensions
    {

        public static bool NearestEqualX(this Vector3 v1, Vector3 v2)
        {
            var v1x = Math.Abs(v1.x);
            var v2x = Math.Abs(v2.x);

            return Math.Abs(v1x - v2x) < 0.5f;
        }

        public static bool NearestEqualZ(this Vector3 v1, Vector3 v2)
        {
            var v1z = Math.Abs(v1.z);
            var v2z = Math.Abs(v2.z);

            return Math.Abs(v1z - v2z) < 0.5f;
        }
    }
}
