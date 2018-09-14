using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace PhysicsEngine
{
    public class DebugUtils
    { 
        public static void DrawAABB(AABB aabb, Color color)
        {
            Debug.DrawLine(new Vector3(aabb.Min.x, aabb.Min.y, aabb.Min.z), new Vector3(aabb.Max.x, aabb.Min.y, aabb.Min.z), color);
            Debug.DrawLine(new Vector3(aabb.Max.x, aabb.Min.y, aabb.Min.z), new Vector3(aabb.Max.x, aabb.Min.y, aabb.Max.z), color);
            Debug.DrawLine(new Vector3(aabb.Max.x, aabb.Min.y, aabb.Max.z), new Vector3(aabb.Min.x, aabb.Min.y, aabb.Max.z), color);
            Debug.DrawLine(new Vector3(aabb.Min.x, aabb.Min.y, aabb.Max.z), new Vector3(aabb.Min.x, aabb.Min.y, aabb.Min.z), color);

            Debug.DrawLine(new Vector3(aabb.Min.x, aabb.Min.y, aabb.Min.z), new Vector3(aabb.Min.x, aabb.Max.y, aabb.Min.z), color);
            Debug.DrawLine(new Vector3(aabb.Max.x, aabb.Min.y, aabb.Min.z), new Vector3(aabb.Max.x, aabb.Max.y, aabb.Min.z), color);
            Debug.DrawLine(new Vector3(aabb.Max.x, aabb.Min.y, aabb.Max.z), new Vector3(aabb.Max.x, aabb.Max.y, aabb.Max.z), color);
            Debug.DrawLine(new Vector3(aabb.Min.x, aabb.Min.y, aabb.Max.z), new Vector3(aabb.Min.x, aabb.Max.y, aabb.Max.z), color);

            Debug.DrawLine(new Vector3(aabb.Min.x, aabb.Max.y, aabb.Min.z), new Vector3(aabb.Max.x, aabb.Max.y, aabb.Min.z), color);
            Debug.DrawLine(new Vector3(aabb.Max.x, aabb.Max.y, aabb.Min.z), new Vector3(aabb.Max.x, aabb.Max.y, aabb.Max.z), color);
            Debug.DrawLine(new Vector3(aabb.Max.x, aabb.Max.y, aabb.Max.z), new Vector3(aabb.Min.x, aabb.Max.y, aabb.Max.z), color);
            Debug.DrawLine(new Vector3(aabb.Min.x, aabb.Max.y, aabb.Max.z), new Vector3(aabb.Min.x, aabb.Max.y, aabb.Min.z), color);
        }
    }
}
