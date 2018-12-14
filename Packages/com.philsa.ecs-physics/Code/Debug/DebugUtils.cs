using UnityEngine;

namespace PhysicsEngine
{
    public class DebugUtils
    {
        public static void DrawAABB(AABB aabb, Color color)
        {
            Debug.DrawLine(new Vector3((float)aabb.Min.x, (float)aabb.Min.y, (float)aabb.Min.z), new Vector3((float)aabb.Max.x, (float)aabb.Min.y, (float)aabb.Min.z), color);
            Debug.DrawLine(new Vector3((float)aabb.Max.x, (float)aabb.Min.y, (float)aabb.Min.z), new Vector3((float)aabb.Max.x, (float)aabb.Min.y, (float)aabb.Max.z), color);
            Debug.DrawLine(new Vector3((float)aabb.Max.x, (float)aabb.Min.y, (float)aabb.Max.z), new Vector3((float)aabb.Min.x, (float)aabb.Min.y, (float)aabb.Max.z), color);
            Debug.DrawLine(new Vector3((float)aabb.Min.x, (float)aabb.Min.y, (float)aabb.Max.z), new Vector3((float)aabb.Min.x, (float)aabb.Min.y, (float)aabb.Min.z), color);

            Debug.DrawLine(new Vector3((float)aabb.Min.x, (float)aabb.Min.y, (float)aabb.Min.z), new Vector3((float)aabb.Min.x, (float)aabb.Max.y, (float)aabb.Min.z), color);
            Debug.DrawLine(new Vector3((float)aabb.Max.x, (float)aabb.Min.y, (float)aabb.Min.z), new Vector3((float)aabb.Max.x, (float)aabb.Max.y, (float)aabb.Min.z), color);
            Debug.DrawLine(new Vector3((float)aabb.Max.x, (float)aabb.Min.y, (float)aabb.Max.z), new Vector3((float)aabb.Max.x, (float)aabb.Max.y, (float)aabb.Max.z), color);
            Debug.DrawLine(new Vector3((float)aabb.Min.x, (float)aabb.Min.y, (float)aabb.Max.z), new Vector3((float)aabb.Min.x, (float)aabb.Max.y, (float)aabb.Max.z), color);

            Debug.DrawLine(new Vector3((float)aabb.Min.x, (float)aabb.Max.y, (float)aabb.Min.z), new Vector3((float)aabb.Max.x, (float)aabb.Max.y, (float)aabb.Min.z), color);
            Debug.DrawLine(new Vector3((float)aabb.Max.x, (float)aabb.Max.y, (float)aabb.Min.z), new Vector3((float)aabb.Max.x, (float)aabb.Max.y, (float)aabb.Max.z), color);
            Debug.DrawLine(new Vector3((float)aabb.Max.x, (float)aabb.Max.y, (float)aabb.Max.z), new Vector3((float)aabb.Min.x, (float)aabb.Max.y, (float)aabb.Max.z), color);
            Debug.DrawLine(new Vector3((float)aabb.Min.x, (float)aabb.Max.y, (float)aabb.Max.z), new Vector3((float)aabb.Min.x, (float)aabb.Max.y, (float)aabb.Min.z), color);
        }
    }
}
