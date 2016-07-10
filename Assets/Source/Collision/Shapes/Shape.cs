using UnityEngine;
namespace CrispyPhysics
{
    public struct MassData
    {
        float mass;
        Vector2 center;
        float rotationalGravity;
    }

    public class Shape
    {
        public enum Types
        {
            Circle = 0,
            Edge = 1,
            Polygon = 2,
            Count = 3
        }

        public Types type { get; private set; }
        public Collider2D collider { get; private set; }

        public Shape(Collider2D collider)
        {
            Debug.Assert(collider != null);
            this.collider = collider;
        }

        public int GetChildCount()
        {
            return collider.shapeCount;
        }

        public bool TestPoint(Vector2 point)
        {
            Debug.Assert(false, "Test Point");
            return false;
        }

        public bool RayCast()
        {
            Debug.Assert(false, "Ray Cast On Shape");
            return false;
        }

        public MassData GetMassData()
        {
            Debug.Assert(false, "Get Mass Data");
            return new MassData();
        }



    }
}