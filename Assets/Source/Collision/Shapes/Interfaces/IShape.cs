using UnityEngine;
namespace CrispyPhysics
{
    public struct MassData
    {
        public float mass;
        public Vector2 center;
        public float rotationalGravity;

        public MassData(float mass, Vector2 center, float rotationalGravity)
        {
            this.mass = mass;
            this.center = center;
            this.rotationalGravity = rotationalGravity;
        }
    }

    public enum ShapeType
    {
        Circle = 0,
        Edge = 1,
        Polygon = 2,
        Count = 3
    }

    public interface IShape
    {
        float radius { get; }
        ShapeType type { get; }
    }
}