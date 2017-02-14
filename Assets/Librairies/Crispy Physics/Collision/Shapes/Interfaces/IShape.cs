using UnityEngine;
namespace CrispyPhysics
{
    using Internal;
    public struct MassData
    {
        public readonly float mass;
        public readonly Vector2 center;
        public readonly float rotationalGravity;

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
        ShapeType type { get; }
        float radius { get; }

        bool TestPoint(Transformation transform, Vector2 point);

        bool RayCast(
            ref RayCastOuput output, RayCastInput input,
            Transformation transform);

        AABB computeAABB(Transformation transform);

        MassData computeMassData(float mass);

    }
}