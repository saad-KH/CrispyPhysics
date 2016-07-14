using UnityEngine;
namespace CrispyPhysics
{
    public struct MassData
    {
        float mass;
        Vector2 center;
        float rotationalGravity;
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
        
        ShapeType type { get;}
        Collider2D collider { get; }

        int GetChildCount();

        bool TestPoint(Vector2 point);

        bool RayCast();

        MassData GetMassData();
    }
}