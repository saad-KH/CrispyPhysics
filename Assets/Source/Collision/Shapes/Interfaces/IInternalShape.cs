using UnityEngine;
namespace CrispyPhysics.Internal
{
    public interface IInternalShape : IShape
    {
        float radius { get; }

        bool TestPoint(Transformation transform, Vector2 point);

        bool RayCast(
            ref RayCastOuput output, RayCastInput input,
            Transformation transform);

        AABB computeAABB(Transformation transform);

        MassData computeMassData(float mass);
    }
}