using UnityEngine;

namespace CrispyPhysics
{
    public interface IContactMomentum
    {
        uint tick { get; }
        float tangentSpeed { get; }
        bool isTouching { get; }
        Vector2 point { get; }
        Vector2 normal { get; }
    }
}