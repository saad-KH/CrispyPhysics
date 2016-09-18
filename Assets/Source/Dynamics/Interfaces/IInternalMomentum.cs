using UnityEngine;

namespace CrispyPhysics.Internal
{
    public interface IInternalMomentum : IMomentum
    {
        Transformation transform { get; }

        void ChangeImpulse(Vector2 force, float torque);
        void ChangeVelocity(Vector2 linearVelocity, float angularVelocity);
        void ChangeSituation(Vector2 position, float angle);

        bool Same(IInternalMomentum other, float tolerance = 0);
    }
}