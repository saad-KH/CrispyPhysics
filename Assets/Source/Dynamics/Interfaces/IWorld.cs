using UnityEngine;

namespace CrispyPhysics
{
    public interface IWorld
    {
        float fixedStep { get; }
        float crispyStep { get; }
        float crispySize { get; }
        uint tick { get; }
        uint pastTick { get; }
        uint futurTick { get; }


        IBody CreateBody(
            BodyType type, IShape shape,
            Vector2 position, float angle,
            float linearDamping = 0f, float angularDamping = 0f,
            float mass = 1f, float gravityScale = 1f);

        void DestroyBody(IBody body);

        void Step(
            uint steps = 1,
            uint foreseeTicks = 0, uint bufferingTicks = 0,
            uint keepTicks = 0);

        void RollBack(uint toPastTick, uint keepTicks = 0);
    }
}