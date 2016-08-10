using UnityEngine;

namespace CrispyPhysics
{
    public interface IWorld
    {
        float crispSize { get; }
        float tick { get; }
        float actionTick { get; }
        float rememberableTime { get; }
        float foreseeableTime { get; }
        float rememberedTime { get; }
        float foreseenTime { get; }
        float bufferTime { get; set; }

        IBody CreateBody(
            BodyType type, IShape shape,
            Vector2 position, float angle,
            float linearDamping = 0f, float angularDamping = 0f,
            float mass = 1f, float gravityScale = 1f);

        void DestroyBody(IBody body);

        void Step(
            float dt, 
            int velocityIterations, 
            int positionIterations);
    }
}