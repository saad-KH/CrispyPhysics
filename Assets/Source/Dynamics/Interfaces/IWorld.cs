using UnityEngine;

namespace CrispyPhysics
{
    public interface IWorld
    {
        float fixedStep { get; }
        float crispyStep { get; }
        float crispySize { get; }
        float tick { get; }
        float rememberedTime { get; }
        float foreseenTime { get; }


        IBody CreateBody(
            BodyType type, IShape shape,
            Vector2 position, float angle,
            float linearDamping = 0f, float angularDamping = 0f,
            float mass = 1f, float gravityScale = 1f);

        void DestroyBody(IBody body);

        void Step(
            float dt,
            float foresee = 0f, float bufferingCap = 0f,
            float keepPast = 0f);

        void StepBack(float dt, float keepPast = 0f);
    }
}