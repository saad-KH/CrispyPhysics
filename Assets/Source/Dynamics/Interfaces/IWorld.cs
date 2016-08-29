using UnityEngine;

namespace CrispyPhysics
{
    #region World Definition
    public struct WorldDefinition
    {
        public float fixedStep;
        public float crispyStep;
        public float crispySize;
        public Vector2 gravity;
        public int velocityIterations;
        public int positionIterations;
        public float maxTranslationSpeed;
        public float maxRotationSpeed;

        public WorldDefinition(
            float fixedStep, float crispyStep, float crispySize,
            Vector2 gravity, int velocityIterations = 8, int positionIterations = 3,
            float maxTranslationSpeed = 100, float maxRotationSpeed = 360)
        {
            this.fixedStep = fixedStep;
            this.crispyStep = crispyStep;
            this.crispySize = crispySize;
            this.gravity = gravity;
            this.velocityIterations = velocityIterations;
            this.positionIterations = positionIterations;
            this.maxTranslationSpeed = maxTranslationSpeed;
            this.maxRotationSpeed = maxRotationSpeed;
        }
    }
    #endregion

    #region Interface Defintion
    public interface IWorld
    {
        #region Nature
        float fixedStep { get; }
        float crispyStep { get; }
        float crispySize { get; }

        Vector2 gravity { get; }
        int velocityIterations { get; }
        int positionIterations { get; }

        float maxTranslationSpeed { get; }
        float maxRotationSpeed { get; }

        uint tick { get; }
        uint pastTick { get; }
        uint futurTick { get; }
        #endregion

        #region Body Manager
        IBody CreateBody(
            Vector2 position, float angle,
            BodyType type, IShape shape, float mass = 1f,
            float linearDamping = 0f, float angularDamping = 0f,
            float gravityScale = 1f);

        IBody CreateBody(
            Vector2 position, float angle,
            BodyDefintion bodyDef);

        void DestroyBody(IBody body);
        #endregion

        #region Tracker
        void Step(
            uint steps = 1,
            uint foreseeTicks = 0, uint bufferingTicks = 0,
            uint keepTicks = 0);

        void RollBack(uint toTick, uint keepTicks = 0);
        #endregion
    }
    #endregion
}