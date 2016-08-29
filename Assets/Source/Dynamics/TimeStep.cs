using UnityEngine;

namespace CrispyPhysics.Internal
{
    public struct TimeStep
    {
        public float dt;
        public float invDt;
        public float dtRatio;
        public Vector2 gravity;
        public int velocityIterations;
        public int positionIterations;
        public float maxTranslationSpeed;
        public float maxRotationSpeed;

        public TimeStep(
            float dt, float invDt, float dtRatio,
            Vector2 gravity, int velocityIterations, int positionIterations,
            float maxTranslationSpeed, float maxRotationSpeed)
        {
            this.dt = dt;
            this.invDt = invDt;
            this.dtRatio = dtRatio;
            this.gravity = gravity;
            this.velocityIterations = velocityIterations;
            this.positionIterations = positionIterations;
            this.maxTranslationSpeed = maxTranslationSpeed;
            this.maxRotationSpeed = maxRotationSpeed;
        }

    }

    public struct Position
    {
        public Vector2 center;
        public float angle;
    }

    public struct Velocity
    {
        public Vector2 linearVelocity;
        public float angularVelocity; 
    }

    public struct SolverData
    {
        public TimeStep step;
        public Position[] positions;
        public Velocity[] velocities;
    }
}