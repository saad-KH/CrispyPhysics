using UnityEngine;

namespace CrispyPhysics.Internal
{
    public struct TimeStep
    {
        public readonly float dt;
        public readonly float invDt;
        public readonly float dtRatio;
        public readonly Vector2 gravity;
        public readonly uint velocityIterations;
        public readonly uint positionIterations;
        public readonly float maxTranslationSpeed;
        public readonly float maxRotationSpeed;

        public TimeStep(
            float dt, float invDt, float dtRatio,
            Vector2 gravity, uint velocityIterations, uint positionIterations,
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
        public readonly Vector2 center;
        public readonly float angle;

        public Position(Vector2 center, float angle)
        {
            this.center = center;
            this.angle = angle;
        }
    }

    public struct Velocity
    {
        public readonly Vector2 linearVelocity;
        public readonly float angularVelocity;

        public Velocity(Vector2 linearVelocity, float angularVelocity)
        {
            this.linearVelocity = linearVelocity;
            this.angularVelocity = angularVelocity;
        }
    }

    public struct SolverData
    {
        public readonly TimeStep step;
        public readonly Position[] positions;
        public readonly Velocity[] velocities;
    }
}