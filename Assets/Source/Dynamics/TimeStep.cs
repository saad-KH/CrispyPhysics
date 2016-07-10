using UnityEngine;

namespace CrispyPhysics
{
    public struct TimeStep
    {
        public float dt;
        public float invDt;
        public float dtRatio;
        public int velocityIterations;
        public int positionIterations;
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