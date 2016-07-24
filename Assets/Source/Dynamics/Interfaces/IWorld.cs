using UnityEngine;

namespace CrispyPhysics
{
    public interface IWorld
    {
        float crispSize { get; set; }
        float tick { get; }
        float actionTick { get; set; }
        float tickRatio { get; }

        void Add(IBody body);

        void Remove(IBody body);

        void SetAllowSleeping(bool flag);

        void ClearForces();

        bool IsLocked();

        Vector2 GetGravity();

        void SetAutoClearForces(bool flag);

        bool GetAutoClearForces();

        bool IsCrisped();

        void Step(float dt, int velocityIterations, int positionIterations);
    }
}