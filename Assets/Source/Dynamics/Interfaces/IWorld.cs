using UnityEngine;

namespace CrispyPhysics
{
    public interface IWorld
    {
        float crispSize { get; set; }
        float tick { get; }
        float actionTick { get; set; }
        float tickRatio { get; }

        void AddBody(IBody body);

        void RemoveBody(IBody body);

        void SetAllowSleeping(bool flag);

        void ClearForces();

        bool IsLocked();

        Vector2 GetGravity();

        void SetAutoClearForces(bool flag);

        bool GetAutoClearForces();

        bool IsCrisped();

        void NotifyShapeAdded();

        void Step(float dt, int velocityIterations, int positionIterations);
    }
}