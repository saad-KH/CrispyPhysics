using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics
{
    public enum BodyType
    {
        DynamicBody
    }

    public interface IBody
    {
        BodyType type { get; }

        IShape shape { get; }

        Vector2 linearVelocity { get; }
        float angularVelocity { get; }
        Vector2 force { get; }
        float torque { get; }

        float mass { get; }
        float invMass { get; }

        float linearDamping { get; set; }
        float angularDamping { get; set; }
        float gravityScale { get; set; }

        float sleepTime { get; set; }

        IWorld world { get; }
        List<IContact> contacts { get; }

        Vector2 position { get; }
        float angle { get; }

        void SetMass(float mass);

        void DefineShape(IShape shape);

        bool ShouldCollide(IBody other);

        void CalculateInertia();

        float GetInertia();

        void SetActive(bool flag);

        bool IsActive();

        void SetAwake(bool flag);

        bool IsAwake();

        void SetSleepingAllowed(bool flag);

        bool IsSleepingAllowed();

        void SetFixedRotation(bool flag);

        bool IsFixedRotation();

        void SetIslandBound(bool flag);

        bool IsIslandBound();

        void ApplyForce(Vector2 force, Vector2 point, bool wake);

        void ApplyForceToCenter(Vector2 force, bool wake);

        void ApplyTorque(float torque, bool wake);

        void ApplyLinearImpulse(Vector2 impulse, Vector2 point, bool wake);

        void ApplyLinearImpulseToCenter(Vector2 impulse, bool wake);

        void ApplyAngularImpulse(float impulse, bool wake);

        void ChangeImpulse(Vector2 force, float torque);

        void ChangeVelocity(Vector2 linearVelocity, float angularVelocity);

        void ChangeSituation(Vector2 position, float angle);

        Vector2 GetWorldPoint(Vector2 localPoint);

        Vector2 GetWorldVector(Vector2 localVector);

        Vector2 GetLocalPoint(Vector2 worldPoint);

        Vector2 GetLocalVector(Vector2 worldVector);
    }
}