using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics
{
    #region Types Definition
    public enum BodyType
    {
        DynamicBody
    }
    #endregion

    #region Events Definition
    public delegate void BodyEventHandler(IBody body, EventArgs args);
    #endregion

    #region Body Definition
    public struct BodyDefintion
    {
        public BodyType type;
        public IShape shape;
        public float mass;

        public float linearDamping;
        public float angularDamping;
        public float gravityScale;

        public BodyDefintion(
            BodyType type, IShape shape, float mass, 
            float linearDamping = 0.2f, float angularDamping = 0.2f, 
            float gravityScale = 1f)
        {
            this.type = type;
            this.shape = shape;
            this.mass = mass;
            this.linearDamping = linearDamping;
            this.angularDamping = angularDamping;
            this.gravityScale = gravityScale;
        }
    }
    #endregion

    #region Interface
    public interface IBody
    {
        #region Nature
        BodyType type { get; }
        IShape shape { get; }
        float mass { get; }
        #endregion

        #region Behavior
        float linearDamping { get; }
        float angularDamping { get; }
        float gravityScale { get; }

        Vector2 position { get; }
        float angle { get; }
        Vector2 linearVelocity { get; }
        float angularVelocity { get; }
        Vector2 force { get; }
        float torque { get; }

        List<IContact> contacts { get; }

        void ChangeImpulse(Vector2 force, float torque);
        void ChangeVelocity(Vector2 linearVelocity, float angularVelocity);
        void ChangeSituation(Vector2 position, float angle);

        void ApplyForce(Vector2 force, Vector2 point);
        void ApplyForceToCenter(Vector2 force);
        void ApplyTorque(float torque);
        void ApplyLinearImpulse(Vector2 impulse, Vector2 point);
        void ApplyLinearImpulseToCenter(Vector2 impulse);
        void ApplyAngularImpulse(float impulse);
        #endregion
    }
    #endregion
}