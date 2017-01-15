using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics
{
    #region Types Definition
    public enum BodyType
    {
        Static,
        Kinematic,
        Dynamic
    }
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
        public float friction;
        public float restitution;

        public bool sensor;

        public BodyDefintion(
            BodyType type, IShape shape, float mass, 
            float linearDamping = 0f, float angularDamping = 0f, 
            float gravityScale = 1f, float friction = 0f, float restitution = 1f,
            bool sensor = false)
        {
            this.type = type;
            this.shape = shape;
            this.mass = mass;
            this.linearDamping = linearDamping;
            this.angularDamping = angularDamping;
            this.gravityScale = gravityScale;
            this.friction = friction;
            this.restitution = restitution;
            this.sensor = sensor;
        }
    }
    #endregion

    #region Interface
    public interface IBody
    {
        #region Events
        event IContactHandlerDelegate ContactStartForeseen;
        event IContactHandlerDelegate ContactEndForeseen;
        event IContactHandlerDelegate ContactStarted;
        event IContactHandlerDelegate ContactEnded;
        #endregion

        #region Nature
        uint id { get; }
        BodyType type { get; }
        IShape shape { get; }
        float mass { get; }
        #endregion

        #region Behavior
        float linearDamping { get; }
        float angularDamping { get; }
        float gravityScale { get; }
        float friction { get; }
        float restitution { get; }

        bool sensor { get; }

        Vector2 position { get; }
        float angle { get; }
        Vector2 linearVelocity { get; }
        float angularVelocity { get; }
        Vector2 force { get; }
        float torque { get; }
        bool enduringContact { get; }

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

        #region Track
        IMomentum MomentumForTick(uint tick);
        IEnumerable<IMomentum> MomentumIterator(uint startingTick = 0, uint endingTick = 0);
        bool CrispAtTick(
            uint tick, 
            Vector2 position, float angle,
            float maxDivergencePerTick = 0.25f);
        #endregion
    }
    #endregion
}