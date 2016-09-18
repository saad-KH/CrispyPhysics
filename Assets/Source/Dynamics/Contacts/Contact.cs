using System;
using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region Events Definition
    public delegate void ContactEventHandler(Contact contact, EventArgs args);
    #endregion

    public class Contact
    {
        public event ContactEventHandler BeginContact;
        public event ContactEventHandler EndContact;
        public event ContactEventHandler Presolve;
        public event ContactEventHandler PostSolve;

        public Manifold manifold { get; private set; }
        public IInternalBody bodyA { get; private set; }
        public IInternalBody bodyB { get; private set; }
        public float friction { get; private set; }
        public float restitution { get; private set; }
        public float tangentSpeed { get; private set; }

        [Flags]
        protected enum OperationFlag
        {
            IslandFlag = 0x0001,
            TouchingFlag = 0x0002
        }

        protected OperationFlag opFlags;

        public Contact(IInternalBody bodyA, IInternalBody bodyB)
        {
            this.bodyA = bodyA;
            this.bodyB = bodyB;
        }

        public bool isTouching()
        {
            return (opFlags & OperationFlag.TouchingFlag) == OperationFlag.TouchingFlag;
        }

        public WorldManifold GetWorldManifold()
        {
            Debug.Assert(bodyA.shape != null);
            Debug.Assert(bodyB.shape != null);

            return new WorldManifold(
                manifold,
                bodyA.transform, bodyA.shape.radius,
                bodyB.transform, bodyB.shape.radius);
        }

        public static float MixFriction(float frictionA, float frictionB)
        {
            return Mathf.Sqrt(frictionA * frictionB);
        }

        public static float MixRestitution(float restitutionA, float restitutionB)
        {
            return restitutionA > restitutionB ? restitutionA : restitutionB;
        }
    }
}