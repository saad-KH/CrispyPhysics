using System;
using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region Events Definition
    public delegate void ContactEventHandler(Contact contact, EventArgs args);
    #endregion

    public class Contact : IContact
    {
        public event ContactEventHandler BeginContact;
        public event ContactEventHandler EndContact;
        public event ContactEventHandler Presolve;
        public event ContactEventHandler PostSolve;

        public Manifold manifold { get; private set; }
        public Body internalBodyA { get; private set; }
        public IBody bodyA { get { return internalBodyA as IBody; } }
        public Body internalBodyB { get; private set; }
        public IBody bodyB { get { return internalBodyB as IBody; } }
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

        public Contact(Body bodyA, Body bodyB)
        {
            internalBodyA = bodyA;
            internalBodyB = bodyB;
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
                internalBodyA.transform, bodyA.shape.radius,
                internalBodyB.transform, bodyB.shape.radius);
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