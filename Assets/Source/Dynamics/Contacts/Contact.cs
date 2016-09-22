using System;
using System.Collections.Generic;
using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region Events Definition
    public delegate void ContactEventHandler(Contact contact, EventArgs args);
    #endregion

    public class Contact : IContact
    {
        #region Constructors
        public Contact(uint tick, Body bodyA, Body bodyB)
        {
            currentTick = tick;
            this.bodyA = bodyA;
            this.bodyB = bodyB;

            momentums = new List<ContactMomentum>();
            momentums.Add(new ContactMomentum(currentTick, new Manifold()));


            currentIndex = 0;
        }
        #endregion

        #region Events
        public event ContactEventHandler BeginContact;
        public event ContactEventHandler EndContact;
        public event ContactEventHandler Presolve;
        public event ContactEventHandler PostSolve;
        #endregion

        #region Nature
        public Body bodyA { get; private set; }
        public IBody firstBody { get { return bodyA as IBody; } }
        public Body bodyB { get; private set; }
        public IBody secondBody { get { return bodyB as IBody; } }
        public float friction { get; private set; }
        public float restitution { get; private set; }
        public float tangentSpeed { get; private set; }

        public WorldManifold GetWorldManifold()
        {
            Debug.Assert(bodyA.shape != null);
            Debug.Assert(bodyB.shape != null);

            return new WorldManifold(
                current.manifold,
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
        #endregion

        #region Track
        private uint currentTick;
        private int currentIndex;
        public ContactMomentum past { get { return momentums[0]; } }
        public ContactMomentum current { get { return momentums[currentIndex]; } }
        public ContactMomentum futur { get { return momentums[momentums.Count - 1]; } }
        private List<ContactMomentum> momentums;
        public bool islandBound { get; set; }

        #endregion
    }
}