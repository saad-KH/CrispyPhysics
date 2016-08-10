using UnityEngine;
using System;

namespace CrispyPhysics.Internal
{
    public class Island
    {
        private IInternalBody[] bodies;
        private IContact[] contacts;

        private Position[] positions;
        private Velocity[] velocities;

        private int bodyCount;
        private int contactCount;

        private int bodyCapacity;
        private int contactCapacity;

        public Island(int bodyCapacity = 1, int contactCapacity = 1)
        {
            if (bodyCapacity <= 0)
                throw new ArgumentException("bodyCapacity should be strictly greater than 0");
            if (contactCapacity <= 0)
                throw new ArgumentException("contactCapacity should be strictly greater than 0");

            this.bodyCapacity = bodyCapacity;
            this.contactCapacity = contactCapacity;

            bodyCount = 0;
            contactCount = 0;

            bodies = new IInternalBody[this.bodyCapacity];
            contacts = new IContact[this.contactCapacity];

            positions = new Position[this.bodyCapacity];
            velocities = new Velocity[this.contactCapacity];
        }

        public void Add(IInternalBody body)
        {
            if(bodyCount >= bodyCapacity)
                throw new InvalidOperationException("bodyCapacity is full");
            bodies[bodyCount++] = body;
        }

        public void Add(IContact contact)
        {
            if (contactCount >= contactCapacity)
                throw new InvalidOperationException("contactCapacity is full");
            contacts[contactCount++] = contact;
        }

        public void Solve(TimeStep step, Vector2 gravity)
        {
            float dt = step.dt;

            for(int i=0; i < bodyCount; ++i)
            {
                IInternalBody body = bodies[i];
                body.Foresee(dt);
                IInternalMomentum momentum = body.futur;
                
                Vector2 linearVelocity = momentum.linearVelocity;
                float angularVelocity = momentum.angularVelocity;

                positions[i].center = momentum.position;
                positions[i].angle = momentum.angle;

                if (body.type == BodyType.DynamicBody)
                {
                    linearVelocity += 
                            dt 
                        *   (body.gravityScale * gravity + body.invMass * momentum.force);

                    angularVelocity += 
                            dt 
                        *   body.invMass * momentum.torque;

                    linearVelocity *= 1f / (1f + dt * body.linearDamping);
                    angularVelocity *= 1f / (1f + dt * body.angularDamping);
                }
                
                velocities[i].linearVelocity = linearVelocity;
                velocities[i].angularVelocity = angularVelocity;
            }

            //Debug.Assert(false, "Solve Contacts");

            for (int i = 0; i < bodyCount; ++i)
            {
                Vector2 linearVelocity = velocities[i].linearVelocity;
                float angularVelocity = velocities[i].angularVelocity;

                Vector2 translation = dt * linearVelocity;
                if(Vector2.Dot(translation, translation) > Physics2D.maxTranslationSpeed * Physics2D.maxTranslationSpeed)
                    linearVelocity *= Physics2D.maxTranslationSpeed / translation.magnitude;

                float rotation = dt * angularVelocity;
                if (rotation * rotation > Physics2D.maxRotationSpeed * Physics2D.maxRotationSpeed)
                    angularVelocity *= Physics2D.maxRotationSpeed / Mathf.Abs(rotation);

                positions[i].center += dt * linearVelocity;
                positions[i].angle += dt * angularVelocity;
                velocities[i].linearVelocity = linearVelocity;
                velocities[i].angularVelocity = angularVelocity;
            }

            /*bool positionSolved = false;
            {
                positionSolved = true;
            }*/

            //Debug.Assert(false, "Solve Position Constraints");

            for (int i = 0; i < bodyCount; ++i)
            {
                IInternalMomentum momentum = bodies[i].futur;
                momentum.ChangeSituation(positions[i].center, positions[i].angle);
                momentum.ChangeVelocity(velocities[i].linearVelocity, velocities[i].angularVelocity);
            }
        }

        public void SolveTOI(TimeStep subStep, int toiIndexA, int toiIndexB)
        {
            Debug.Assert(false, "Solve TOI for Island");
        }

        public void Report()
        {
            Debug.Assert(false, "Report");
        }

        public void Clear()
        {
            bodyCount = 0;
            contactCount = 0;

            bodies = new IInternalBody[bodyCapacity];
            contacts = new IContact[contactCapacity];

            positions = new Position[bodyCapacity];
            velocities = new Velocity[contactCapacity];

        }

    }
}

