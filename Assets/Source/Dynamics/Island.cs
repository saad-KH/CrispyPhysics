using UnityEngine;
using System;
using System.Collections.Generic;


namespace CrispyPhysics.Internal
{
    public class Island
    {
        private Body[] bodies;
        private Contact[] contacts;

        private Position[] positions;
        private Velocity[] velocities;

        private uint bodyCount;
        private uint contactCount;

        private readonly uint bodyCapacity;
        private readonly uint contactCapacity;

        public Island(uint bodyCapacity = 1, uint contactCapacity = 1)
        {
            if (bodyCapacity == 0)
                throw new ArgumentOutOfRangeException("bodyCapacity should be strictly greater than 0");
            if (contactCapacity == 0)
                throw new ArgumentOutOfRangeException("contactCapacity should be strictly greater than 0");

            this.bodyCapacity = bodyCapacity;
            this.contactCapacity = contactCapacity;

            bodyCount = 0;
            contactCount = 0;

            bodies = new Body[this.bodyCapacity];
            contacts = new Contact[this.contactCapacity];

            positions = new Position[this.bodyCapacity];
            velocities = new Velocity[this.bodyCapacity];
        }

        public void Add(Body body)
        {
            if(bodyCount >= bodyCapacity)
                throw new InvalidOperationException("bodyCapacity is full");
            body.islandIndex = bodyCount;
            bodies[bodyCount] = body;
            bodyCount++;
        }

        public void Add(Contact contact)
        {
            if (contactCount >= contactCapacity)
                throw new InvalidOperationException("contactCapacity is full");
            contacts[contactCount++] = contact;
        }

        public IEnumerable<Body> BodyIterator(uint start = 0, uint end = 0)
        {
            if(end == 0)
                end = bodyCount;
            for (uint i=start; i < end; i++)
                yield return bodies[i];
        }

        public IEnumerable<IContact> ContactIterator(uint start = 0, uint end = 0)
        {
            if(end == 0)
                end = bodyCount;
            for (uint i=start; i < end; i++)
                yield return contacts[i];
        }

        public void Solve(TimeStep step)
        {
            float dt = step.dt;

            for(uint i=0; i < bodyCount; i++)
            {
                Body body = bodies[i];
                Momentum momentum = body.futur;
                
                Vector2 linearVelocity = momentum.linearVelocity;
                float angularVelocity = momentum.angularVelocity;

                if (body.type == BodyType.Dynamic)
                {
                    linearVelocity += 
                            dt 
                        *   (body.gravityScale * step.gravity + body.invMass * momentum.force);

                    angularVelocity += 
                            dt 
                        *   body.invMass * momentum.torque;

                    linearVelocity *= 1f / (1f + dt * body.linearDamping);
                    angularVelocity *= 1f / (1f + dt * body.angularDamping);
                }

                positions[i] = new Position(momentum.position, momentum.angle);
                velocities[i] = new Velocity(linearVelocity, angularVelocity);
            }

            //Debug.Assert(false, "Solve Contacts");
            ContactSolver contactSolver = null;
            if(contactCount > 0)
            {
                ContactSolverDefinition contactSolverDef = new ContactSolverDefinition(
                    step, contactCount, contacts, positions, velocities);
                contactSolver = new ContactSolver(contactSolverDef);
                contactSolver.InitializeVelocityConstraints();

                for (uint i = 0; i < step.velocityIterations; i++)
                    contactSolver.SolveVelocityConstraints();
            }
            
            for (uint i = 0; i < bodyCount; i++)
            {
                Vector2 linearVelocity = velocities[i].linearVelocity;
                float angularVelocity = velocities[i].angularVelocity;

                Vector2 translation = dt * linearVelocity;
                if(Vector2.Dot(translation, translation) > step.maxTranslationSpeed * step.maxTranslationSpeed)
                    linearVelocity *= step.maxTranslationSpeed / translation.magnitude;

                float rotation = dt * angularVelocity;
                if (rotation * rotation > step.maxRotationSpeed * step.maxRotationSpeed)
                    angularVelocity *= step.maxRotationSpeed / Mathf.Abs(rotation);

                positions[i] = new Position(
                    positions[i].center + dt * linearVelocity,
                    positions[i].angle + dt * angularVelocity);

                velocities[i] = new Velocity(linearVelocity, angularVelocity);
            }

            if(contactSolver != null)
                for(uint i=0; i<step.positionIterations; i++)
                    if(contactSolver.SolvePositionConstraints())
                        break;

            for (uint i = 0; i < bodyCount; ++i)
            {
                Momentum momentum = bodies[i].futur;
                momentum.ChangeSituation(positions[i].center, positions[i].angle);
                momentum.ChangeVelocity(velocities[i].linearVelocity, velocities[i].angularVelocity);
            }
        }

        public void Clear()
        {
            bodyCount = 0;
            contactCount = 0;

            bodies = new Body[bodyCapacity];
            contacts = new Contact[contactCapacity];

            positions = new Position[bodyCapacity];
            velocities = new Velocity[contactCapacity];

        }

    }
}

