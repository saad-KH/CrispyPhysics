using UnityEngine;

namespace CrispyPhysics
{
    public class Island : IIsland
    {
        private IBody[] bodies;
        private IContact[] contacts;

        private Position[] positions;
        private Velocity[] velocities;

        private int bodyCount;
        private int contactCount;

        private int bodyCapacity;
        private int contactCapacity;

        public Island(int bodyCapacity = 1, int contactCapacity = 1)
        {
            Debug.Assert(bodyCapacity >= 0);
            Debug.Assert(contactCapacity >= 0);

            this.bodyCapacity = (bodyCapacity > 0) ? bodyCapacity : 1;
            this.contactCapacity = (contactCapacity > 0) ? contactCapacity: 1;

            bodyCount = 0;
            contactCount = 0;

            bodies = new IBody[this.bodyCapacity];
            contacts = new IContact[this.contactCapacity];

            positions = new Position[this.bodyCapacity];
            velocities = new Velocity[this.contactCapacity];
        }

        public void Add(IBody body)
        {
            Debug.Assert(bodyCount < bodyCapacity);
            bodies[bodyCount++] = body;
        }

        public void Add(IContact contact)
        {
            Debug.Assert(contactCount < contactCapacity);
            contacts[contactCount++] = contact;
        }

        public void Solve(TimeStep step, Vector2 gravity, bool allowSleep)
        {
            float dt = step.dt;

            for(int i=0; i < bodyCount; ++i)
            {
                IBody body = bodies[i];
                Vector2 linearVelocity = body.linearVelocity;
                float angularVelocity = body.angularVelocity;

                positions[i].center = body.position;
                positions[i].angle = body.angle;

                if (body.type == BodyType.DynamicBody)
                {
                    linearVelocity += 
                            dt 
                        *   (body.gravityScale * gravity + body.invMass * body.force);

                    angularVelocity += 
                            dt 
                        *   body.invMass * body.torque;

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

            bool positionSolved = false;
            {
                positionSolved = true;
            }

            //Debug.Assert(false, "Solve Position Constraints");

            for (int i = 0; i < bodyCount; ++i)
            {
                IBody body = bodies[i];
                body.ChangeSituation(positions[i].center, positions[i].angle);
                body.ChangeVelocity(velocities[i].linearVelocity, velocities[i].angularVelocity);
            }

            if (allowSleep)
            {
                float minSleepTime = float.MaxValue;

                float linTolSqr = Physics2D.linearSleepTolerance * Physics2D.linearSleepTolerance;
                float angTolSqr = Physics2D.angularSleepTolerance * Physics2D.angularSleepTolerance;

                foreach (IBody body in bodies)
                {
                    if(
                            body.IsSleepingAllowed() == false
                        ||  (body.angularVelocity * body.angularVelocity > angTolSqr)
                        ||  (Vector2.Dot(body.linearVelocity, body.linearVelocity) > linTolSqr)
                      )
                    {
                        body.sleepTime = 0f;
                        minSleepTime = 0f;
                    }
                    else
                    {
                        body.sleepTime += dt;
                        minSleepTime = Mathf.Min(minSleepTime, body.sleepTime);
                    }
                }

                if (minSleepTime > Physics2D.timeToSleep && positionSolved)
                    foreach (Body body in bodies)
                        body.SetAwake(false);
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

            bodies = new IBody[bodyCapacity];
            contacts = new Contact[contactCapacity];

            positions = new Position[bodyCapacity];
            velocities = new Velocity[contactCapacity];

        }

    }
}

