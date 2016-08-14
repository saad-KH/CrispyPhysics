using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics.Internal
{
    public class Body : IInternalBody
    {
        #region Constructor
        public Body(
           BodyType type,
           IShape shape,
           Vector2 position,
           float angle,
           float linearDamping = 0f,
           float angularDamping = 0f,
           float mass = 1f,
           float gravityScale = 1f
           )
        {
            this.type = type;

            SetMass(mass);
            DefineShape(shape);

            this.linearDamping = linearDamping;
            this.angularDamping = angularDamping;
            this.gravityScale = gravityScale;

            momentums = new List<IInternalMomentum>();
            momentums.Add(new Momentum(
                0f,
                Vector2.zero, 0f,
                Vector2.zero, 0f,
                position, angle));

            currentTick = 0f;
            currentIndex = 0;

            contacts = new List<IContact>();
        }
        #endregion

        #region Events
        public event BodyEventHandler FuturCleared;
        #endregion

        #region Nature
        public BodyType type { get; private set; }
        public IShape shape { get; private set; }
        public float mass { get; private set; }
        public float invMass { get; private set; }
        private float rotationalInertia, invRotationalInertia;

        private void SetMass(float mass)
        {
            if (type != BodyType.DynamicBody)
            {
                this.mass = invMass = 0f;
                return;
            }

            this.mass = (mass > Mathf.Epsilon) ? mass : 1f;
            invMass = 1f / this.mass;
        }

        private void DefineShape(IShape shape)
        {
            this.shape = shape;
            //Debug.Assert(false, "Search Contacts");
            //Debug.Assert(false, "Broad Phase");
            if (type == BodyType.DynamicBody)
                CalculateInertia();
        }

        public bool ShouldCollide(IBody other)
        {
            if (type != BodyType.DynamicBody && other.type != BodyType.DynamicBody)
                return false;
            return true;
        }

        private void CalculateInertia()
        {
            rotationalInertia = 0f;
            invRotationalInertia = 0f;

            Debug.Assert(type == BodyType.DynamicBody);
            //Debug.Assert(false, "Find Inertia of body");
        }

        public float GetInertia()
        {
            return rotationalInertia;
        }
        #endregion

        #region Behavior

        public float linearDamping { get; private set; }
        public float angularDamping { get; private set; }
        public float gravityScale { get; private set; }

        public Vector2 position { get { return current.position; } }
        public float angle { get { return current.angle; } }
        public Vector2 linearVelocity { get { return current.linearVelocity; } }
        public float angularVelocity { get { return current.angularVelocity; } }
        public Vector2 force { get { return current.force; } }
        public float torque { get { return current.torque; } }

        public List<IContact> contacts { get; private set; }

        public void ApplyForce(Vector2 force, Vector2 point)
        {
            if (type != BodyType.DynamicBody) return;

            ClearFutur();

            current.ChangeImpulse(
                current.force + force,
                current.torque + Calculus.Cross(point - current.position, force));
        }

        public void ApplyForceToCenter(Vector2 force)
        {
            if (type != BodyType.DynamicBody) return;

            ClearFutur();
            current.ChangeImpulse(current.force + force, current.torque);
        }

        public void ApplyTorque(float torque)
        {
            if (type != BodyType.DynamicBody) return;

            ClearFutur();
            current.ChangeImpulse(current.force, current.torque + torque);
        }

        public void ApplyLinearImpulse(Vector2 impulse, Vector2 point)
        {
            if (type != BodyType.DynamicBody) return;
            ClearFutur();

            current.ChangeVelocity(
                current.linearVelocity + invMass * impulse,
                current.angularVelocity +
                    (invRotationalInertia
                    * Calculus.Cross(point - current.position, impulse)));
        }

        public void ApplyLinearImpulseToCenter(Vector2 impulse)
        {
            if (type != BodyType.DynamicBody) return;

            ClearFutur();

            current.ChangeVelocity(
                current.linearVelocity + invMass * impulse,
                current.angularVelocity);

        }

        public void ApplyAngularImpulse(float impulse)
        {
            if (type != BodyType.DynamicBody) return;

            ClearFutur();

            current.ChangeVelocity(
                current.linearVelocity,
                current.angularVelocity + invRotationalInertia * impulse);
        }
        #endregion

        #region Track
        private float currentTick;
        private int currentIndex;
        public IInternalMomentum current { get { return momentums[currentIndex]; } }
        public IInternalMomentum futur { get { return momentums[momentums.Count - 1]; } }
        private List<IInternalMomentum> momentums;
        public bool islandBound { get; set; }

        public void Step(float dt)
        {
            currentTick += dt;

            //The aim is to find the momentum with the higher tick that is lesser than current tick
            //If while looking a momentum's tick is higher than the previous one and lesser than the current
            //But the momentum is equivalent to the previous one
            //We get rid of the newest and operate with the oldest
            //This is because the new one doesn't provide any change in the timeline
            //And the previous one tell us since when this momentum was effective
            //This one remains effective until a different momentum exists in the timeline


            //If current momentum is equivalent to the previous one, we delete it
            //The current one was just meant to be synced with the old current tick
            //We will create a new current Momentum synced with the new current tick
            if (currentIndex > 0 && momentums[currentIndex].Same(momentums[currentIndex - 1]))
            {
                momentums.RemoveAt(currentIndex);
                currentIndex--;
            }

            bool done = false;
            while (!done && currentIndex < momentums.Count)
                //Reaching Last
                if (currentIndex == momentums.Count - 1)
                    done = true;
                // Next one has a higher tick
                else if (   momentums[currentIndex + 1].tick > currentTick
                        &&  !Calculus.Approximately(momentums[currentIndex + 1].tick, currentTick))
                    done = true;
                //Next one does not change the timeline, we delete it and conserve the current
                else if (momentums[currentIndex].Same(momentums[currentIndex + 1]))
                    momentums.RemoveAt(currentIndex + 1);
                //Next one is a better candidate than the current one
                else currentIndex++;


            //We create a new current momentum that is synced with the current tick
            //It will than hold any new momentum change for this tick
            if (!Calculus.Approximately(momentums[currentIndex].tick, currentTick))
            {
                currentIndex++;
                momentums.Insert(
                    currentIndex,
                    new Momentum(
                        currentTick,
                        momentums[currentIndex - 1]));
            }
        }

        public void StepBack(float dt)
        {
            currentTick -= dt;

            //The aime is to find the momentum with a tick lesser or equal to the current one
            //While looking if the current momentum's tick is greater than the current tick
            //and is equivalent to the previous momentum, we get rid of the current momentum
            //as it becomes useless in the timeline
            bool done = false;
            while (!done && currentIndex >= 0)
                if (currentIndex == 0)
                    done = true;
                else if (   momentums[currentIndex].tick <= currentTick
                        ||  Calculus.Approximately(momentums[currentIndex].tick, currentTick))
                    done = true;
                else
                {
                    if (momentums[currentIndex].Same(momentums[currentIndex - 1]))
                        momentums.RemoveAt(currentIndex);
                    currentIndex--;
                }

            //We create a new current momentum that has the current tick
            //It will than hold any new momentum change for this tick
            if (!Calculus.Approximately(momentums[currentIndex].tick, currentTick))
            {
                currentIndex++;
                momentums.Insert(
                    currentIndex,
                    new Momentum(
                        currentTick,
                        momentums[currentIndex - 1]));
            }
        }

        public void Foresee(float dt)
        {
            IInternalMomentum futurMomentum = new Momentum(
                momentums[momentums.Count - 1].tick + dt,
                momentums[momentums.Count - 1]);

            if (currentIndex != momentums.Count - 1)
                if (momentums[momentums.Count - 1].Same(momentums[momentums.Count - 2]))
                    momentums.RemoveAt(momentums.Count - 1);

            momentums.Add(futurMomentum);
        }

        public bool IsForeseen()
        {
            return currentIndex < (momentums.Count - 1);
        }

        public void keep(float past = -1f, float futur = -1f)
        {
            if (Calculus.Approximately(past, 0f))
                ForgetPast();
            else if (past > 0f)
            {
                float keepTick = currentTick - past;
                int keepIndex = 0;
                //We want to find the momentum with the highest tick that is lesser or equal to the keep tick
                //The aim is to keep it along with any newer momentums
                bool done = false;
                while (!done && currentIndex != keepIndex)
                {
                    //Next tick is wihtin the keeping range, we keep the current one
                    if  (   momentums[keepIndex + 1].tick > keepTick
                        &&  !Calculus.Approximately(momentums[keepIndex + 1].tick, keepTick))
                        done = true;
                    else
                        keepIndex++;
                }

                //We only retains the keeped momentum and the next ones
                //However we raise the keeped momentum's tick to the keep tick if needed
                if  (   momentums[keepIndex].tick < keepTick
                    &&  !Calculus.Approximately(momentums[keepIndex].tick, keepTick))
                {
                    currentIndex++;
                    keepIndex++;
                    momentums.Insert(
                        keepIndex,
                        new Momentum(
                            keepTick,
                            momentums[keepIndex - 1]));

                }
                //We remove the momentums precending the keeped one
                if (keepIndex > 0)
                {
                    currentIndex -= keepIndex;
                    momentums.RemoveRange(0, keepIndex);
                }
            }

            if (Calculus.Approximately(futur, 0f))
                ClearFutur();
            else if (futur > 0f)
            {
                float keepTick = currentTick + futur;
                int keepIndex = momentums.Count - 1;
                while   (   currentIndex != keepIndex
                        &&  (   momentums[keepIndex].tick > keepTick
                            &&  !Calculus.Approximately(momentums[keepIndex].tick, keepTick)))
                    keepIndex--;

                if ((keepIndex + 1) < momentums.Count)
                {
                    momentums.RemoveRange(keepIndex + 1, momentums.Count - (keepIndex + 1));
                    if (FuturCleared != null)
                        FuturCleared(this, EventArgs.Empty);
                }
            }
        }

        public void ForgetPast()
        {
            if (currentIndex > 0)
            {
                momentums.RemoveRange(0, currentIndex);
                currentIndex = 0;
            }
        }

        public void ClearFutur()
        {
            if (currentIndex < momentums.Count)
            {
                momentums.RemoveRange(currentIndex + 1, momentums.Count - (currentIndex + 1));
                if (FuturCleared != null)
                    FuturCleared(this, EventArgs.Empty);
            }
        }

        #endregion
    }
}
