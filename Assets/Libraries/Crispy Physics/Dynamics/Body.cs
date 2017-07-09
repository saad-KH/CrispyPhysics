using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics.Internal
{
    #region Delegate Definition
    public delegate void BodyHandlerDelegate(Body body, uint fromTick);
    public delegate IEnumerable<Body> BodyIteratorDelegate(uint start = 0, uint end = 0);
    #endregion

    public class Body : IBody, IEquatable<IBody>
    {
        #region Constructors
        public Body(
           uint id, uint tick,
           Vector2 position, float angle,
           BodyType type, IShape shape, float mass = 1f,
           float linearDamping = 0f, float angularDamping = 0f,
           float gravityScale = 1f, float friction = 0f, float restitution = 1f,
           bool sensor = false
           )
        {
            this.id = id;
            currentTick = tick;
            this.type = type;

            SetMass(mass);
            DefineShape(shape);

            this.linearDamping = linearDamping;
            this.angularDamping = angularDamping;
            this.gravityScale = gravityScale;
            this.friction = friction;
            this.restitution = restitution;

            this.sensor = sensor;

            momentums = new List<Momentum>();
            momentums.Add(new Momentum(
                currentTick,
                0f,
                Vector2.zero, 0f,
                Vector2.zero, 0f,
                position, angle, false));


            currentIndex = 0;
            islandBound = false;
            islandIndex = 0;
        }

        public Body(uint id, uint currentTick, Vector2 position, float angle, BodyDefintion bodyDef) : this (
            id, currentTick,
            position, angle,
            bodyDef.type, bodyDef.shape, bodyDef.mass,
            bodyDef.linearDamping, bodyDef.angularDamping,
            bodyDef.gravityScale, bodyDef.friction, bodyDef.restitution,
            bodyDef.sensor)
        {}
        #endregion

        #region Events
        public event BodyHandlerDelegate ExternalChange;
        public event IContactHandlerDelegate ContactStartForeseen;
        public event IContactHandlerDelegate ContactEndForeseen;
        public event IContactHandlerDelegate ContactStarted;
        public event IContactHandlerDelegate ContactEnded;

        public void NotifyContactStartForeseen(IContact contact, IContactMomentum momentum)
        {
            if (ContactStartForeseen != null)
                ContactStartForeseen(contact, momentum);
        }

        public void NotifyContactEndForeseen(IContact contact, IContactMomentum momentum)
        {
            if (ContactEndForeseen != null)
                ContactEndForeseen(contact, momentum);
        }

        public void NotifyContactStarted(IContact contact, IContactMomentum momentum)
        {
            if (ContactStarted != null)
                ContactStarted(contact, momentum);
        }

        public void NotifyContactEnded(IContact contact, IContactMomentum momentum)
        {
            if (ContactEnded != null)
                ContactEnded(contact, momentum);
        }
        #endregion

        #region Nature
        public uint id { get; private set; }
        public BodyType type { get; private set; }
        public IShape shape { get; private set; }
        public float mass { get; private set; }
        public float invMass { get; private set; }
        public Vector2 center { get; private set; }
        public float rotationalInertia { get; private set; }
        public float invRotationalInertia { get; private set; }

        public override int GetHashCode()
        {
            return id.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            return Equals(obj as Body);
        }

        public bool Equals(IBody obj)
        {
            return obj != null && obj.id == this.id;
        }

        private void SetMass(float mass)
        {
            if (type == BodyType.Static || type == BodyType.Kinematic)
                this.mass = invMass = 0f;
            else
            {
                this.mass = (mass > Mathf.Epsilon) ? mass : 1f;
                invMass = 1f / this.mass;
            }
            CalculateInertia();
        }

        private void DefineShape(IShape shape)
        {
            this.shape = shape;
            if (type == BodyType.Dynamic)
                CalculateInertia();
        }

        public bool ShouldCollide(IBody other)
        {
            if (type != BodyType.Dynamic && other.type != BodyType.Dynamic)
                return false;
            return true;
        }

        private void CalculateInertia()
        {
            rotationalInertia = 0f;
            invRotationalInertia = 0f;

            if (type != BodyType.Dynamic)
                return;

            if (shape != null)
            {
                MassData massData = shape.computeMassData(mass);
                center = massData.center;
                rotationalInertia =
                        massData.rotationalGravity
                    -   (mass * Calculus.Dot(massData.center, massData.center));
                invRotationalInertia = 1f / rotationalInertia;
            }
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
        public float friction { get; private set; }
        public float restitution { get; private set; }

        public bool sensor { get; private set; }

        public Vector2 position { get { return current.position; } }
        public float angle { get { return current.angle; } }
        public Vector2 linearVelocity { get { return current.linearVelocity; } }
        public float angularVelocity { get { return current.angularVelocity; } }
        public Vector2 force { get { return current.force; } }
        public float torque { get { return current.torque; } }
        public bool enduringContact {  get { return current.enduringContact; } }
        public Transformation transform { get { return internalCurrent.transform; } }

        public void ChangeImpulse(Vector2 force, float torque)
        {
            if (type == BodyType.Static) return;
            
            momentums[currentIndex].ChangeImpulse(force, torque);

            if (ExternalChange != null)
                ExternalChange(this, currentTick);
        }

        public void ChangeVelocity(Vector2 linearVelocity, float angularVelocity)
        {
            if (type == BodyType.Static) return;

            momentums[currentIndex].ChangeVelocity(linearVelocity, angularVelocity);

            if (ExternalChange != null)
                ExternalChange(this, currentTick);
        }

        public void ChangeSituation(Vector2 position, float angle)
        {
            momentums[currentIndex].ChangeSituation(position, angle);

            if (ExternalChange != null)
                ExternalChange(this, currentTick);
        }

        public void ApplyForce(Vector2 force, Vector2 point)
        {
            if (type != BodyType.Dynamic) return;

            momentums[currentIndex].ChangeImpulse(
                current.force + force,
                current.torque + Calculus.Cross(point - current.position, force));

            if (ExternalChange != null)
                ExternalChange(this, currentTick);
        }

        public void ApplyForceToCenter(Vector2 force)
        {
            if (type != BodyType.Dynamic) return;

            momentums[currentIndex].ChangeImpulse(current.force + force, current.torque);

            if (ExternalChange != null)
                ExternalChange(this, currentTick);
        }

        public void ApplyTorque(float torque)
        {
            if (type != BodyType.Dynamic) return;

            momentums[currentIndex].ChangeImpulse(current.force, current.torque + torque);

            if (ExternalChange != null)
                ExternalChange(this, currentTick);
        }

        public void ApplyLinearImpulse(Vector2 impulse, Vector2 point)
        {
            if (type != BodyType.Dynamic) return;

            momentums[currentIndex].ChangeVelocity(
                current.linearVelocity + invMass * impulse,
                current.angularVelocity +
                    (invRotationalInertia
                    * Calculus.Cross(point - current.position, impulse)));

            if (ExternalChange != null)
                ExternalChange(this, currentTick);
        }

        public void ApplyLinearImpulseToCenter(Vector2 impulse)
        {
            if (type != BodyType.Dynamic) return;

            momentums[currentIndex].ChangeVelocity(
                current.linearVelocity + invMass * impulse,
                current.angularVelocity);

            if (ExternalChange != null)
                ExternalChange(this, currentTick);

        }

        public void ApplyAngularImpulse(float impulse)
        {
            if (type != BodyType.Dynamic) return;

            momentums[currentIndex].ChangeVelocity(
                current.linearVelocity,
                current.angularVelocity + invRotationalInertia * impulse);

            if (ExternalChange != null)
                ExternalChange(this, currentTick);
        }
        #endregion

        #region Track
        private uint currentTick;
        private int currentIndex;
        public Momentum internalPast { get { return momentums[0]; } }
        public Momentum internalCurrent { get { return momentums[currentIndex]; } }
        public Momentum internalFutur { get { return momentums[momentums.Count - 1]; } }
        public IMomentum past { get { return internalPast; } }
        public IMomentum  current{ get { return internalCurrent; } }
        public IMomentum futur { get { return internalFutur; } }
        private List<Momentum> momentums;
        public bool islandBound { get; set; }
        public uint islandIndex { get; set; }

        public void Step(uint steps = 1)
        {
            currentTick += steps;

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
                else if (momentums[currentIndex + 1].tick > currentTick)
                    done = true;
                //Next one does not change the timeline, we delete it and conserve the current
                else if (momentums[currentIndex].Same(momentums[currentIndex + 1]))
                    momentums.RemoveAt(currentIndex + 1);
                //Next one is a better candidate than the current one
                else currentIndex++;


            //We create a new current momentum that is synced with the current tick
            //It will than hold any new momentum change for this tick
            if (momentums[currentIndex].tick != currentTick)
            {
                int sourceIndex = currentIndex;
                if(momentums[currentIndex].tick < currentTick)
                    currentIndex++;
                momentums.Insert(
                    currentIndex,
                    new Momentum(
                        currentTick,
                        momentums[sourceIndex]));
            }
        }

        public void RollBack(uint toTick)
        {
            currentTick = toTick;

            //The aime is to find the momentum with a tick lesser or equal to the current one
            //While looking if the current momentum's tick is greater than the current tick
            //and is equivalent to the previous momentum, we get rid of the current momentum
            //as it becomes useless in the timeline
            bool done = false;
            while (!done && currentIndex >= 0)
                if (currentIndex == 0)
                    done = true;
                else if (momentums[currentIndex].tick <= currentTick)
                    done = true;
                else
                {
                    if (momentums[currentIndex].Same(momentums[currentIndex - 1]))
                        momentums.RemoveAt(currentIndex);
                    currentIndex--;
                }

            //We create a new current momentum that has the current tick
            //It will than hold any new momentum change for this tick
            if (momentums[currentIndex].tick != currentTick)
            {
                int sourceIndex = currentIndex;
                if (momentums[currentIndex].tick < currentTick)
                    currentIndex++;
                momentums.Insert(
                    currentIndex,
                    new Momentum(
                        currentTick,
                        momentums[sourceIndex]));
            }
        }

        public void Foresee(uint steps = 1)
        {
            Momentum futurMomentum = new Momentum(
                momentums[momentums.Count - 1].tick + steps,
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

        public void ForgetPast(uint fromTick)
        {
            if (fromTick >= currentTick)
            {
                if (currentIndex > 0)
                {
                    momentums.RemoveRange(0, currentIndex);
                    currentIndex = 0;
                }
            }
            else
            {
                int keepIndex = 0;
                //We want to find the momentum with the highest tick that is lesser or equal to the keep tick
                //The aim is to keep it along with any newer momentums
                bool done = false;
                while (!done && currentIndex != keepIndex)
                {
                    //Next tick is wihtin the keeping range, we keep the current one
                    if (momentums[keepIndex + 1].tick > fromTick)
                        done = true;
                    else
                        keepIndex++;
                }

                //We only retains the keeped momentum and the next ones
                //However we raise the keeped momentum's tick to the keep tick if needed
                if (momentums[keepIndex].tick < fromTick)
                {
                    currentIndex++;
                    keepIndex++;
                    momentums.Insert(
                        keepIndex,
                        new Momentum(
                            fromTick,
                            momentums[keepIndex - 1]));

                }
                //We remove the momentums precending the keeped one
                if (keepIndex > 0)
                {
                    currentIndex -= keepIndex;
                    momentums.RemoveRange(0, keepIndex);
                }
            }
        }

        public void ClearFutur()
        {
            ClearFutur(currentTick + 1);
        }

        public void ClearFutur(uint fromTick)
        {
            if(fromTick <= currentTick)
                throw new ArgumentOutOfRangeException("Tick to be cleared should be above the current tick");

            int indexForTick = IndexForTick(fromTick);
            if (indexForTick == currentIndex)
                indexForTick++;

            if (indexForTick >= 0 && indexForTick < momentums.Count)
                momentums.RemoveRange(indexForTick, momentums.Count - indexForTick);
        }

        public int IndexForTick(uint tick)
        {
            for (int i = (tick >= currentTick) ? currentIndex : 0; i < momentums.Count; i++)
                if (momentums[i].tick == tick) return i;
                else if (momentums[i].tick > tick) return i > 0 ? i - 1 : i;

            return -1;
        }

        public IMomentum MomentumForTick(uint tick)
        {
            for (int i = (tick >= currentTick) ? currentIndex : 0; i < momentums.Count; i++)
                if (momentums[i].tick == tick) return momentums[i];
                else if (momentums[i].tick > tick)
                {
                    if (i > 0)
                        return momentums[i - 1];
                    else
                        return null;
                }


            return null;
        }

        public IEnumerable<IMomentum> MomentumIterator(uint startingTick = 0, uint endingTick = 0)
        {
            if (endingTick == 0)
                endingTick = futur.tick;

            if(startingTick <= endingTick)
            {
                for (int i = (startingTick >= currentTick) ? currentIndex : 0; i < momentums.Count; i++)
                    if (momentums[i].tick >= startingTick && momentums[i].tick <= endingTick)
                        yield return momentums[i];
                    else if (momentums[i].tick > endingTick) break;
            }
            else
            {
                for (int i = (startingTick >= currentTick) ? momentums.Count - 1 : currentIndex; i >= 0; i--)
                    if (momentums[i].tick <= startingTick && momentums[i].tick >= endingTick)
                        yield return momentums[i];
                    else if (momentums[i].tick < endingTick) break;
            }
                
        }

        public bool CrispAtTick(
            uint crispTick, 
            Vector2 position, float angle,
            float maxDivergencePerTick = 0.25f)
        {
            if (crispTick <= currentTick)
                throw new ArgumentOutOfRangeException("Tick to be converged should be above the current tick");

            int crispIndex = IndexForTick(crispTick);
            if (momentums[crispIndex].enduringContact)
                return false;

            Momentum crispMomentum = null;
            if (momentums[crispIndex].tick == crispTick)
                crispMomentum = momentums[crispIndex];
            else
            {
                crispMomentum = new Momentum(crispTick, momentums[crispIndex]);
                momentums.Insert(++crispIndex, crispMomentum);
            }

            Vector2 divergence = position - crispMomentum.position;

            int lockedIndex = crispIndex - 1;
            bool done = false;
            while (!done && lockedIndex > 0)
                if (    momentums[lockedIndex].tick <= currentTick
                    ||  momentums[lockedIndex].enduringContact)
                    done = true;
                else
                    lockedIndex--;

            uint startingTick = momentums[lockedIndex].tick + 1;
            uint convergenceTickCount = crispMomentum.tick - startingTick + 1;

            if (divergence.magnitude / convergenceTickCount > maxDivergencePerTick)
                return false;

            Vector2 convergencePerTick = divergence / convergenceTickCount;
            Vector2 convergence = Vector2.zero;
            float angleConvergencePerTick = (angle - crispMomentum.angle) / convergenceTickCount;
            float angleConvergence = 0f;

            float tickDt = 0f;

            uint tick = startingTick;
            int index = lockedIndex;
            while(tick <= crispTick)
            {
                while ( index + 1 < momentums.Count
                     && momentums[index + 1].tick <= tick)
                    index++;

                Momentum momentum = null;
                if (momentums[index].tick == tick)
                {
                    momentum = momentums[index];
                    momentum.ChangeSituation(
                        momentum.position + convergence,
                        momentum.angle + angleConvergence);

                    if (!Calculus.Approximately(momentum.tickDt, 0f))
                        momentum.ChangeVelocity(
                            momentum.linearVelocity + convergence / momentum.tickDt,
                            momentum.angularVelocity + angleConvergence / momentum.tickDt);
                } 
                else
                {
                    momentum = new Momentum(tick, momentums[index]);
                    momentums.Insert(++index, momentum);
                }

                momentum.ChangeSituation(
                    momentum.position + convergencePerTick,
                    momentum.angle + angleConvergencePerTick);

                if (!Calculus.Approximately(momentum.tickDt, 0f))
                    momentum.ChangeVelocity(
                        momentum.linearVelocity + convergencePerTick / momentum.tickDt,
                        momentum.angularVelocity + angleConvergencePerTick / momentum.tickDt);

                convergence += convergencePerTick;
                angleConvergence += angleConvergencePerTick;
                tickDt += momentum.tickDt;

                tick++;
            }

            if (ExternalChange != null)
                ExternalChange(this, crispTick);

            return true;
        }

        #endregion
    }
}
