using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics.Internal
{
    #region Delegate Definition
    public delegate void ContactHandlerDelegate(Contact contact, ContactMomentum momentum);
    public delegate void NewPairDelegate(Body bodyA, Body bodyB);
    public delegate IEnumerable<Contact> ContactIteratorDelegate();
    #endregion

    #region Contact Factory
    public class ContactFactory
    {
        public static Contact CreateContact(uint tick, Body bodyA, Body bodyB)
        {
            if (bodyA == null)
                throw new ArgumentNullException("bodyA should not be null");
            if (bodyB == null)
                throw new ArgumentNullException("bodyB should not be null");
            if (bodyA.shape == null)
                throw new ArgumentException("Shape of bodyA should not be null");
            if (bodyB.shape == null)
                throw new ArgumentException("Shape of bodyB should not be null");

            IShape shapeA = bodyA.shape;
            IShape shapeB = bodyB.shape;

            if (shapeA is CircleShape && shapeB is CircleShape)
                return new CircleContact(tick, bodyA, bodyB);

            if (shapeA is EdgeShape && shapeB is CircleShape)
                return new EdgeAndCircleContact(tick, bodyA, bodyB);

            if (shapeA is CircleShape && shapeB is EdgeShape)
                return new EdgeAndCircleContact(tick, bodyB, bodyA);

            if (shapeA is PolygonShape && shapeB is CircleShape)
                return new PolygonAndCircleContact(tick, bodyA, bodyB);

            if (shapeA is CircleShape && shapeB is PolygonShape)
                return new PolygonAndCircleContact(tick, bodyB, bodyA);

            return null;
        }
    }
    #endregion

    public abstract class Contact : IContact
    {
        #region Constructors
        public Contact(uint tick, Body bodyA, Body bodyB)
        {
            if (bodyA == null)
                throw new ArgumentNullException("bodyA should not be null");
            if (bodyB == null)
                throw new ArgumentNullException("bodyB should not be null");

            if (bodyA.shape == null)
                throw new ArgumentException("Shape of bodyA should not be null");
            if (bodyB.shape == null)
                throw new ArgumentException("Shape of bodyB should not be null");

            currentTick = tick;
            this.bodyA = bodyA;
            this.bodyB = bodyB;

            friction = MixFriction(bodyA.friction, bodyB.friction);
            restitution = MixRestitution(bodyA.restitution, bodyB.restitution);

            momentums = new List<ContactMomentum>();
            momentums.Add(new ContactMomentum(currentTick));

            islandBound = false;
            currentIndex = 0;
        }
        #endregion

        #region Nature
        public Body bodyA { get; private set; }
        public IBody firstBody { get { return bodyA as IBody; } }
        public Body bodyB { get; private set; }
        public IBody secondBody { get { return bodyB as IBody; } }
        public float friction { get; private set; }
        public float restitution { get; private set; }

        public WorldManifold GetWorldManifold()
        {
            if (current.manifold == null)
                return null;

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

        #region Behavior
        public abstract Manifold Evaluate(Transformation transformA, Transformation transformB);
        #endregion

        #region Track
        private uint currentTick;
        private int currentIndex;
        public ContactMomentum past { get { return momentums[0]; } }
        public ContactMomentum current { get { return momentums[currentIndex]; } }
        public ContactMomentum futur { get { return momentums[momentums.Count - 1]; } }
        private List<ContactMomentum> momentums;
        public bool islandBound { get; set; }

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
                if (momentums[currentIndex].tick < currentTick)
                    currentIndex++;
                momentums.Insert(
                    currentIndex,
                    new ContactMomentum(
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
                    new ContactMomentum(
                        currentTick,
                        momentums[sourceIndex]));
            }
        }

        public void Foresee(uint steps = 1)
        {
            ContactMomentum futurMomentum = new ContactMomentum(
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
            if (fromTick == 0)
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
                        new ContactMomentum(
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
            if (fromTick <= currentTick)
                throw new ArgumentOutOfRangeException("Tick to be cleared should be above the current tick");

            int indexForTick = IndexForTick(fromTick);
            if (indexForTick == currentIndex)
                indexForTick++;

            if (indexForTick < momentums.Count)
                momentums.RemoveRange(indexForTick, momentums.Count - indexForTick);
        }

        public int IndexForTick(uint tick)
        {
            int index = (tick >= currentTick) ? currentIndex : 0;
            while (index < momentums.Count)
                if (momentums[index].tick == tick) return index;
                else if (momentums[index].tick > tick) return index - 1;
                else index++;

            return -1;
        }

        public bool IsDroppable()
        {
            foreach(ContactMomentum momentum in momentums)
                if (momentum.isTouching)
                    return false;

            return true;
        }


        #endregion
    }
}