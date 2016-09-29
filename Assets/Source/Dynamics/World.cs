using UnityEngine;
using System;
using System.Collections.Generic;

#region Factory
namespace CrispyPhysics
{
    using Internal;
    public class WorldFactory
    {
        public static IWorld CreateWorld(
            float fixedStep, float crispyStep, float crispySize,
            Vector2 gravity, int velocityIterations = 8, int positionIterations = 3,
            float maxTranslationSpeed = 100, float maxRotationSpeed = 360)
        {
            return new World(
                fixedStep, crispyStep, crispySize,
                gravity, velocityIterations, positionIterations,
                maxTranslationSpeed, maxRotationSpeed);
        }

        public static IWorld CreateWorld(WorldDefinition wordDefinition)
        {
            return new World(wordDefinition);
        }
    }
}
#endregion

namespace CrispyPhysics.Internal
{
    public class World : IWorld
    {
        #region Constructors
        public World(
            float fixedStep, float crispyStep, float crispySize,
            Vector2 gravity, int velocityIterations = 8, int positionIterations = 3,
            float maxTranslationSpeed = 100, float maxRotationSpeed = 360)
        {
            if  (   fixedStep < 0
                ||  Calculus.Approximately(fixedStep, 0f))
                throw new ArgumentOutOfRangeException("Fixed Step should be stricly greater than 0");
            
            this.fixedStep = fixedStep;
            this.crispyStep = Mathf.Max(crispyStep, this.fixedStep);
            this.crispySize = Mathf.Max(crispySize, 0f);

            this.gravity = gravity;
            this.velocityIterations = velocityIterations;
            this.positionIterations = positionIterations;

            this.maxTranslationSpeed = maxTranslationSpeed;
            this.maxRotationSpeed = maxRotationSpeed;

            tick = 0;
            pastTick = 0;
            futurTick = 0;

            bodies = new List<Body>();
            bodyContacts = new Dictionary<Body, List<Contact>>();
            contacts = new List<Contact>();
            contactManager = new ContactManager(
                new BodyIteratorDelegate(BodyIterator),
                new NewPairDelegate(NewPair),
                new ContactIteratorDelegate(ContactIterator),
                new ContactHandlerDelegate(NotifyContactStartForeseen),
                new ContactHandlerDelegate(NotifyContactEndForeseen),
                new ContactHandlerDelegate(NotifyContactForPreSolving));

            step = new TimeStep(
                fixedStep, 1f / fixedStep, 1f,
                gravity, velocityIterations, positionIterations,
                maxTranslationSpeed, maxRotationSpeed);
        }

        public World(WorldDefinition worldDef) : this(
            worldDef.fixedStep, worldDef.crispyStep, worldDef.crispySize,
            worldDef.gravity, worldDef.velocityIterations, worldDef.positionIterations,
            worldDef.maxTranslationSpeed, worldDef.maxRotationSpeed)
        {}
        #endregion

        #region Events
        public event IContactHandlerDelegate ContactStartForeseen;
        public event IContactHandlerDelegate ContactEndForeseen;
        public event IContactHandlerDelegate ContactStarted;
        public event IContactHandlerDelegate ContactEnded;
        #endregion

        #region Body & Contact Manager
        private List<Body> bodies;
        private Dictionary<Body, List<Contact>> bodyContacts;
        private List<Contact> contacts;
        private ContactManager contactManager;

        public IBody CreateBody(
            Vector2 position, float angle,
            BodyType type, IShape shape, float mass = 1f,
            float linearDamping = 0f, float angularDamping = 0f,
            float gravityScale = 1f)
        {
            Body newBody = new Body(
                tick,
                position, angle,
                type, shape, mass,
                linearDamping, angularDamping,
                gravityScale
            );

            bodies.Add(newBody);
            opFlags |= OperationFlag.BodyListUpdated;
            newBody.FuturCleared += FuturCleared;

            return newBody;
        }

        public IBody CreateBody(
            Vector2 position, float angle,
            BodyDefintion bodyDef)
        {
            return CreateBody(
                position, angle,
                bodyDef.type, bodyDef.shape, bodyDef.mass,
                bodyDef.linearDamping, bodyDef.angularDamping,
                bodyDef.gravityScale);
        }

        public void DestroyBody(IBody body)
        {
            if (!(body is Body)) return;

            Body oldBody = body as Body;

            int bodyIndex = bodies.IndexOf(oldBody);
            if (bodyIndex == -1) return;

            //Debug.Assert(false, "Remove Contact");
            opFlags |= OperationFlag.BodyListUpdated;
            oldBody.FuturCleared -= FuturCleared;

            bodies.RemoveAt(bodyIndex);
        }
        #endregion

        #region Nature
        public float fixedStep { get; private set; }
        public float crispyStep { get; private set; }
        public float crispySize { get; private set; }

        public Vector2 gravity { get; private set; }
        public int velocityIterations { get; private set; }
        public int positionIterations { get; private set; }

        public float maxTranslationSpeed { get; private set; }
        public float maxRotationSpeed { get; private set; }

        public uint tick { get; private set; }
        public uint pastTick { get; private set; }
        public uint futurTick { get; private set; }

        [Flags]
        private enum OperationFlag
        {
            Locked = 0x0001,
            BodyListUpdated = 0x0002,
            FuturForeseen = 0x0004,
            Crisped = 0x0008
        }

        private OperationFlag opFlags;
        private TimeStep step;
        #endregion

        #region Track
        public void Step(
            uint steps = 1,
            uint foreseeTicks = 0, uint bufferingTicks = 0,
            uint keepTicks = 0)
        {
            opFlags |= OperationFlag.Locked;
            tick += steps;

            bool clearFutur = 
                    (opFlags & OperationFlag.FuturForeseen) != OperationFlag.FuturForeseen
                ||  (opFlags & OperationFlag.BodyListUpdated) == OperationFlag.BodyListUpdated;

            bool lookForContacts = 
                (opFlags & OperationFlag.BodyListUpdated) == OperationFlag.BodyListUpdated;

            if (clearFutur)
            {
                foreach (Body body in bodies)
                    body.ClearFutur();

                foreach (Contact contact in contacts)
                    contact.ClearFutur();
            }

            if (keepTicks > tick) keepTicks = tick;
            pastTick = (uint)Mathf.Max(tick - keepTicks , pastTick);

            uint iterationsToSolve = 0;
            if (futurTick < tick)
            {
                iterationsToSolve = steps;
                futurTick = tick;
            }

            uint iterationsToForesee = 0;
            if (tick + foreseeTicks > futurTick)
                iterationsToForesee = (uint) 
                    Mathf.Min(
                        tick + foreseeTicks - futurTick,
                        bufferingTicks);

            futurTick  += iterationsToForesee;
            iterationsToSolve += iterationsToForesee;

            for (int i = 0; i < iterationsToSolve; i++)
            {
                foreach (Body body in bodies)
                    body.Foresee();

                foreach (Contact contact in contacts)
                    contact.Foresee();

                if (lookForContacts)
                {
                    contactManager.FindNewContacts();
                    lookForContacts = false;
                }

                contactManager.Collide();

                Solve(step);
            }

            foreach (Body body in bodies)
            {
                body.Step(steps);
                body.ForgetPast(pastTick);

                Debug.Assert(body.current.tick == tick);
                Debug.Assert(body.futur.tick == futurTick);
            }

            foreach (Contact contact in contacts)
            {
                bool wasTouching = contact.current.isTouching;
                contact.Step(steps);
                contact.ForgetPast(pastTick);

                Debug.Assert(contact.current.tick == tick);
                Debug.Assert(contact.futur.tick == futurTick);

                if(wasTouching == false && contact.current.isTouching == true)
                {
                    contact.bodyA.NotifyContactStarted(contact, EventArgs.Empty);
                    contact.bodyB.NotifyContactStarted(contact, EventArgs.Empty);

                    if (ContactStarted != null)
                        ContactStarted(contact, EventArgs.Empty);
                }

                if(wasTouching == true && contact.current.isTouching == false)
                {
                    contact.bodyA.NotifyContactEnded(contact, EventArgs.Empty);
                    contact.bodyB.NotifyContactEnded(contact, EventArgs.Empty);

                    if (ContactEnded != null)
                        ContactEnded(contact, EventArgs.Empty);
                }

            }

            opFlags &= ~OperationFlag.Locked;
            //opFlags &= ~OperationFlag.Crisped;
            opFlags &= ~OperationFlag.BodyListUpdated;
            opFlags |= OperationFlag.FuturForeseen;

        }

        public void RollBack(uint toTick, uint keepTicks = 0)
        {

            opFlags |= OperationFlag.Locked;
            tick = toTick;

            if (pastTick > tick) pastTick = tick;

            if (keepTicks > tick) keepTicks = tick;
            pastTick = (uint)Mathf.Max(tick - keepTicks, pastTick);

            foreach (Body body in bodies)
            {
                body.RollBack(tick);
                body.ForgetPast(pastTick);

                Debug.Assert(body.current.tick == tick);
                Debug.Assert(body.futur.tick == futurTick);
            }

            foreach (Contact contact in contacts)
            {
                bool wasTouching = contact.current.isTouching;
                contact.RollBack(tick);
                contact.ForgetPast(pastTick);

                Debug.Assert(contact.current.tick == tick);
                Debug.Assert(contact.futur.tick == futurTick);

                if (wasTouching == false && contact.current.isTouching == true)
                {
                    contact.bodyA.NotifyContactStarted(contact, EventArgs.Empty);
                    contact.bodyB.NotifyContactStarted(contact, EventArgs.Empty);

                    if (ContactStarted != null)
                        ContactStarted(contact, EventArgs.Empty);
                }

                if (wasTouching == true && contact.current.isTouching == false)
                {
                    contact.bodyA.NotifyContactEnded(contact, EventArgs.Empty);
                    contact.bodyB.NotifyContactEnded(contact, EventArgs.Empty);

                    if (ContactEnded != null)
                        ContactEnded(contact, EventArgs.Empty);
                }
            }

            opFlags &= ~OperationFlag.Locked;
        }

        private void Solve(TimeStep step)
        {
            Island island = new Island((uint)bodies.Count);

            foreach (Body body in bodies)
                body.islandBound = false;

            //Debug.Assert(false, "Clear Contacts Island Flags");

            Stack<Body> stack = new Stack<Body>(bodies.Count);
            foreach (Body seed in bodies)
            {
                if (seed.islandBound)
                    continue;
                if (seed.type == BodyType.Static)
                    continue;

                island.Clear();
                stack.Push(seed);
                seed.islandBound = true;
                while (stack.Count > 0)
                {
                    Body body = stack.Pop();
                    island.Add(body);

                    if (body.type == BodyType.Static)
                        continue;

                    //Debug.Assert(false, "Search Contacts associated to body");

                    island.Solve(step);

                    foreach (Body islandBody in island.BodyIterator())
                    {
                        if(islandBody.type == BodyType.Static)
                            islandBody.islandBound = false;
                    }
                }
            }

            //Debug.Assert(false, "Find New Contacts");
        }

        private void Crisp()
        {
            /*if(tick >= actionTick)
            {
                tick = tick - actionTick;
                opFlags |= OperationFlag.Crisped;
                foreach (IBody body in bodies)
                {
                    Vector2 crispedPosition;

                    if (Mathf.Sign(body.position.x * body.linearVelocity.x) >= 0)
                        crispedPosition.x = 
                                Mathf.Ceil(Mathf.Abs(body.position.x / crispSize)) 
                            *   crispSize * Mathf.Sign(body.position.x);
                    else
                        crispedPosition.x = 
                                Mathf.Floor(Mathf.Abs(body.position.x / crispSize)) 
                            *   crispSize * Mathf.Sign(body.position.x);

                    if (Mathf.Sign(body.position.y * body.linearVelocity.y) >= 0)
                        crispedPosition.y = 
                                Mathf.Ceil(Mathf.Abs(body.position.y / crispSize)) 
                            *   crispSize * Mathf.Sign(body.position.y);
                    else
                        crispedPosition.y = 
                                Mathf.Floor(Mathf.Abs(body.position.y / crispSize)) 
                            *   crispSize * Mathf.Sign(body.position.y);

                    body.ChangeSituation(crispedPosition, body.angle);
                }
            }*/
        }
        #endregion

        #region Event Handlers & Delegates
        private void FuturCleared(IBody body, EventArgs args)
        {
            if((opFlags & World.OperationFlag.FuturForeseen) == World.OperationFlag.FuturForeseen)
            {
                futurTick = tick;
                opFlags &= ~World.OperationFlag.FuturForeseen;
            }
        }


        private IEnumerable<Body> BodyIterator(uint start = 0, uint end = 0)
        {
            if (end == 0)
                end = (uint)bodies.Count;
            for (int i = (int)start; i < (int)end; i++)
                yield return bodies[i];
        }

        private void NewPair(Body bodyA, Body bodyB)
        {
            if (bodyA == bodyB)
                return;

            List<Contact> bodyAContacts = null;
            List<Contact> bodyBContacts = null;

            if (bodyContacts.ContainsKey(bodyA))
            {
                bodyAContacts = bodyContacts[bodyA];
                foreach (Contact contact in bodyAContacts)
                    if (contact.bodyA == bodyB || contact.bodyB == bodyB)
                        return;
            }
            else
            {
                bodyAContacts = new List<Contact>();
                bodyContacts.Add(bodyA, bodyAContacts);
            }

            if (bodyContacts.ContainsKey(bodyB))
                bodyBContacts = bodyContacts[bodyB];
            else
            {
                bodyBContacts = new List<Contact>();
                bodyContacts.Add(bodyB, bodyBContacts);
            }

            Debug.Assert(bodyA.futur.tick == bodyB.futur.tick);
            Contact newContact = ContactFactory.CreateContact(bodyA.futur.tick, bodyA, bodyB);
            bodyAContacts.Add(newContact);
            bodyBContacts.Add(newContact);
            contacts.Add(newContact);
        }

        private IEnumerable<Contact> ContactIterator(uint start = 0, uint end = 0)
        {
            if (end == 0)
                end = (uint)contacts.Count;
            for (int i = (int)start; i < (int)end; i++)
                yield return contacts[i];
        }

        private void NotifyContactStartForeseen(Contact contact, EventArgs args)
        {
            contact.bodyA.NotifyContactStartForeseen(contact, args);
            contact.bodyB.NotifyContactStartForeseen(contact, args);

            if (ContactStartForeseen != null)
                ContactStartForeseen(contact, args);
        }

        private void NotifyContactEndForeseen(Contact contact, EventArgs args)
        {
            contact.bodyA.NotifyContactStartForeseen(contact, EventArgs.Empty);
            contact.bodyB.NotifyContactStartForeseen(contact, EventArgs.Empty);

            if (ContactEndForeseen != null)
                ContactEndForeseen(contact, args);
        }

        private void NotifyContactForPreSolving(Contact contact, EventArgs args)
        {
            Debug.Assert(false, "Resolve Presolving");
        }
        #endregion
    }
}