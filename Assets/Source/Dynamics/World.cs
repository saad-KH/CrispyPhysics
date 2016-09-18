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

            bodies = new List<IInternalBody>();

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

        #region Body Manager
        private List<IInternalBody> bodies;

        public IBody CreateBody(
            Vector2 position, float angle,
            BodyType type, IShape shape, float mass = 1f,
            float linearDamping = 0f, float angularDamping = 0f,
            float gravityScale = 1f)
        {
            IInternalBody newBody = new Body(
                tick,
                position, angle,
                type, shape, mass,
                linearDamping, angularDamping,
                gravityScale
            );

            bodies.Add(newBody);
            opFlags |= OperationFlag.BodyChange;
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
            if (!(body is IInternalBody)) return;

            IInternalBody oldBody = body as IInternalBody;

            int bodyIndex = bodies.IndexOf(oldBody);
            if (bodyIndex == -1) return;

            //Debug.Assert(false, "Remove Contact");
            opFlags |= OperationFlag.BodyChange;
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
            BodyChange = 0x0002,
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
                ||  (opFlags & OperationFlag.BodyChange) == OperationFlag.BodyChange;

            bool lookForContacts = 
                (opFlags & OperationFlag.BodyChange) == OperationFlag.BodyChange;

            if (clearFutur)
                foreach (IInternalBody body in bodies)
                    body.ClearFutur();
            
            if(lookForContacts)
            {
                //Debug.Assert(false, "Look for Contacts");
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
                //Debug.Assert(false, "Destroy useless contacts");
                Solve(step);
            }

            foreach (IInternalBody body in bodies)
            {
                body.Step(steps);
                body.ForgetPast(pastTick);

                Debug.Assert(body.current.tick == tick);
                Debug.Assert(body.past.tick == pastTick);
                Debug.Assert(body.futur.tick == futurTick);
            }

            opFlags &= ~OperationFlag.Locked;
            //opFlags &= ~OperationFlag.Crisped;
            opFlags &= ~OperationFlag.BodyChange;
            opFlags |= OperationFlag.FuturForeseen;

        }

        public void RollBack(uint toTick, uint keepTicks = 0)
        {

            opFlags |= OperationFlag.Locked;
            tick = toTick;

            if (pastTick > tick) pastTick = tick;

            if (keepTicks > tick) keepTicks = tick;
            pastTick = (uint)Mathf.Max(tick - keepTicks, pastTick);

            foreach (IInternalBody body in bodies)
            {
                body.RollBack(tick);
                body.ForgetPast(pastTick);

                Debug.Assert(body.current.tick == tick);
                Debug.Assert(body.past.tick == pastTick);
                Debug.Assert(body.futur.tick == futurTick);
            }
            opFlags &= ~OperationFlag.Locked;
        }

        private void Solve(TimeStep step)
        {
            Island island = new Island(bodies.Count);

            foreach (IInternalBody body in bodies)
                body.islandBound = false;

            //Debug.Assert(false, "Clear Contacts Island Flags");

            Stack<IInternalBody> stack = new Stack<IInternalBody>(bodies.Count);
            foreach (IInternalBody seed in bodies)
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
                    IInternalBody body = stack.Pop();
                    island.Add(body);

                    if (body.type == BodyType.Static)
                        continue;

                    //Debug.Assert(false, "Search Contacts associated to body");

                    island.Solve(step);

                    //foreach
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

        private void FuturCleared(IBody body, EventArgs args)
        {
            if((opFlags & World.OperationFlag.FuturForeseen) == World.OperationFlag.FuturForeseen)
            {
                futurTick = tick;
                opFlags &= ~World.OperationFlag.FuturForeseen;
            }
        }
        #endregion
    }
}