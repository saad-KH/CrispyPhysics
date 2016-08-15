using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics
{
    using Internal;
    public class WorldFactory
    {
        public static IWorld CreateWorld(
            float fixedStep = 0.01f, float crispyStep = 0.01f, float crispySize = 0.01f)
        {
            return new World(fixedStep, crispyStep, crispySize);
        }
    } 
}
namespace CrispyPhysics.Internal
{
    public class World : IWorld
    {
        public float fixedStep { get; private set; }
        public float crispyStep { get; private set; }
        public float crispySize { get; private set; }
        public float tick { get; private set; }
        public float rememberedTime { get { return tick - pastTick; } }
        public float foreseenTime { get { return futurTick - tick; } }

        private List<IInternalBody> bodies;
        private float pastTick;
        private float futurTick;

        [Flags]
        private enum OperationFlag
        {
            Locked = 0x0001,
            BodyChange = 0x0002,
            FuturForeseen = 0x0004,
            Crisped = 0x0008
        }

        private OperationFlag opFlags;

        public World(float fixedStep = 0.01f, float crispyStep = 0.01f, float crispySize = 0.01f)
        {
            if  (   fixedStep < 0
                ||  Calculus.Approximately(fixedStep, 0f))
                throw new ArgumentOutOfRangeException("Fixed Step should be stricly greater than 0");
            
            this.fixedStep = fixedStep;
            this.crispyStep = Mathf.Max(crispyStep, this.fixedStep);
            this.crispySize = Mathf.Max(crispySize, 0f);

            tick = 0f;
            pastTick = 0f;
            futurTick = 0f;

            bodies = new List<IInternalBody>();
        }

        public IBody CreateBody(
            BodyType type, IShape shape,
            Vector2 position, float angle,
            float linearDamping = 0f, float angularDamping = 0f,
            float mass = 1f, float gravityScale = 1f)
        {
            IInternalBody newBody = new Body(
                type, shape,
                position, angle,
                linearDamping, angularDamping,
                mass, gravityScale
            );

            bodies.Add(newBody);
            opFlags |= OperationFlag.BodyChange;
            newBody.FuturCleared += FuturCleared;

            return newBody;
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

        public void Step(
            float dt, 
            float foresee = 0f, float bufferingCap = 0f,
            float keepPast = 0f)
        {
            if  (   dt < 0
                ||  Calculus.Approximately(dt, 0f))
                throw new ArgumentOutOfRangeException("Step Delta time should be stricly greater than 0");
            
            opFlags |= OperationFlag.Locked;
            tick += dt;

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

            TimeStep step;
            step.dt = fixedStep;
            step.velocityIterations = 8;
            step.positionIterations = 3;
            step.invDt = 1f / dt;
            step.dtRatio = 1f;

            
            
            float toBuffer = 0f;
            float foreseenTime = futurTick - tick;

            if (    foreseenTime < 0f
               &&   !Calculus.Approximately(dt, foreseenTime))
            {
                toBuffer += Mathf.Abs(foreseenTime);
                foreseenTime = 0f;
            }

            toBuffer += Mathf.Max(foresee, foreseenTime) - foreseenTime;
            toBuffer = Mathf.Min(toBuffer, Mathf.Max(bufferingCap + dt, 0f));

            while (toBuffer >= step.dt || Calculus.Approximately(toBuffer, step.dt))
            {
                Solve(step);
                //Debug.Assert(false, "Handle TOI events");
                //Debug.Assert(false, "Update Collider");
                toBuffer -= step.dt;
            }

            pastTick = tick - keepPast;
            futurTick = tick + foresee;
            foreach (IInternalBody body in bodies)
            {
                body.Step(dt);
                body.ForgetPast(keepPast);

                if (    body.past.tick > pastTick
                    &&  !Calculus.Approximately(body.past.tick, pastTick))
                    pastTick = body.past.tick;

                if (    body.futur.tick < futurTick
                    &&  !Calculus.Approximately(body.futur.tick, futurTick))
                    futurTick = body.futur.tick;
            }


            opFlags &= ~OperationFlag.Locked;
            //opFlags &= ~OperationFlag.Crisped;
            opFlags &= ~OperationFlag.BodyChange;
            opFlags |= OperationFlag.FuturForeseen;

        }

        public void StepBack(float dt, float keepPast = 0f)
        {
            if  (   dt < 0
                ||  Calculus.Approximately(dt, 0f))
                throw new ArgumentOutOfRangeException("Step Delta time should be stricly greater than 0");

            opFlags |= OperationFlag.Locked;
            tick -= dt;

            pastTick = tick - keepPast;
            foreach (IInternalBody body in bodies)
            {
                body.StepBack(dt);
                body.ForgetPast(keepPast);

                if (    body.past.tick > pastTick
                    &&  !Calculus.Approximately(body.past.tick, pastTick))
                    pastTick = body.past.tick;

                if (    body.futur.tick < futurTick
                    &&  !Calculus.Approximately(body.futur.tick, futurTick))
                    futurTick = body.futur.tick;
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

                island.Clear();
                stack.Push(seed);
                seed.islandBound = (true);
                while (stack.Count > 0)
                {
                    IInternalBody body = stack.Pop();
                    island.Add(body);

                    //Debug.Assert(false, "Search Contacts associated to body");

                    island.Solve(step, Physics2D.gravity);
                }
            }

            //Debug.Assert(false, "Find New Contacts");
        }

        private void SolveTOI(TimeStep step)
        {
            Debug.Assert(false, "SolveTOI");
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
    }
}