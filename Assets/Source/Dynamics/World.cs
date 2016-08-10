using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics
{
    using Internal;
    public class WorldFactory
    {
        public static IWorld CreateWorld(
            float crispSize = 0.01f, float actionTick = 0.01f, 
            float rememberableTime = 0f, float forseeableTime = 0f, float bufferTime = 0f)
        {
            return new World(crispSize, actionTick, rememberableTime, forseeableTime, bufferTime);
        }
    } 
}
namespace CrispyPhysics.Internal
{
    public class World : IWorld
    {
        public float crispSize { get; private set; }
        public float tick { get; private set; }
        public float actionTick { get; private set; }
        public float rememberableTime { get; private set; }
        public float foreseeableTime { get; private set; }
        public float rememberedTime { get; private set; }
        public float foreseenTime { get; private set; }
        public float bufferTime { get; set; }



        private float usedDeltaTime;
        private List<IInternalBody> bodies;
        private float lastInvDt = 0f;

        [Flags]
        private enum OperationFlag
        {
            Locked = 0x0001,
            BodyChange = 0x0002,
            FuturCleared = 0x0004,
            Crisped = 0x0008
        }

        private OperationFlag opFlags;

        public World(
            float crispSize = 0.01f, float actionTick = 0.01f,
            float rememberableTime = 0f, float foreseeableTime = 0f, float bufferTime = 0f)
        {
            this.crispSize = crispSize;
            this.actionTick = actionTick;
            this.rememberableTime = rememberableTime;
            this.foreseeableTime = foreseeableTime;
            this.bufferTime = bufferTime;

            lastInvDt = 0f;
            tick = 0f;
            rememberedTime = 0f;
            foreseenTime = 0f;

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

            Debug.Assert(false, "Remove Contact");
            opFlags |= OperationFlag.BodyChange;
            oldBody.FuturCleared -= FuturCleared;

            bodies.RemoveAt(bodyIndex);
        }

        public void Step(float dt, int velocityIterations, int positionIterations)
        {
            if (dt <= 0) return;
            bool clearFutur = 
                    (opFlags & OperationFlag.FuturCleared) == OperationFlag.FuturCleared
                ||  (opFlags & OperationFlag.BodyChange) == OperationFlag.BodyChange;

            bool lookForContacts = 
                (opFlags & OperationFlag.BodyChange) == OperationFlag.BodyChange;

            if (usedDeltaTime != dt)
            {
                usedDeltaTime = dt;
                clearFutur = true;
            }
            tick += dt;

            if (clearFutur)
            {
                foreach (IInternalBody body in bodies)
                    body.ClearFutur();
                foreseenTime = 0f;
            }

            if(lookForContacts)
            {
                //Debug.Assert(false, "Look for Contacts");
            }

            opFlags |= OperationFlag.Locked;

            TimeStep step;
            step.dt = dt;
            step.velocityIterations = velocityIterations;
            step.positionIterations = positionIterations;
            step.invDt = 1f / dt;
            step.dtRatio = lastInvDt * dt;

            //Debug.Assert(false, "Update Collider");

            float time = 0f;
            float toBuffer = Math.Min(bufferTime, foreseeableTime - foreseenTime);

            if(foreseenTime < step.dt)
                toBuffer += step.dt;

            while(time < toBuffer)
            {
                Solve(step);
                time += step.dt;
            }
            foreseenTime += time;

            //Debug.Assert(false, "Handle TOI events");

            foreach (IInternalBody body in bodies)
            {
                body.Step(step.dt);
                body.keep(rememberableTime, foreseeableTime);
            }
                

            rememberedTime = Math.Min(rememberableTime, rememberedTime + step.dt);
            foreseenTime = Math.Max(0, foreseenTime - step.dt);

            lastInvDt = step.invDt;
            opFlags &= ~OperationFlag.Locked;
            //opFlags &= ~OperationFlag.Crisped;
            opFlags &= ~OperationFlag.BodyChange;
            opFlags &= ~OperationFlag.FuturCleared;

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
        	opFlags |= World.OperationFlag.FuturCleared;
        }
    }
}