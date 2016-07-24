using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics
{
    public class World : IWorld
    {
        public float crispSize { get; set; }
        public float tick { get; private set; }
        public float actionTick { get; set; }
        public float tickRatio { get { return tick / actionTick; } }


        private List<IBody> bodies;
        private float lastInvDt = 0f;

        private bool allowSleep;
        private bool stepComplete;

        [Flags]
        private enum OperationFlag
        {
            Locked = 0x0001,
            ClearForces = 0x0002,
            NewShape = 0x0004,
            Crisped = 0x0008
        }

        private OperationFlag opFlags;

        public World(float crispSize = 1f, float actionTick = 0.1f)
        {
            this.crispSize = crispSize;
            this.actionTick = actionTick;

            lastInvDt = 0f;
            opFlags = OperationFlag.ClearForces;

            stepComplete = true;
            allowSleep = true;

            bodies = new List<IBody>();
            tick = 0f;
        }

        public void Add(IBody body)
        {
            bodies.Add(body);
            body.ShapeChanged += ShapeChanged;
        }

        public void Remove(IBody body)
        {
            Debug.Assert(false, "Remove Contact");
            body.ShapeChanged -= ShapeChanged;
            bodies.Remove(body);
        }

        public void SetAllowSleeping(bool flag)
        {
            if (flag == allowSleep) return;
            allowSleep = flag;
            if (allowSleep == false)
                foreach (IBody body in bodies)
                    body.SetAwake(true);
        }

        public void ClearForces()
        {
            foreach (IBody body in bodies)
                body.ChangeImpulse(Vector2.zero, 0f);
        }

        public bool IsLocked()
        {
            return (opFlags & OperationFlag.Locked) == OperationFlag.Locked;
        }

        public Vector2 GetGravity()
        {
            return Physics2D.gravity;
        }

        public void SetAutoClearForces(bool flag)
        {
            if (flag)
                opFlags |= OperationFlag.ClearForces;
            else
                opFlags &= ~OperationFlag.ClearForces;
        }

        public bool GetAutoClearForces()
        {
            return (opFlags & OperationFlag.ClearForces) == OperationFlag.ClearForces;
        }

        public bool IsCrisped()
        {
            return (opFlags & OperationFlag.Crisped) == OperationFlag.Crisped;
        }

        public void Step(float dt, int velocityIterations, int positionIterations)
        {
            TimeStep step;
            tick += dt;

            if ((opFlags & OperationFlag.NewShape) == OperationFlag.NewShape)
            {
                //Debug.Assert(false, "Find New Contacts");
                opFlags &= ~OperationFlag.NewShape;
            }

            opFlags |= OperationFlag.Locked;
            opFlags &= ~OperationFlag.Crisped;

            step.dt = dt;
            step.velocityIterations = velocityIterations;
            step.positionIterations = positionIterations;
            step.invDt = (dt > 0f) ? 1f / dt : 0f;
            step.dtRatio = lastInvDt * dt;

            //Debug.Assert(false, "Update Collider");

            if (stepComplete && step.dt > 0)
                Solve(step);

            //Debug.Assert(false, "Handle TOI events");

            if (step.dt > 0f) lastInvDt = step.invDt;

            if ((opFlags & OperationFlag.ClearForces) == OperationFlag.ClearForces)
                ClearForces();


            Crisp();

            opFlags &= ~OperationFlag.Locked;

        }

        private void Solve(TimeStep step)
        {
            Island island = new Island(bodies.Count);

            foreach (IBody body in bodies)
                body.SetIslandBound(false);

            //Debug.Assert(false, "Clear Contacts Island Flags");

            Stack<IBody> stack = new Stack<IBody>(bodies.Count);
            foreach (IBody seed in bodies)
            {
                if (seed.IsIslandBound())
                    continue;

                if (!seed.IsAwake() || !seed.IsActive())
                    continue;

                island.Clear();
                stack.Push(seed);
                seed.SetIslandBound(true);
                while (stack.Count > 0)
                {
                    IBody body = stack.Pop();
                    Debug.Assert(body.IsActive() == true);
                    island.Add(body);

                    body.SetAwake(true);

                    //Debug.Assert(false, "Search Contacts associated to body");

                    island.Solve(step, GetGravity(), allowSleep);
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
            if(tick >= actionTick)
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

                    Vector2 crispation = crispedPosition - body.position;

                    Vector2 crispedLinearVelocity = new Vector2(
                            Mathf.Max(
                                    Mathf.Abs(body.linearVelocity.x) 
                                -   Mathf.Abs(crispation.x) * body.linearDamping
                                ,
                                0)
                        *   Mathf.Sign(body.linearVelocity.x)
                        ,
                            Mathf.Max(
                                    Mathf.Abs(body.linearVelocity.y) 
                                -   Mathf.Abs(crispation.y) * body.linearDamping
                                ,
                                0)
                        *   Mathf.Sign(body.linearVelocity.y)
                    );

                    float timeLaps = Mathf.Max(
                        crispation.x / crispedLinearVelocity.x,
                        crispation.y / crispedLinearVelocity.y
                    );

                    float crispedAngularVelocity = 
                            body.angularVelocity
                        /   1 + timeLaps * body.angularDamping;

                    float crispedAngle = body.angle + crispedAngularVelocity * timeLaps;

                    body.ChangeSituation(crispedPosition, crispedAngle);
                    body.ChangeVelocity(crispedLinearVelocity, crispedAngularVelocity);
                }
            }
        }

        private void ShapeChanged(IBody body, EventArgs args)
        {
        	opFlags |= World.OperationFlag.NewShape;
        }
    }
}