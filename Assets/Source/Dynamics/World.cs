using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics
{
    public class World
    {
        public float crispSize = 1.0f;
        public float tick { get; private set; }
        public float tickRatio { get { return tick / actionTick; } }

        [Flags]
        public enum OperationFlags
        {
            Locked = 0x0001,
            ClearForces = 0x0002,
            NewShape = 0x0004,
            Crisped = 0x0008
        }

        public OperationFlags opFlags;

        private float _actionTick;
        public float actionTick {
            get { return _actionTick; }
            set { _actionTick = Mathf.Max(value, Time.fixedDeltaTime);}
        }

        private List<Body> bodies;
        private float lastInvDt = 0f;

        private bool allowSleep;
        private bool stepComplete;

        public World(float crispSize = 1f, float actionTick = 0.1f)
        {
            this.crispSize = crispSize;
            this._actionTick = actionTick;

            lastInvDt = 0f;
            opFlags = OperationFlags.ClearForces;

            stepComplete = true;
            allowSleep = true;

            bodies = new List<Body>();
            tick = 0f;
        }

        public void AddBody(Body body)
        {
            bodies.Add(body);
        }

        public void RemoveBody(Body body)
        {
            Debug.Assert(false, "Remove Contact");
            bodies.Remove(body);
        }

        public void SetAllowSleeping(bool flag)
        {
            if (flag == allowSleep) return;
            allowSleep = flag;
            if (allowSleep == false)
                foreach (Body body in bodies)
                    body.SetAwake(true);
        }

        public void ClearForces()
        {
            foreach (Body body in bodies)
            {
                body.force = Vector2.zero;
                body.torque = 0f;
            }
        }

        public bool IsLocked()
        {
            return (opFlags & OperationFlags.Locked) == OperationFlags.Locked;
        }

        public Vector2 GetGravity()
        {
            return Physics2D.gravity;
        }

        public void SetAutoClearForces(bool flag)
        {
            if (flag)
                opFlags |= OperationFlags.ClearForces;
            else
                opFlags &= ~OperationFlags.ClearForces;
        }

        public bool GetAutoClearForces()
        {
            return (opFlags & OperationFlags.ClearForces) == OperationFlags.ClearForces;
        }

        public bool IsCrisped()
        {
            return (opFlags & OperationFlags.Crisped) == OperationFlags.Crisped;
        }

        public void Step(float dt, int velocityIterations, int positionIterations)
        {
            TimeStep step;
            tick += dt;

            if ((opFlags & OperationFlags.NewShape) == OperationFlags.NewShape)
            {
                //Debug.Assert(false, "Find New Contacts");
                opFlags &= ~OperationFlags.NewShape;
            }

            opFlags |= OperationFlags.Locked;
            opFlags &= ~OperationFlags.Crisped;

            
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

            if ((opFlags & OperationFlags.ClearForces) == OperationFlags.ClearForces)
                ClearForces();


            Crisp();

            opFlags &= ~OperationFlags.Locked;

        }

        private void Solve(TimeStep step)
        {
            Island island = new Island(bodies.Count);

            foreach(Body body in bodies)
                body.opFlags &= ~Body.OperationFlags.Island;

            //Debug.Assert(false, "Clear Contacts Island Flags");

            Stack<Body> stack = new Stack<Body>(bodies.Count);
            foreach (Body seed in bodies)
            {
                if ((seed.opFlags & Body.OperationFlags.Island) == Body.OperationFlags.Island)
                    continue;

                if (!seed.IsAwake() || !seed.IsActive())
                    continue;

                island.Clear();
                stack.Push(seed);
                seed.opFlags |= Body.OperationFlags.Island;
                while (stack.Count > 0)
                {
                    Body body = stack.Pop();
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
                opFlags |= OperationFlags.Crisped;
                foreach (Body body in bodies)
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

                    body.sweep.center = crispedPosition;
                    body.sweep.angle = crispedAngle;
                    body.linearVelocity = crispedLinearVelocity;
                    body.angularVelocity = crispedAngularVelocity;
                    body.SynchronizeTransform();
                }
            }
        }
    }
}