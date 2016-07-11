using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics
{
    public class Body : IBody
    {
        public enum Types
        {
            DynamicBody
        }

        [Flags]
        public enum OperationFlags
        {
            Island = 0x0001,
            Awake = 0x0002,
            AutoSleep = 0x0004,
            FixedRotation = 0x0008,
            Active = 0x0010,
            TOI = 0x0020
        }

        public OperationFlags opFlags;

        public Types type { get; private set; }

        public Shape shape { get; private set; }

        public Vector2 linearVelocity;
        public float angularVelocity;
        public Vector2 force;
        public float torque;

        public float mass { get; private set; }
        public float invMass { get; private set; }
        private float rotationalInertia, invRotationalInertia;

        public float linearDamping;
        public float angularDamping;
        public float gravityScale;

        public int islandIndex;

        public Transformation transform;
        public Sweep sweep;

        public float sleepTime;

        public World world { get; private set; }
        public List<Contact> contacts { get; private set; }

        public Vector2 position { get { return transform.position; } }
        public float angle { get { return sweep.angle; } }
        public Vector2 worldCenter { get { return sweep.center; } }

        public Body(
            Types type,
            World world, 
            Vector2 position, 
            float angle,
            float linearDamping = 0f,
            float angularDamping = 0f,
            float mass = 1f,
            float gravityScale = 1f,
            bool fixedRotation = false,
            bool allowSleep = false,
            bool awake = true,
            bool active = true
            )
        {
            Debug.Assert(world != null);

            this.type = type;
            opFlags = 0;

            if (fixedRotation) opFlags |= OperationFlags.FixedRotation;
            if (allowSleep) opFlags |= OperationFlags.AutoSleep;
            if (awake) opFlags |= OperationFlags.Awake;
            if (active) opFlags |= OperationFlags.Active;

            this.type = type;
            this.world = world;

            transform.Set(position, angle);

            sweep.Reset(transform.position, angle);

            SetMass(mass);

            contacts = new List<Contact>();         

            linearVelocity = Vector2.zero;
            angularVelocity = 0f;

            this.linearDamping = linearDamping;
            this.angularDamping = angularDamping;
            this.gravityScale = gravityScale;

            force = Vector2.zero;
            torque = 0f;

            rotationalInertia = 0f;
            invRotationalInertia = 0f;

            sleepTime = 0f;
        }

        public void SetMass(float mass)
        {
            if (type != Types.DynamicBody)
            {
                this.mass = invMass = 0f;
                return;
            }

            this.mass = (mass > 0) ? mass : 1f;
            invMass =  1f / this.mass;
        }

        public void DefineShape(Shape shape)
        {
            if (this.shape != null)
            {
                Debug.Assert(false, "Clean Contacts");
            }

            this.shape = shape;
            Debug.Assert(false, "Search Contacts");
            Debug.Assert(false, "Broad Phase");
            if (type == Types.DynamicBody)
                CalculateInertia();

            world.opFlags |= World.OperationFlags.NewShape;
        }

        public bool ShouldCollide(Body other)
        {
            if (type != Types.DynamicBody && other.type != Types.DynamicBody)
                return false;
            return true;
        }

        public void SetTransform(Vector2 position, float angle)
        {
            if (world.IsLocked()) return;
            transform.Set(position, angle);

            sweep.center0 = sweep.center = transform.position;
            sweep.angle0 = sweep.angle = angle;

            Debug.Assert(false, "Broad Phase");
        }

        public void CalculateInertia()
        {
            rotationalInertia = 0f;
            invRotationalInertia = 0f;

            Debug.Assert(type == Types.DynamicBody);
            Debug.Assert(false, "Find Inertia of body");
        }

        public void SetActive(bool flag)
        {
            Debug.Assert(world.IsLocked() == false);
            if (flag == IsActive()) return;

            if (flag)
            {
                opFlags |= OperationFlags.Active;
                //Debug.Assert(false, "Broad Phase");
            }
            else
            {
                opFlags &= ~OperationFlags.Active;
                //Debug.Assert(false, "Broad Phase");
                //Debug.Assert(false, "Remove Contacts");
            }
        }

        public void SetFixedRotation(bool flag)
        {
            if (flag == IsFixedRotation()) return;

            if (flag)
                opFlags |= OperationFlags.FixedRotation;
            else
                opFlags &= ~OperationFlags.FixedRotation;

            angularVelocity = 0f;

            CalculateInertia();
        }

        public void SetLinearVelocity(Vector2 velocity)
        {
            if (Vector2.Dot(velocity, velocity) > 0) SetAwake(true);
            linearVelocity = velocity;
        }

        public void SetAngularVelocity(float velocity)
        {
            if (velocity * velocity > 0f) SetAwake(true);
            angularVelocity = velocity;
        }

        public float GetInertia()
        {
            return rotationalInertia;
        }

        public Vector2 GetWorldPoint(Vector2 localPoint)
        {
            return Calculus.Mul(transform, localPoint);
        }

        public Vector2 GetWorldVector(Vector2 localVector)
        {
            return Calculus.Mul(transform.rotation, localVector);
        }

        public Vector2 GetLocalPoint(Vector2 worldPoint)
        {
            return Calculus.MulT(transform, worldPoint);
        }

        public Vector2 GetLocalVector(Vector2 worldVector)
        {
            return Calculus.MulT(transform.rotation, worldVector);
        }

        public Vector2 GetLinearVelocityFromWorldPoint(Vector2 worldPoint)
        {
            return linearVelocity + Calculus.Cross(angularVelocity , worldPoint - sweep.center);
        }

        public Vector2 GetLinearVelocityFromLocalPoint(Vector2 localPoint)
        {
            return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
        }

        public void SetAwake(bool flag)
        {
            if (flag)
            {
                if( (opFlags & OperationFlags.Awake) == 0)
                {
                    opFlags |= OperationFlags.Awake;
                    sleepTime = 0f;
                }
            }
            else
            {
                opFlags &= ~OperationFlags.Awake;
                sleepTime = 0f;
                linearVelocity = Vector2.zero;
                angularVelocity = 0f;
                force = Vector2.zero;
                torque = 0f;
            }
        }

        public bool IsAwake()
        {
            return (opFlags & OperationFlags.Awake) == OperationFlags.Awake;
        }

        public bool IsActive()
        {
            return (opFlags & OperationFlags.Active) == OperationFlags.Active;
        }

        public bool IsFixedRotation()
        {
            return (opFlags & OperationFlags.FixedRotation) == OperationFlags.FixedRotation;
        }

        public void SetSleepingAllowed(bool flag)
        {
            if(flag)
            {
                opFlags |= OperationFlags.AutoSleep;
            }
            else
            {
                opFlags &= ~OperationFlags.AutoSleep;
                SetAwake(true);
            }
        }

        public bool IsSleepingAllowed()
        {
            return (opFlags & OperationFlags.AutoSleep) == OperationFlags.AutoSleep;
        }

        public void ApplyForce(Vector2 force, Vector2 point, bool wake)
        {
            if (type != Types.DynamicBody) return;
            if (wake && (opFlags & OperationFlags.Awake) == 0) SetAwake(true);

            if((opFlags & OperationFlags.Awake) == OperationFlags.Awake)
            {
                this.force += force;
                this.torque += Calculus.Cross(point - sweep.center, force);
            }
        }

        public void ApplyForceToCenter(Vector2 force, bool wake)
        {
            if (type != Types.DynamicBody) return;
            if (wake && (opFlags & OperationFlags.Awake) == 0) SetAwake(true);

            if ((opFlags & OperationFlags.Awake) == OperationFlags.Awake)
                this.force += force;
        }

        public void ApplyTorque(float torque, bool wake)
        {
            if (type != Types.DynamicBody) return;
            if (wake && (opFlags & OperationFlags.Awake) == 0) SetAwake(true);

            if ((opFlags & OperationFlags.Awake) == OperationFlags.Awake)
                this.torque += torque;
        }

        public void ApplyLinearImpulse(Vector2 impulse, Vector2 point, bool wake)
        {
            if (type != Types.DynamicBody) return;
            if (wake && (opFlags & OperationFlags.Awake) == 0) SetAwake(true);

            if ((opFlags & OperationFlags.Awake) == OperationFlags.Awake)
            {
                linearVelocity += invMass * impulse;
                angularVelocity += 
                        invRotationalInertia 
                    *   Calculus.Cross(point - sweep.center, impulse);
            }
        }

        public void ApplyLinearImpulseToCenter(Vector2 impulse, bool wake)
        {
            if (type != Types.DynamicBody) return;
            if (wake && (opFlags & OperationFlags.Awake) == 0) SetAwake(true);

            if ((opFlags & OperationFlags.Awake) == OperationFlags.Awake)
                linearVelocity += invMass * impulse;
        }

        public void ApplyAngularImpulse(float impulse, bool wake)
        {
            if (type != Types.DynamicBody) return;
            if (wake && (opFlags & OperationFlags.Awake) == 0) SetAwake(true);

            if ((opFlags & OperationFlags.Awake) == OperationFlags.Awake)
                angularVelocity += invRotationalInertia * impulse;
        }

        public void SynchronizeTransform()
        {
            transform.Set(
                sweep.center,
                sweep.angle
            );
        }


        private void Advance(float alpha)
        {
            sweep.Advance(alpha);
            sweep.center = sweep.center0;
            sweep.angle = sweep.angle0;
            transform.Set(
                sweep.center,
                sweep.angle
            );
        }
    }
}
