using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics
{
    public class Body : IBody
    {
        public BodyType type { get; private set; }

        public IShape shape { get; private set; }

        public Vector2 linearVelocity { get; private set; }
        public float angularVelocity { get; private set; }
        public Vector2 force { get; private set; }
        public float torque { get; private set; }

        public float mass { get; private set; }
        public float invMass { get; private set; }
        private float rotationalInertia, invRotationalInertia;

        public float linearDamping { get; set; }
        public float angularDamping { get; set; }
        public float gravityScale { get; set; }

        public float sleepTime { get; set; }

        private Transformation transform;

        public IWorld world { get; private set; }
        public List<IContact> contacts { get; private set; }

        public Vector2 position { get { return transform.position; } }
        public float angle { get { return transform.rotation.GetAngle(); } }

        [Flags]
        private enum OperationFlag
        {
            Island = 0x0001,
            Awake = 0x0002,
            AutoSleep = 0x0004,
            FixedRotation = 0x0008,
            Active = 0x0010,
            TOI = 0x0020
        }

        private OperationFlag opFlags;

        public Body(
            BodyType type,
            IWorld world, 
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

            if (fixedRotation) opFlags |= OperationFlag.FixedRotation;
            if (allowSleep) opFlags |= OperationFlag.AutoSleep;
            if (awake) opFlags |= OperationFlag.Awake;
            if (active) opFlags |= OperationFlag.Active;

            this.type = type;
            this.world = world;

            transform.Set(position, angle);

            SetMass(mass);

            contacts = new List<IContact>();         

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
            if (type != BodyType.DynamicBody)
            {
                this.mass = invMass = 0f;
                return;
            }

            this.mass = (mass > 0) ? mass : 1f;
            invMass =  1f / this.mass;
        }

        public void DefineShape(IShape shape)
        {
            if (this.shape != null)
            {
                Debug.Assert(false, "Clean Contacts");
            }

            this.shape = shape;
            Debug.Assert(false, "Search Contacts");
            Debug.Assert(false, "Broad Phase");
            if (type == BodyType.DynamicBody)
                CalculateInertia();

            world.NotifyShapeAdded();
        }

        public bool ShouldCollide(IBody other)
        {
            if (type != BodyType.DynamicBody && other.type != BodyType.DynamicBody)
                return false;
            return true;
        }

        public void ChangeSituation(Vector2 position, float angle)
        {
            if (world.IsLocked()) return;
            transform.Set(position, angle);

            Debug.Assert(false, "Broad Phase");
        }

        public void CalculateInertia()
        {
            rotationalInertia = 0f;
            invRotationalInertia = 0f;

            Debug.Assert(type == BodyType.DynamicBody);
            Debug.Assert(false, "Find Inertia of body");
        }

        public void SetActive(bool flag)
        {
            Debug.Assert(world.IsLocked() == false);
            if (flag == IsActive()) return;

            if (flag)
            {
                opFlags |= OperationFlag.Active;
                //Debug.Assert(false, "Broad Phase");
            }
            else
            {
                opFlags &= ~OperationFlag.Active;
                //Debug.Assert(false, "Broad Phase");
                //Debug.Assert(false, "Remove Contacts");
            }
        }

        public void SetFixedRotation(bool flag)
        {
            if (flag == IsFixedRotation()) return;

            if (flag)
                opFlags |= OperationFlag.FixedRotation;
            else
                opFlags &= ~OperationFlag.FixedRotation;

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
            return linearVelocity + Calculus.Cross(angularVelocity , worldPoint - transform.position);
        }

        public Vector2 GetLinearVelocityFromLocalPoint(Vector2 localPoint)
        {
            return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
        }

        public void SetAwake(bool flag)
        {
            if (flag)
            {
                if( (opFlags & OperationFlag.Awake) == 0)
                {
                    opFlags |= OperationFlag.Awake;
                    sleepTime = 0f;
                }
            }
            else
            {
                opFlags &= ~OperationFlag.Awake;
                sleepTime = 0f;
                linearVelocity = Vector2.zero;
                angularVelocity = 0f;
                force = Vector2.zero;
                torque = 0f;
            }
        }

        public bool IsAwake()
        {
            return (opFlags & OperationFlag.Awake) == OperationFlag.Awake;
        }

        public bool IsActive()
        {
            return (opFlags & OperationFlag.Active) == OperationFlag.Active;
        }

        public bool IsFixedRotation()
        {
            return (opFlags & OperationFlag.FixedRotation) == OperationFlag.FixedRotation;
        }

        public void SetSleepingAllowed(bool flag)
        {
            if(flag)
            {
                opFlags |= OperationFlag.AutoSleep;
            }
            else
            {
                opFlags &= ~OperationFlag.AutoSleep;
                SetAwake(true);
            }
        }

        public bool IsSleepingAllowed()
        {
            return (opFlags & OperationFlag.AutoSleep) == OperationFlag.AutoSleep;
        }

        public void SetIslandBound(bool flag)
        {
            if (flag)
                opFlags |= OperationFlag.Island;
            else
                opFlags &= ~OperationFlag.Island;
        }

        public bool IsIslandBound()
        {
            return (opFlags & OperationFlag.Island) == OperationFlag.Island;
        }

        public void ApplyForce(Vector2 force, Vector2 point, bool wake)
        {
            if (type != BodyType.DynamicBody) return;
            if (wake && (opFlags & OperationFlag.Awake) == 0) SetAwake(true);

            if((opFlags & OperationFlag.Awake) == OperationFlag.Awake)
            {
                this.force += force;
                torque += Calculus.Cross(point - transform.position, force);
            }
        }

        public void ApplyForceToCenter(Vector2 force, bool wake)
        {
            if (type != BodyType.DynamicBody) return;
            if (wake && (opFlags & OperationFlag.Awake) == 0) SetAwake(true);

            if ((opFlags & OperationFlag.Awake) == OperationFlag.Awake)
                this.force += force;
        }

        public void ApplyTorque(float torque, bool wake)
        {
            if (type != BodyType.DynamicBody) return;
            if (wake && (opFlags & OperationFlag.Awake) == 0) SetAwake(true);

            if ((opFlags & OperationFlag.Awake) == OperationFlag.Awake)
                this.torque += torque;
        }

        public void ApplyLinearImpulse(Vector2 impulse, Vector2 point, bool wake)
        {
            if (type != BodyType.DynamicBody) return;
            if (wake && (opFlags & OperationFlag.Awake) == 0) SetAwake(true);

            if ((opFlags & OperationFlag.Awake) == OperationFlag.Awake)
            {
                linearVelocity += invMass * impulse;
                angularVelocity += 
                        invRotationalInertia 
                    *   Calculus.Cross(point - transform.position, impulse);
            }
        }

        public void ApplyLinearImpulseToCenter(Vector2 impulse, bool wake)
        {
            if (type != BodyType.DynamicBody) return;
            if (wake && (opFlags & OperationFlag.Awake) == 0) SetAwake(true);

            if ((opFlags & OperationFlag.Awake) == OperationFlag.Awake)
                linearVelocity += invMass * impulse;
        }

        public void ApplyAngularImpulse(float impulse, bool wake)
        {
            if (type != BodyType.DynamicBody) return;
            if (wake && (opFlags & OperationFlag.Awake) == 0) SetAwake(true);

            if ((opFlags & OperationFlag.Awake) == OperationFlag.Awake)
                angularVelocity += invRotationalInertia * impulse;
        }

        public void ChangeImpulse(Vector2 force, float torque)
        {
            this.force = force;
            this.torque = torque;
        }

        public void ChangeVelocity(Vector2 linearVelocity, float angularVelocity)
        {
            this.linearVelocity = linearVelocity;
            this.angularVelocity = angularVelocity;
        }
    }
}
