using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region Momentum Definition
    public class Momentum : IMomentum
    {
        public uint tick { get; private set; }
        public float tickDt { get; private set; }
        public Vector2 force { get; private set; }
        public float torque { get; private set; }
        public Vector2 linearVelocity { get; private set; }
        public float angularVelocity { get; private set; }
        public Transformation transform { get; private set; }

        public Vector2 position { get { return transform.position; } }
        public float angle { get { return transform.rotation.GetAngle(); } }

        public bool enduringContact { get; private set; }

        public Momentum(
            uint tick,
            float tickDt,
            Vector2 force, float torque,
            Vector2 linearVelocity, float angularVelocity,
            Vector2 position, float angle,
            bool enduringContact)
        {
            this.tick = tick;
            this.tickDt = tickDt;
            this.force = force;
            this.torque = torque;
            this.linearVelocity = linearVelocity;
            this.angularVelocity = angularVelocity;
            transform = new Transformation(position, new Rotation(angle));
            this.enduringContact = enduringContact;
        }

        public Momentum(uint tick, Momentum source)
        {
            this.tick = tick;
            tickDt = source.tickDt;
            force = source.force;
            torque = source.torque;
            linearVelocity = source.linearVelocity;
            angularVelocity = source.angularVelocity;
            transform = new Transformation(source.position, new Rotation(source.angle));
            enduringContact = source.enduringContact;
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

        public void ChangeSituation(Vector2 position, float angle)
        {
            transform = new Transformation(position, new Rotation(angle));
        }

        public void changeEnduringContactState(bool enduringContact)
        {
            this.enduringContact = enduringContact;
        }

        public void ChangeTickDt(float tickDt)
        {
            this.tickDt = tickDt;
        }

        public bool Same(Momentum other, float tolerance = 0)
        {
            if  (   !Calculus.Approximately(tickDt, other.tickDt, tolerance)
                ||  !Calculus.Approximately(force, other.force, tolerance)
                ||  !Calculus.Approximately(torque, other.torque, tolerance)
                ||  !Calculus.Approximately(linearVelocity, other.linearVelocity, tolerance)
                ||  !Calculus.Approximately(angularVelocity, other.angularVelocity, tolerance)
                ||  !Calculus.Approximately(position, other.position, tolerance)
                ||  !Calculus.Approximately(angle, other.angle, tolerance)
                ||  enduringContact != other.enduringContact)
                return false;

            return true;
        }

    }
    #endregion
}