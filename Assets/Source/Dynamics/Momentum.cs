using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region Momentum Definition
    public class Momentum : IInternalMomentum
    {
        public uint tick { get; private set; }
        public Vector2 force { get; private set; }
        public float torque { get; private set; }
        public Vector2 linearVelocity { get; private set; }
        public float angularVelocity { get; private set; }
        public Transformation transform { get; private set; }

        public Vector2 position { get { return transform.position; } }
        public float angle { get { return transform.rotation.GetAngle(); } }

 

        public Momentum(
            uint tick,
            Vector2 force, float torque,
            Vector2 linearVelocity, float angularVelocity,
            Vector2 position, float angle)
        {
            this.tick = tick;
            this.force = force;
            this.torque = torque;
            this.linearVelocity = linearVelocity;
            this.angularVelocity = angularVelocity;
            transform = new Transformation(position, new Rotation(angle));
        }

        public Momentum(uint tick, IInternalMomentum source)
        {
            this.tick = tick;
            force = source.force;
            torque = source.torque;
            linearVelocity = source.linearVelocity;
            angularVelocity = source.angularVelocity;
            transform = new Transformation(source.position, new Rotation(source.angle));
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
            transform.Set(position, angle);
            //Debug.Assert(false, "Broad Phase");
        }

        public bool Same(IInternalMomentum other, float tolerance = 0)
        {
            if  (   !Calculus.Approximately(force, other.force, tolerance)
                ||  !Calculus.Approximately(torque, other.torque, tolerance)
                ||  !Calculus.Approximately(linearVelocity, other.linearVelocity, tolerance)
                ||  !Calculus.Approximately(angularVelocity, other.angularVelocity, tolerance)
                ||  !Calculus.Approximately(position, other.position, tolerance)
                ||  !Calculus.Approximately(angle, other.angle, tolerance))
                return false;

            return true;
        }

    }
    #endregion
}