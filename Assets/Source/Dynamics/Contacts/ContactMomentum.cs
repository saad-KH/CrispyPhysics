using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region Contact Momentum Definition
    public class ContactMomentum
    {
        public uint tick { get; private set; }
        public Manifold manifold { get; private set; }
        public float tangentSpeed { get; private set; }
        public bool isTouching { get; private set; }

        public ContactMomentum(uint tick)
        {
            this.tick = tick;
            manifold = null;
            tangentSpeed = 0f;
            isTouching = false;
        }

        public ContactMomentum(uint tick, Manifold manifold, float tangentSpeed, bool isTouching)
        {
            this.tick = tick;
            this.manifold = manifold;
            this.tangentSpeed = tangentSpeed;
            this.isTouching = isTouching;
        }

        public ContactMomentum(uint tick, ContactMomentum source)
        {
            this.tick = tick;
            manifold = source.manifold;
            tangentSpeed = source.tangentSpeed;
            isTouching = source.isTouching;
        }

        public void Change(Manifold manifold, float tangentSpeed, bool isTouching)
        {
            this.manifold = manifold;
            this.tangentSpeed = tangentSpeed;
            this.isTouching = isTouching;
        }

        public bool Same(ContactMomentum other, float tolerance = 0)
        {
            return manifold.Same(other.manifold, tolerance);
        }

    }
    #endregion
}