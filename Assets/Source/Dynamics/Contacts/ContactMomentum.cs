using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region Contact Momentum Definition
    public class ContactMomentum
    {
        public uint tick { get; private set; }
        public Manifold manifold { get; private set; }
        public bool isTouching { get; private set; }


        public ContactMomentum(uint tick, Manifold manifold)
        {
            this.tick = tick;
            this.manifold = manifold;
        }

        public ContactMomentum(uint tick, ContactMomentum source)
        {
            this.tick = tick;
            manifold = source.manifold;
        }

        public void Change(Manifold manifold)
        {
            this.manifold = manifold;
        }

        public bool Same(ContactMomentum other, float tolerance = 0)
        {
            return manifold.Same(other.manifold, tolerance);
        }

    }
    #endregion
}