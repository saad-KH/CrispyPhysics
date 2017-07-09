using UnityEngine;
namespace CrispyPhysics.Internal
{
    public class ContactMomentum : IContactMomentum
    {
        public uint tick { get; private set; }
        public Manifold manifold { get; private set; }
        public float tangentSpeed { get; private set; }
        public bool isTouching { get; private set; }
        public Vector2 firstBodyPosition { get; private set; }
        public Vector2 secondBodyPosition { get; private set; }
        public Vector2 point { get { return (manifold != null) ? manifold.point : Vector2.zero;} }
        public Vector2 normal { get { return (manifold != null) ? manifold.normal : Vector2.zero; } }

        public ContactMomentum(uint tick)
        {
            this.tick = tick;
            manifold = null;
            tangentSpeed = 0f;
            isTouching = false;
            firstBodyPosition = Vector2.zero;
            secondBodyPosition = Vector2.zero;
        }

        public ContactMomentum(uint tick, 
            Manifold manifold, float tangentSpeed,
            bool isTouching, 
            Vector2 firstBodyPosition, Vector2 secondBodyPosition)
        {
            this.tick = tick;
            this.manifold = manifold;
            this.tangentSpeed = tangentSpeed;
            this.isTouching = isTouching;
            this.firstBodyPosition = firstBodyPosition;
            this.secondBodyPosition = secondBodyPosition;
        }

        public ContactMomentum(uint tick, ContactMomentum source)
        {
            this.tick = tick;
            manifold = source.manifold;
            tangentSpeed = source.tangentSpeed;
            isTouching = source.isTouching;
            firstBodyPosition = source.firstBodyPosition;
            secondBodyPosition = source.secondBodyPosition;
        }

        public void Change(Manifold manifold, float tangentSpeed, bool isTouching, 
            Vector2 firstBodyPosition, Vector2 secondBodyPosition)
        {
            this.manifold = manifold;
            this.tangentSpeed = tangentSpeed;
            this.isTouching = isTouching;
            this.firstBodyPosition = firstBodyPosition;
            this.secondBodyPosition = secondBodyPosition;
        }

        public bool Same(ContactMomentum other, float tolerance = 0)
        {
            if (    isTouching != other.isTouching
               ||   !Calculus.Approximately(firstBodyPosition, other.firstBodyPosition, tolerance)
               ||   !Calculus.Approximately(secondBodyPosition, other.secondBodyPosition, tolerance)
               ||   !Calculus.Approximately(tangentSpeed, other.tangentSpeed, tolerance))
                return false;

            if (manifold == null && other.manifold == null)
                return true;

            if (manifold != null)
                return manifold.Same(other.manifold, tolerance);

            return true;
        }

    }
}