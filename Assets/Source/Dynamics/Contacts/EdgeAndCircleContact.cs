using UnityEngine;
using System;

namespace CrispyPhysics.Internal
{
    #region Edge and Circle Contact Definition
    public class EdgeAndCircleContact : Contact
    {
        #region Constructors
        public EdgeAndCircleContact(uint tick, Body bodyA, Body bodyB) : base(tick, bodyA, bodyB)
        {
            if (!(bodyA.shape is EdgeShape))
                throw new ArgumentException("Shape of bodyA should be of EdgeShape type");
            if (!(bodyB.shape is CircleShape))
                throw new ArgumentException("Shape of bodyB should be of CircleShape type");
        }
        #endregion

        override public Manifold Evaluate(Transformation transformA, Transformation transformB)
        {
            Debug.Assert(bodyA.shape is EdgeShape && bodyB.shape is CircleShape);
            return Collision.CollideEdgeAndCircle(
                (EdgeShape)bodyA.shape, transformA,
                (CircleShape)bodyB.shape, transformB);
        }
    }
    #endregion
}