using UnityEngine;
using System;

namespace CrispyPhysics.Internal
{
    #region Circle Contact Definition
    public class CircleContact : Contact
    {
        #region Constructors
        public CircleContact(uint tick, Body bodyA, Body bodyB) : base(tick, bodyA, bodyB)
        {
            if (!(bodyA.shape is CircleShape))
                throw new ArgumentException("Shape of bodyA should be of CircleShape type");
            if (!(bodyB.shape is CircleShape))
                throw new ArgumentException("Shape of bodyB should be of CircleShape type");
        }
        #endregion

        override public Manifold Evaluate(Transformation transformA, Transformation transformB)
        {
            Debug.Assert(bodyA.shape is CircleShape && bodyB.shape is CircleShape);
            return Collision.CollideCircles(
                (CircleShape)bodyA.shape, transformA,
                (CircleShape)bodyB.shape, transformB);
        }
    }
    #endregion
}