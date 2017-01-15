using UnityEngine;
using System;

namespace CrispyPhysics.Internal
{
    #region Polygon and Circle Contact Definition
    public class PolygonAndCircleContact : Contact
    {
        #region Constructors
        public PolygonAndCircleContact(uint tick, Body bodyA, Body bodyB) : base(tick, bodyA, bodyB)
        {
            if (!(bodyA.shape is PolygonShape))
                throw new ArgumentException("Shape of bodyA should be of PolmygonShape type");
            if (!(bodyB.shape is CircleShape))
                throw new ArgumentException("Shape of bodyB should be of CircleShape type");
        }
        #endregion

        override public Manifold Evaluate(Transformation transformA, Transformation transformB)
        {
            Debug.Assert(bodyA.shape is PolygonShape && bodyB.shape is CircleShape);
            return Collision.CollidePolygionAndCircle(
                (PolygonShape) bodyA.shape, transformA,
                (CircleShape) bodyB.shape, transformB);
        }
    }
    #endregion
}