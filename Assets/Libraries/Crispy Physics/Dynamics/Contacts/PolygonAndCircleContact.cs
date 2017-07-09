using UnityEngine;
using System;

namespace CrispyPhysics.Internal
{
    #region Polygon and Circle Contact Definition
    public class PolygonAndCircleContact : Contact
    {
        #region Constructors
        public PolygonAndCircleContact(uint tick, Body firstBody, Body secondBody) 
            : base(tick, firstBody, secondBody)
        {
            if (!(internalFirstBody.shape is PolygonShape))
                throw new ArgumentException("Shape of internalFirstBody should be of PolmygonShape type");
            if (!(internalSecondBody.shape is CircleShape))
                throw new ArgumentException("Shape of internalSecondBody should be of CircleShape type");
        }
        #endregion

        override public Manifold Evaluate(Transformation transformA, Transformation transformB)
        {
            Debug.Assert(internalFirstBody.shape is PolygonShape && internalSecondBody.shape is CircleShape);
            return Collision.CollidePolygionAndCircle(
                (PolygonShape) internalFirstBody.shape, transformA,
                (CircleShape) internalSecondBody.shape, transformB);
        }
    }
    #endregion
}