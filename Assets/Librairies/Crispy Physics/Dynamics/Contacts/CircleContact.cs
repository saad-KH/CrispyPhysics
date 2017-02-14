using UnityEngine;
using System;

namespace CrispyPhysics.Internal
{
    #region Circle Contact Definition
    public class CircleContact : Contact
    {
        #region Constructors
        public CircleContact(uint tick, Body firstBody, Body secondBody) : base(tick, firstBody, secondBody)
        {
            if (!(internalFirstBody.shape is CircleShape))
                throw new ArgumentException("Shape of internalFirstBody should be of CircleShape type");
            if (!(internalSecondBody.shape is CircleShape))
                throw new ArgumentException("Shape of internalSecondBody should be of CircleShape type");
        }
        #endregion

        override public Manifold Evaluate(Transformation transformA, Transformation transformB)
        {
            Debug.Assert(internalFirstBody.shape is CircleShape && internalSecondBody.shape is CircleShape);
            return Collision.CollideCircles(
                (CircleShape)internalFirstBody.shape, transformA,
                (CircleShape)internalSecondBody.shape, transformB);
        }
    }
    #endregion
}