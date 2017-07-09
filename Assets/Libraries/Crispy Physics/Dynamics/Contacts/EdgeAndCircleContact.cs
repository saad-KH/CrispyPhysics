using UnityEngine;
using System;

namespace CrispyPhysics.Internal
{
    #region Edge and Circle Contact Definition
    public class EdgeAndCircleContact : Contact
    {
        #region Constructors
        public EdgeAndCircleContact(uint tick, Body firstBody, Body secondBody) 
            : base(tick, firstBody, secondBody)
        {
            if (!(internalFirstBody.shape is EdgeShape))
                throw new ArgumentException("Shape of internalFirstBody should be of EdgeShape type");
            if (!(internalSecondBody.shape is CircleShape))
                throw new ArgumentException("Shape of internalSecondBody should be of CircleShape type");
        }
        #endregion

        override public Manifold Evaluate(Transformation transformA, Transformation transformB)
        {
            Debug.Assert(internalFirstBody.shape is EdgeShape && internalSecondBody.shape is CircleShape);
            return Collision.CollideEdgeAndCircle(
                (EdgeShape)internalFirstBody.shape, transformA,
                (CircleShape)internalSecondBody.shape, transformB);
        }
    }
    #endregion
}