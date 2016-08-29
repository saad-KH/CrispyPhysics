using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics.Internal
{
    #region Interface
    public interface IInternalBody : IBody
    {
        #region Events
        event BodyEventHandler FuturCleared;
        #endregion

        #region Nature
        float invMass { get; }
        #endregion

        #region Track
        IMomentum past { get; }
        IMomentum current { get; }
        IInternalMomentum futur { get; }
        bool islandBound { get; set; }

        void Step(uint steps = 1);
        void Foresee(uint steps = 1);
        void RollBack(uint toTick);
        bool IsForeseen();
        void ForgetPast(uint fromTick);
        void ClearFutur();
        #endregion
    }
    #endregion
}