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

        void Step(float dt);
        void StepBack(float dt);
        void Foresee(float dt);
        bool IsForeseen();
        void ForgetPast(float past = 0f);
        void ClearFutur();
        #endregion
    }
    #endregion
}