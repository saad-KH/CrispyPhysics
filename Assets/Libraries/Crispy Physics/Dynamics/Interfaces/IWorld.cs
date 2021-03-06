using UnityEngine;
using System.Collections.Generic;

namespace CrispyPhysics
{
    #region World Definition
    public struct WorldDefinition
    {
        public readonly float fixedStep;
        public readonly Vector2 gravity;
        public readonly uint velocityIterations;
        public readonly uint positionIterations;
        public readonly float maxTranslationSpeed;
        public readonly float maxRotationSpeed;

        public WorldDefinition(
            float fixedStep,
            Vector2 gravity, uint velocityIterations = 8, uint positionIterations = 3,
            float maxTranslationSpeed = 100f, float maxRotationSpeed = 360f)
        {
            this.fixedStep = fixedStep;
            this.gravity = gravity;
            this.velocityIterations = velocityIterations;
            this.positionIterations = positionIterations;
            this.maxTranslationSpeed = maxTranslationSpeed;
            this.maxRotationSpeed = maxRotationSpeed;
        }
    }
    #endregion

    #region Events Definition
    public delegate void IWorldHandlerDelegate(IWorld world, uint atTick);
    #endregion

    #region Interface Defintion
    public interface IWorld
    {
        #region Events
        event IWorldHandlerDelegate FuturCleared;
        event IContactHandlerDelegate ContactStartForeseen;
        event IContactHandlerDelegate ContactEndForeseen;
        event IContactHandlerDelegate ContactStarted;
        event IContactHandlerDelegate ContactEnded;
        #endregion

        #region Nature
        float fixedStep { get; }

        Vector2 gravity { get; }
        uint velocityIterations { get; }
        uint positionIterations { get; }

        float maxTranslationSpeed { get; }
        float maxRotationSpeed { get; }

        uint tick { get; }
        uint pastTick { get; }
        uint futurTick { get; }
        #endregion

        #region Body Manager
        IBody CreateBody(
            Vector2 position, float angle,
            BodyType type, IShape shape, float mass = 1f,
            float linearDamping = 0f, float angularDamping = 0f,
            float gravityScale = 1f,
            float friction = 0f, float restitution = 1f,
            bool sensor = false);

        IBody CreateBody(
            Vector2 position, float angle,
            BodyDefintion bodyDef);
        #endregion

        #region Tracker
        void Step(
            uint steps = 1,
            uint foreseeTicks = 0, uint bufferingTicks = 0,
            uint keepTicks = 0);

        void RollBack(uint toTick, uint keepTicks = 0);
        #endregion
    }
    #endregion
}