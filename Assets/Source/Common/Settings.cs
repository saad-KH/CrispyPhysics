using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region Constants
    public static class Constants
    {
        public const float linearSlop = 0.005f;
        public const float polygonRadius = 2f * linearSlop;
        public const int maxPolygonVertices = 8;
        public const int maxManifoldPoints = 2;
    }
    #endregion
}