using UnityEngine;
using System;
using System.Collections.Generic;

#region Factory
namespace CrispyPhysics
{
    using Internal;
    public class ShapeFactory
    {
        public static IShape CreateCircle(float radius)
        {
            return new CircleShape(Vector2.zero, radius);
        }

        public static IShape CreateEdge(
            Vector2 firstVertex, Vector2 secondVertex)
        {
            return new EdgeShape(firstVertex, secondVertex);
        }

        public static IShape CreateBox(
            float width, float height)
        {
            PolygonShape polygon = new PolygonShape();
            polygon.SetAsBox(width, height);
            return polygon;
        }
    }
}
#endregion