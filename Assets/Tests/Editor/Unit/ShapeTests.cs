using NUnit.Framework;
using UnityEngine;


namespace CrispyPhysics
{
    using Internal;
    using System;
    [TestFixture]
    public class ShapeTests
    {

        [Test]
        public void usingCircleShape()
        {
            CircleShape circle = new CircleShape(Vector2.one, 0.5f);

            Assert.That(circle.type == ShapeType.Circle);
            Assert.That(circle.position, OwnNUnit.Is.EqualTo(Vector2.one).Within(0.001f));
            Assert.That(circle.radius, Is.EqualTo(0.5f).Within(0.001f));


            bool inside = circle.TestPoint(
                new Transformation(Vector2.zero, new Rotation(0f)),
                Vector2.one);

            Assert.That(inside);


            inside = circle.TestPoint(
                new Transformation(Vector2.one, new Rotation(0f)),
                Vector2.one);

            Assert.That(inside == false);


            inside = circle.TestPoint(
                new Transformation(Vector2.zero, new Rotation(Mathf.PI)),
                Vector2.one);

            Assert.That(inside == false);


            inside = circle.TestPoint(
                new Transformation(Vector2.one * 0.1f, new Rotation(0.1f)),
                Vector2.one);

            Assert.That(inside);


            AABB aabb = circle.computeAABB(new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(
                aabb.lowerBound,
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 0.5f)).Within(0.001f));
            Assert.That(
                aabb.upperBound,
                OwnNUnit.Is.EqualTo(new Vector2(1.5f, 1.5f)).Within(0.001f));


            aabb = 
                circle.computeAABB(new Transformation(Vector2.one, new Rotation(Mathf.PI/2)));

            Assert.That(
                aabb.lowerBound,
                OwnNUnit.Is.EqualTo(new Vector2(-0.5f, 1.5f)).Within(0.001f));
            Assert.That(
                aabb.upperBound,
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 2.5f)).Within(0.001f));


            MassData massData = circle.computeMassData(5f);

            Assert.That(massData.mass, Is.EqualTo(5f).Within(0.001f));
            Assert.That(massData.center, Is.EqualTo(circle.position));
            Assert.That(massData.rotationalGravity, Is.EqualTo(10.625f).Within(0.001f));


            RayCastInput input =
                new RayCastInput(new Vector2(-2f, -2f), new Vector2(2f, 2f), 1f);

            RayCastOuput output = new RayCastOuput();
            bool hit = circle.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(-0.707f, -0.707f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.661f).Within(0.001f));

            input =
                new RayCastInput(new Vector2(2f, 2f), new Vector2(-2f, -2f), 1f);

            hit = circle.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(0.707f, 0.707f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.161f).Within(0.001f));


            input =
                new RayCastInput(new Vector2(1.5f, 0f), new Vector2(1.5f, 2f), 1f);

            hit = circle.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.5f).Within(0.001f));


            input =
                new RayCastInput(new Vector2(1.51f, 0f), new Vector2(1.5f, 2f), 1f);

            hit = circle.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(hit == false);

            input =
                new RayCastInput(new Vector2(2f, 2f), new Vector2(-2f, -2f), 1f);

            hit = circle.RayCast(ref output, input,
                new Transformation(Vector2.up, new Rotation(0)));

            Assert.That(hit == false);

            input =
                new RayCastInput(new Vector2(2f, 2f), new Vector2(-2f, -2f), 1f);

            hit = circle.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(Mathf.PI/2f)));

            Assert.That(hit == false);

            input =
                new RayCastInput(new Vector2(1.51f, 0f), new Vector2(1.51f, 2f), 1f);

            hit = circle.RayCast(ref output, input,
                new Transformation(Vector2.right * 0.01f, new Rotation(0f)));

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.5f).Within(0.001f));
        }


        [Test]
        public void usingEdgeShape()
        {
            EdgeShape edge = new EdgeShape(Vector2.one * -1f, Vector2.one);

            Assert.That(edge.type == ShapeType.Edge);
            Assert.That(edge.radius, Is.EqualTo(Constants.polygonRadius).Within(0.001f));

            Assert.That(edge.vertex1, OwnNUnit.Is.EqualTo(Vector2.one * -1f).Within(0.001f));
            Assert.That(edge.vertex2, OwnNUnit.Is.EqualTo(Vector2.one).Within(0.001f));

            Assert.That(edge.hasVertex0 == false);
            Assert.That(edge.hasVertex3 == false);

            bool inside = edge.TestPoint(
                new Transformation(Vector2.zero, new Rotation(0f)),
                Vector2.up);

            Assert.That(inside == false);


            AABB aabb = edge.computeAABB(new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(
                aabb.lowerBound,
                OwnNUnit.Is.EqualTo(new Vector2(
                    -1f - Constants.polygonRadius, 
                    -1f - Constants.polygonRadius))
                    .Within(0.001f));
            Assert.That(
                aabb.upperBound,
                OwnNUnit.Is.EqualTo(new Vector2(
                    1f + Constants.polygonRadius,
                    1f + Constants.polygonRadius))
                    .Within(0.001f));

           
            aabb =
                edge.computeAABB(new Transformation(Vector2.one, new Rotation(Mathf.PI / 2)));

            Assert.That(
                aabb.lowerBound,
                OwnNUnit.Is.EqualTo(new Vector2(
                    0f - Constants.polygonRadius,
                    0f - Constants.polygonRadius))
                    .Within(0.001f));
            Assert.That(
                aabb.upperBound,
                OwnNUnit.Is.EqualTo(new Vector2(
                    2f + Constants.polygonRadius,
                    2f + Constants.polygonRadius))
                    .Within(0.001f));

            
            MassData massData = edge.computeMassData(5f);

            Assert.That(massData.mass, Is.EqualTo(0f).Within(0.001f));
            Assert.That(massData.center, OwnNUnit.Is.EqualTo(Vector2.zero).Within(0.001f));
            Assert.That(massData.rotationalGravity, Is.EqualTo(0f).Within(0.001f));

            
            RayCastInput input =
                new RayCastInput(new Vector2(-2f, -2f), new Vector2(2f, 2f), 1f);

            RayCastOuput output = new RayCastOuput();
            bool hit = edge.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(hit == false);


            input =
                new RayCastInput(new Vector2(2f, -2f), new Vector2(-2f, 2f), 1f);

            output = new RayCastOuput();
            hit = edge.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(0.707f, -0.707f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.5f).Within(0.001f));


            input =
                new RayCastInput(new Vector2(-2f, 2f), new Vector2(2f, -2f), 1f);

            output = new RayCastOuput();
            hit = edge.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(-0.707f, 0.707f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.5f).Within(0.001f));


            input =
                new RayCastInput(new Vector2(1f, 0f), new Vector2(1f, 2f), 1f);

            hit = edge.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(0.707f, -0.707f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.5f).Within(0.001f));


            input =
                new RayCastInput(new Vector2(1f, 0f), new Vector2(1f, 2f), 1f);

            hit = edge.RayCast(ref output, input,
                new Transformation(Vector2.left * 0.01f, new Rotation(0f)));

            Assert.That(hit == false);


            input =
                new RayCastInput(new Vector2(1f, 0f), new Vector2(1f, 2f), 1f);

            hit = edge.RayCast(ref output, input,
                new Transformation(Vector2.right * 0.01f, new Rotation(Mathf.PI/2)));

            Assert.That(hit == false);


            input =
                new RayCastInput(new Vector2(-2f, -2f), new Vector2(2f, 2f), 1f);

            output = new RayCastOuput();
            hit = edge.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(Mathf.PI/4f)));

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(-1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.5f).Within(0.001f));
        }


        [Test]
        public void usingPolygonShape()
        {
            PolygonShape polygon = new PolygonShape();

            Assert.That(polygon.type == ShapeType.Polygon);
            Assert.That(polygon.count,  Is.EqualTo(0));
            Assert.That(polygon.radius, Is.EqualTo(Constants.polygonRadius).Within(0.001f));


            polygon.SetAsBox(1f, 1f);
            Assert.That(polygon.count, Is.EqualTo(4));
            Assert.That(
                polygon.vertices[0],
                OwnNUnit.Is.EqualTo(new Vector2(-1f, -1f)).Within(0.001f));
            Assert.That(
                polygon.vertices[1],
                OwnNUnit.Is.EqualTo(new Vector2(1f, -1f)).Within(0.001f));
            Assert.That(
                polygon.vertices[2],
                OwnNUnit.Is.EqualTo(new Vector2(1f, 1f)).Within(0.001f));
            Assert.That(
                polygon.vertices[3],
                OwnNUnit.Is.EqualTo(new Vector2(-1f, 1f)).Within(0.001f));
            Assert.That(
                polygon.centroid,
                OwnNUnit.Is.EqualTo(Vector2.zero).Within(0.001f));

            polygon.SetAsBox(1f, 1f, Vector2.one * 0.5f, Mathf.PI/4);
            Assert.That(polygon.count, Is.EqualTo(4));
            Assert.That(
                polygon.vertices[0],
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, -0.914f)).Within(0.001f));
            Assert.That(
                polygon.vertices[1],
                OwnNUnit.Is.EqualTo(new Vector2(1.914f, 0.5f)).Within(0.001f));
            Assert.That(
                polygon.vertices[2],
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 1.914f)).Within(0.001f));
            Assert.That(
                polygon.vertices[3],
                OwnNUnit.Is.EqualTo(new Vector2(-0.914f, 0.5f)).Within(0.001f));
            Assert.That(
                polygon.centroid,
                OwnNUnit.Is.EqualTo(Vector2.one * 0.5f).Within(0.001f));


            polygon.Set(
                new Vector2[] {
                    new Vector2(-1f,-1f),
                    new Vector2(1f,-1f),
                    new Vector2(1f,1f),
                    new Vector2(0f,2f),
                    new Vector2(-1f,1f),
                }, 5);

            Assert.That(polygon.count, Is.EqualTo(5));
            Assert.That(
                polygon.vertices[0],
                OwnNUnit.Is.EqualTo(new Vector2(1f, -1f)).Within(0.001f));
            Assert.That(
                polygon.vertices[1],
                OwnNUnit.Is.EqualTo(new Vector2(1f, 1f)).Within(0.001f));
            Assert.That(
                polygon.vertices[2],
                OwnNUnit.Is.EqualTo(new Vector2(0f, 2f)).Within(0.001f));
            Assert.That(
                polygon.vertices[3],
                OwnNUnit.Is.EqualTo(new Vector2(-1f, 1f)).Within(0.001f));
            Assert.That(
                polygon.vertices[4],
                OwnNUnit.Is.EqualTo(new Vector2(-1f, -1f)).Within(0.001f));
            Assert.That(
                polygon.centroid,
                OwnNUnit.Is.EqualTo(new Vector2(0f, 0.266f)).Within(0.001f));
           
            bool inside = polygon.TestPoint(
                new Transformation(Vector2.zero, new Rotation(0f)),
                Vector2.one);

            Assert.That(inside);


            inside = polygon.TestPoint(
                new Transformation(Vector2.zero, new Rotation(0f)),
                Vector2.one * 2f);

            Assert.That(inside == false);


            inside = polygon.TestPoint(
                new Transformation(Vector2.zero, new Rotation(Mathf.PI/4f)),
                Vector2.one * 0.5f);

            Assert.That(inside);

            
            AABB aabb = polygon.computeAABB(new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(
                aabb.lowerBound,
                OwnNUnit.Is.EqualTo(new Vector2(
                    -1f - Constants.polygonRadius, 
                    -1f - Constants.polygonRadius)).Within(0.001f));
            Assert.That(
                aabb.upperBound,
                OwnNUnit.Is.EqualTo(new Vector2(
                    1f + Constants.polygonRadius,
                    2f + Constants.polygonRadius)).Within(0.001f));

            
            aabb =
                polygon.computeAABB(new Transformation(Vector2.zero, new Rotation(Mathf.PI / 2)));

            Assert.That(
                aabb.lowerBound,
                OwnNUnit.Is.EqualTo(new Vector2(
                    -2f - Constants.polygonRadius, 
                    -1f - Constants.polygonRadius)).Within(0.001f));
            Assert.That(
                aabb.upperBound,
                OwnNUnit.Is.EqualTo(new Vector2(
                    1f + Constants.polygonRadius,
                    1f + Constants.polygonRadius)).Within(0.001f));

            
            MassData massData = polygon.computeMassData(5f);

            Assert.That(massData.mass, Is.EqualTo(5f).Within(0.001f));
            Assert.That(massData.center, OwnNUnit.Is.EqualTo(polygon.centroid).Within(0.001f));
            Assert.That(massData.rotationalGravity, Is.EqualTo(4.666f).Within(0.001f));

           
            RayCastInput input =
                new RayCastInput(new Vector2(-2f, -2f), new Vector2(2f, 2f), 1f);

            RayCastOuput output = new RayCastOuput();
            bool hit = polygon.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(-1, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.25f).Within(0.001f));
            
           
            input =
                new RayCastInput(new Vector2(2f, 2f), new Vector2(-2f, 2f), 1f);

            hit = polygon.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(0f)));

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(0.707f, 0.707f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.5f).Within(0.001f));

            input =
                new RayCastInput(new Vector2(2f, 2f), new Vector2(-2f, 2f), 1f);

            hit = polygon.RayCast(ref output, input,
                new Transformation(Vector2.down * 0.01f, new Rotation(0f)));

            Assert.That(hit == false);

            input =
                new RayCastInput(new Vector2(2f, 2f), new Vector2(-2f, 2f), 1f);

            hit = polygon.RayCast(ref output, input,
                new Transformation(Vector2.zero, new Rotation(Mathf.PI/2f)));

            Assert.That(hit == false);
        }
    }
}
