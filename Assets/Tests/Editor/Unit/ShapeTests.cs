using NUnit.Framework;
using UnityEngine;


namespace CrispyPhysics
{
    using Internal;

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
    }
}
