using NUnit.Framework;
using UnityEngine;


namespace CrispyPhysics
{
    using Internal;

    [TestFixture]
    public class CollisionTests
    {

        [Test]
        public void usingRaycastInput()
        {
            RayCastInput input = new RayCastInput(Vector2.zero, Vector2.one, 0.5f);

            Assert.That(input.origin, OwnNUnit.Is.EqualTo(Vector2.zero).Within(0.001f));
            Assert.That(input.target, OwnNUnit.Is.EqualTo(Vector2.one).Within(0.001f));
            Assert.That(input.maxFraction, Is.EqualTo(0.5f).Within(0.001f));
        }

        [Test]
        public void usingRaycastOutput()
        {
            RayCastOuput output = new RayCastOuput(Vector2.one, 0.5f);

            Assert.That(output.normal, OwnNUnit.Is.EqualTo(Vector2.one).Within(0.001f));
            Assert.That(output.fraction, Is.EqualTo(0.5f).Within(0.001f));
        }

        [Test]
        public void usingAABB()
        {
            AABB aabb = new AABB(
                new Vector2(0.5f, 1.5f),
                new Vector2(1f, 1f));

            Assert.That(
                aabb.lowerBound, 
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 1f)).Within(0.001f));
            Assert.That(
                aabb.upperBound, 
                OwnNUnit.Is.EqualTo(new Vector2(1f, 1.5f)).Within(0.001f));

            Assert.That(
                aabb.GetCenter(), 
                OwnNUnit.Is.EqualTo(new Vector2(0.75f, 1.25f)).Within(0.001f));

            Assert.That(
                aabb.GetExtents(),
                OwnNUnit.Is.EqualTo(new Vector2(0.25f, 0.25f)).Within(0.001f));

            Assert.That(
                aabb.GetPerimeter(),
                Is.EqualTo(2f).Within(0.001f));


            aabb.Combine(new AABB(
                new Vector2(1f, 1f),
                new Vector2(1.5f, 0.5f)));

            Assert.That(
                aabb.lowerBound,
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 0.5f)).Within(0.001f));
            Assert.That(
                aabb.upperBound,
                OwnNUnit.Is.EqualTo(new Vector2(1.5f, 1.5f)).Within(0.001f));


            aabb.Combine(
                new AABB(new Vector2(0f, 0f), new Vector2(-1f, -1f)),
                new AABB(new Vector2(1f, 1f), new Vector2(0f, 0f)));

            Assert.That(
                aabb.lowerBound,
                OwnNUnit.Is.EqualTo(new Vector2(-1f, -1f)).Within(0.001f));
            Assert.That(
                aabb.upperBound,
                OwnNUnit.Is.EqualTo(new Vector2(1f, 1f)).Within(0.001f));

            Assert.That(
                aabb.Contains(new AABB(new Vector2(1f, 1f), new Vector2(0f, 0f))));


            RayCastInput input = 
                new RayCastInput(new Vector2(-2f, -2f), new Vector2(2f, 2f), 1f);

            RayCastOuput output = new RayCastOuput();
            bool hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal, 
                OwnNUnit.Is.EqualTo(new Vector2(-1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.25f).Within(0.001f));


            input = new RayCastInput(new Vector2(2f, 2f), new Vector2(-2f, -2f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.25f).Within(0.001f));


            input = new RayCastInput(new Vector2(1f, 1f), new Vector2(-1f, -1f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0f).Within(0.001f));


            input = new RayCastInput(new Vector2(0f, -2f), new Vector2(0f, 2f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(0f, -1f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.25f).Within(0.001f));


            input = new RayCastInput(new Vector2(0.5f, 1.5f), new Vector2(1.5f, 0.5f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(0f, 1f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.5f).Within(0.001f));


            input = new RayCastInput(new Vector2(-2f, -2f), new Vector2(0.5f, 0.5f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(-1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.4f).Within(0.001f));


            input = new RayCastInput(new Vector2(0.5f, 1.5f), new Vector2(1.51f, 0.5f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit == false);


            input = new RayCastInput(new Vector2(0.5f, 0.5f), new Vector2(1f, 1f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit == false);


            input = new RayCastInput(new Vector2(0.5f, 0.5f), new Vector2(1.5f, 1.5f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit == false);


        }
    }
}
