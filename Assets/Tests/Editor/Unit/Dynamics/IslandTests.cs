using NUnit.Framework;
using NSubstitute;
using UnityEngine;
using System;

namespace CrispyPhysics
{
    using Internal;
    [TestFixture]
    public class IslandTests
    {
        [Test]
        public void SettingUpIsland()
        {
            Assert.Throws<ArgumentOutOfRangeException>(
                delegate { new Island(0, 0); });

            IShape shape1 = ShapeFactory.CreateCircle(1f);
            IShape shape2 = ShapeFactory.CreateCircle(1f);

            Body body1 = new Body(0, 0, Vector2.zero, 0f, BodyType.Dynamic, shape1);
            Body body2 = new Body(1, 0, Vector2.zero, 0f, BodyType.Dynamic, shape2);

            Contact contact1 = new CircleContact(0, body1, body2);
            Contact contact2 = new CircleContact(0, body1, body2);

            Island island = new Island();

            island.Add(body1);
            Assert.Throws<InvalidOperationException>(() => island.Add(body2));

            island.Add(contact1);
            Assert.Throws<InvalidOperationException>(() => island.Add(contact2));

            island.Clear();
            Assert.DoesNotThrow(() => island.Add(body2));
            Assert.DoesNotThrow(() => island.Add(contact2));
        }

        [Test]
        public void SolvingBody()
        {
            Island island = new Island(2, 1);

            IShape circleShape = ShapeFactory.CreateCircle(1f);
            Body body = new Body(
                0, 0, 
                Vector2.zero, 0f, 
                BodyType.Dynamic, circleShape,
                1f, 0.2f, 0.2f, 1f);
            body.ChangeImpulse(Vector2.one, 1f);
            island.Add(body);

            TimeStep step = new TimeStep(
                1f, 1f, 0f,
                new Vector2(0f, -9.8f), 8, 3,
                100, 360);

            
            island.Solve(step);
            Assert.That(body.futur.tick == 0);
            Assert.That(
                body.futur.position,
                OwnNUnit.Is.EqualTo(new Vector2(0.833f, -7.333f)).Within(0.001f));
            Assert.That(body.futur.angle, Is.EqualTo(0.833f).Within(0.001f));
            Assert.That(
                body.futur.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(0.833f, -7.333f)).Within(0.001f));
            Assert.That(body.futur.angularVelocity, Is.EqualTo(0.833f).Within(0.001f));

            body.ChangeSituation(Vector2.up, 0f);

            IShape edgeShape = ShapeFactory.CreateEdge(Vector2.left, Vector2.right);
            Body colliderBody = new Body(
                1, 0,
                Vector2.zero, 0f,
                BodyType.Static, edgeShape);

            island.Add(colliderBody);

            Contact contact = ContactFactory.CreateContact(0, body, colliderBody);
            Manifold mf = contact.Evaluate(contact.bodyA.futur.transform, contact.bodyB.futur.transform);
            contact.futur.Change(mf, 0f, true);

            island.Add(contact);

            island.Solve(step);

            Assert.That(body.futur.tick == 0);
            Assert.That(
                body.futur.position,
                OwnNUnit.Is.EqualTo(new Vector2(1.527f, 14.444f)).Within(0.001f));
            Assert.That(body.futur.angle, Is.EqualTo(1.527f).Within(0.001f));
            Assert.That(
                body.futur.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(1.527f, 13.444f)).Within(0.001f));
            Assert.That(body.futur.angularVelocity, Is.EqualTo(1.527f).Within(0.001f));

            Assert.That(colliderBody.futur.tick == 0);
            Assert.That(
                colliderBody.futur.position,
                OwnNUnit.Is.EqualTo(Vector2.zero).Within(0.001f));
            Assert.That(colliderBody.futur.angle, Is.EqualTo(0f).Within(0.001f));
            Assert.That(
                colliderBody.futur.linearVelocity,
                OwnNUnit.Is.EqualTo(Vector2.zero).Within(0.001f));
            Assert.That(colliderBody.futur.angularVelocity, Is.EqualTo(0f).Within(0.001f));
        }
    }
}