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

            Body body1 = new Body(0, 0, Vector2.zero, 0f, BodyType.Dynamic, null);
            Body body2 = new Body(1, 0, Vector2.zero, 0f, BodyType.Dynamic, null);

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
            Island island = new Island();

            Body body = new Body(
                0, 0, 
                Vector2.zero, 0f, 
                BodyType.Dynamic, null,
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
        }
    }
}