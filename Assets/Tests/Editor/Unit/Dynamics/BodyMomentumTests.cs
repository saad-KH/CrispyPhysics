using NUnit.Framework;
using NSubstitute;
using UnityEngine;

namespace CrispyPhysics
{
    using Internal;
    [TestFixture]
    public class BodyMomenntumTests
    {
        public Momentum CreateBodyMomentum()
        {
            return new Momentum(
                 0,
                 Vector2.zero, 0f,
                 Vector2.zero, 0f,
                 Vector2.zero, 0f);
        }

        [Test]
        public void CreatingBodyMomentums()
        {
            Momentum bodyMomentum = CreateBodyMomentum();

            Assert.That(bodyMomentum.tick, Is.EqualTo(0f));
            Assert.That(bodyMomentum.force, Is.EqualTo(Vector2.zero));
            Assert.That(bodyMomentum.torque, Is.EqualTo(0f));
            Assert.That(bodyMomentum.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(bodyMomentum.angularVelocity, Is.EqualTo(0f));
            Assert.That(bodyMomentum.position, Is.EqualTo(Vector2.zero));
            Assert.That(bodyMomentum.angle, Is.EqualTo(0f));

            Momentum newBodyMomentum = new Momentum(1, bodyMomentum);

            Assert.That(newBodyMomentum.tick, Is.EqualTo(1f));
            Assert.That(newBodyMomentum.force, Is.EqualTo(bodyMomentum.force));
            Assert.That(newBodyMomentum.torque, Is.EqualTo(bodyMomentum.torque));
            Assert.That(newBodyMomentum.linearVelocity, Is.EqualTo(bodyMomentum.linearVelocity));
            Assert.That(newBodyMomentum.angularVelocity, Is.EqualTo(bodyMomentum.angularVelocity));
            Assert.That(newBodyMomentum.position, Is.EqualTo(bodyMomentum.position));
            Assert.That(newBodyMomentum.angle, Is.EqualTo(bodyMomentum.angle));
        }

        
        [Test]
        public void ChangingOperativeValues()
        {
            Momentum bodyMomentum = CreateBodyMomentum();
            bodyMomentum.ChangeSituation(Vector2.one, Mathf.PI/4f);
            Assert.That(bodyMomentum.position, Is.EqualTo(Vector2.one));
            Assert.That(bodyMomentum.angle, Is.EqualTo(Mathf.PI / 4f).Within(0.001f));

            bodyMomentum.ChangeSituation(Vector2.zero, 0f);
            Assert.That(bodyMomentum.position, Is.EqualTo(Vector2.zero));
            Assert.That(bodyMomentum.angle, Is.EqualTo(0f));

            bodyMomentum.ChangeVelocity(Vector2.one, Mathf.PI / 4f);
            Assert.That(bodyMomentum.linearVelocity, Is.EqualTo(Vector2.one));
            Assert.That(bodyMomentum.angularVelocity, Is.EqualTo(Mathf.PI / 4f).Within(0.001f));

            bodyMomentum.ChangeVelocity(Vector2.zero, 0f);
            Assert.That(bodyMomentum.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(bodyMomentum.angularVelocity, Is.EqualTo(0f));

            bodyMomentum.ChangeImpulse(Vector2.one, Mathf.PI / 4f);
            Assert.That(bodyMomentum.force, Is.EqualTo(Vector2.one));
            Assert.That(bodyMomentum.torque,Is.EqualTo(Mathf.PI / 4f).Within(0.001f));

            bodyMomentum.ChangeImpulse(Vector2.zero, 0f);
            Assert.That(bodyMomentum.force, Is.EqualTo(Vector2.zero));
            Assert.That(bodyMomentum.torque, Is.EqualTo(0f));
        }

        [Test]
        public void ComparingMomentums()
        {
            Momentum bodyMomentum = CreateBodyMomentum();
            bodyMomentum.ChangeSituation(Vector2.one, Mathf.PI / 4f);
            bodyMomentum.ChangeVelocity(Vector2.one, Mathf.PI / 4f);
            bodyMomentum.ChangeImpulse(Vector2.one, Mathf.PI / 4f);

            Momentum sameMomentum = new Momentum(1, bodyMomentum);
            Assert.That(sameMomentum.Same(bodyMomentum));

            bodyMomentum.ChangeSituation(Vector2.down, Mathf.PI);
            Assert.That(sameMomentum.Same(bodyMomentum) == false);

        }

        /*
        [Test]
        public void TransposingValues()
        {
            BodyMomentum bodyMomentum = CreateBodyMomentum();
            bodyMomentum.ChangeSituation(Vector2.one, Mathf.PI/4f);
            Assert.That(
                bodyMomentum.GetWorldPoint(Vector2.one),
                OwnNUnit.Is.EqualTo(new Vector2(1f, 2.414f)).Within(0.001f));

            Assert.That(
                bodyMomentum.GetWorldVector(Vector2.one),
                OwnNUnit.Is.EqualTo(new Vector2(0f, 1.414f)).Within(0.001f));

            Assert.That(
                bodyMomentum.GetLocalPoint(Vector2.zero),
                OwnNUnit.Is.EqualTo(new Vector2(-1.414f, 0f)).Within(0.001f));

            Assert.That(
                bodyMomentum.GetLocalVector(Vector2.one),
                OwnNUnit.Is.EqualTo(new Vector2(1.414f, 0f)).Within(0.001f));
       }*/
    }
}
