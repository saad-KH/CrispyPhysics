using NUnit.Framework;
using NSubstitute;
using UnityEngine;

namespace CrispyPhysics
{
    [TestFixture]
    public class BodyTests
    {
        private IBody body;

        [SetUp]
        public void CreateBody()
        {
            body = new Body(
                BodyType.DynamicBody,
                Vector2.zero,
                0f
            );
        }

        [Test]
        public void CreatingBody()
        {
            Assert.That(body.IsFixedRotation() == false);
            Assert.That(body.IsSleepingAllowed() == false);
            Assert.That(body.IsAwake() == true);
            Assert.That(body.IsActive() == true);
            Assert.That(body.IsIslandBound() == false);

            Assert.That(body.type, Is.EqualTo(BodyType.DynamicBody));

            Assert.That(body.position, Is.EqualTo(Vector2.zero));
            Assert.That(body.angle, Is.EqualTo(0f));

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(body.angularVelocity, Is.EqualTo(0f));

            Assert.That(body.mass, Is.EqualTo(1f));
            Assert.That(body.invMass, Is.EqualTo(1f));

            Assert.That(body.linearDamping, Is.EqualTo(0f));
            Assert.That(body.angularDamping, Is.EqualTo(0f));
            Assert.That(body.gravityScale, Is.EqualTo(1f));

            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));

            Assert.That(body.sleepTime, Is.EqualTo(0f));

            Assert.That(body.GetInertia(), Is.EqualTo(0f));
        }

        [Test]
        public void SettingMass()
        {
            body.SetMass(80f);

            Assert.That(body.mass, Is.EqualTo(80f));
            Assert.That(body.invMass, Is.EqualTo(1f/80f).Within(0.001f));

            body.SetMass(0f);

            Assert.That(body.mass, Is.EqualTo(1f));
            Assert.That(body.invMass, Is.EqualTo(1f).Within(0.001f));
        }

        [Test]
        public void DefiningShape()
        {
            IShape shape = Substitute.For<IShape>();
            body.DefineShape(shape);
            body.CalculateInertia();
        }

        [Test]
        public void ShouldBeCollidingBody()
        {
            IBody dynamicBody = Substitute.For<IBody>();
            dynamicBody.type.Returns(BodyType.DynamicBody);

            Assert.That(body.ShouldCollide(dynamicBody) == true);
        }

        [Test]
        public void ChangingOperativeValues()
        {
            body.ChangeSituation(Vector2.one, Mathf.PI/4f);
            Assert.That(body.position, Is.EqualTo(Vector2.one));
            Assert.That(body.angle, Is.EqualTo(Mathf.PI / 4f).Within(0.001f));

            body.ChangeSituation(Vector2.zero, 0f);
            Assert.That(body.position, Is.EqualTo(Vector2.zero));
            Assert.That(body.angle, Is.EqualTo(0f));

            body.SetAwake(false);
            body.ChangeVelocity(Vector2.one, Mathf.PI / 4f);
            Assert.That(body.IsAwake() == true);
            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.one));
            Assert.That(body.angularVelocity, Is.EqualTo(Mathf.PI / 4f).Within(0.001f));

            body.SetAwake(false);
            body.ChangeVelocity(Vector2.zero, 0f);
            Assert.That(body.IsAwake() == false);
            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(body.angularVelocity, Is.EqualTo(0f));

            body.ChangeImpulse(Vector2.one, Mathf.PI / 4f);
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque,Is.EqualTo(Mathf.PI / 4f).Within(0.001f));

            body.ChangeImpulse(Vector2.zero, 0f);
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
        }

        [Test]
        public void ChangingStates()
        {
            body.SetActive(false);
            Assert.That(body.IsActive() == false);
            body.SetActive(true);
            Assert.That(body.IsActive() == true);

            body.SetAwake(false);
            Assert.That(body.IsAwake() == false);
            body.SetAwake(true);
            Assert.That(body.IsAwake() == true);

            body.SetSleepingAllowed(true);
            Assert.That(body.IsSleepingAllowed() == true);
            body.SetSleepingAllowed(false);
            Assert.That(body.IsSleepingAllowed() == false);

            body.SetFixedRotation(true);
            Assert.That(body.IsFixedRotation() == true);
            body.SetFixedRotation(false);
            Assert.That(body.IsFixedRotation() == false);

            body.SetIslandBound(true);
            Assert.That(body.IsIslandBound() == true);
            body.SetIslandBound(false);
            Assert.That(body.IsIslandBound() == false);
        }

        [Test]
        public void TransposingValues()
        {
            body.ChangeSituation(Vector2.one, Mathf.PI/4f);
            Assert.That(
                body.GetWorldPoint(Vector2.one),
                OwnNUnit.Is.EqualTo(new Vector2(1f, 2.414f)).Within(0.001f));

            Assert.That(
                body.GetWorldVector(Vector2.one),
                OwnNUnit.Is.EqualTo(new Vector2(0f, 1.414f)).Within(0.001f));

            Assert.That(
                body.GetLocalPoint(Vector2.zero),
                OwnNUnit.Is.EqualTo(new Vector2(-1.414f, 0f)).Within(0.001f));

            Assert.That(
                body.GetLocalVector(Vector2.one),
                OwnNUnit.Is.EqualTo(new Vector2(1.414f, 0f)).Within(0.001f));

            body.ChangeSituation(Vector2.zero, 0f);
            body.ChangeVelocity(Vector2.zero, 0f);
        }

        [Test]
        public void ApplyingForces()
        {
            //Applying force
            body.ChangeImpulse(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyForce(Vector2.one, new Vector2(-0.5f,0.5f), false);

            Assert.That(body.force, Is.EqualTo(Vector2.zero));

            body.ApplyForce(Vector2.one, new Vector2(-0.5f, 0.5f), true);

            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(-1f));

            //Applying force to Center
            body.ChangeImpulse(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyForceToCenter(Vector2.one, false);

            Assert.That(body.force, Is.EqualTo(Vector2.zero));

            body.ApplyForceToCenter(Vector2.one, true);

            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(0f));

            //Applying Torque
            body.ChangeImpulse(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyTorque(1f, false);

            Assert.That(body.torque, Is.EqualTo(0f));

            body.ApplyTorque(1f, true);

            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(1f));
        }

        [Test]
        public void ApplyingImpulses()
        {
            //Applying Impulse
            body.ChangeVelocity(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyLinearImpulse(Vector2.one, new Vector2(-0.5f, 0.5f), false);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.zero));

            body.ApplyLinearImpulse(Vector2.one, new Vector2(-0.5f, 0.5f), true);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.one));

            Assert.That(body.angularVelocity, Is.EqualTo(0f));

            //Applying Impulse to Center
            body.ChangeVelocity(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyLinearImpulseToCenter(Vector2.one, false);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.zero));

            body.ApplyLinearImpulseToCenter(Vector2.one, true);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.one));

            Assert.That(body.angularVelocity, Is.EqualTo(0f));

            //Apply Angular Impulse
            body.ChangeVelocity(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyAngularImpulse(1f, false);

            Assert.That(body.angularVelocity, Is.EqualTo(0f));

            body.ApplyAngularImpulse(1f, true);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.zero));

            Assert.That(body.angularVelocity, Is.EqualTo(0f));
        }


    }
}
