using NUnit.Framework;
using NSubstitute;
using UnityEngine;

namespace CrispyPhysics
{
    using Internal;
    [TestFixture]
    public class BodyTests
    {
        public Body CreateBody()
        {
           return new Body(
                BodyType.DynamicBody,
                null,
                Vector2.zero,
                0f
            );
        }

        [Test]
        public void CreatingBody()
        {
            Body body = CreateBody();
            Assert.That(body.islandBound == false);

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

            Assert.That(body.GetInertia(), Is.EqualTo(0f));
        }

        [Test]
        public void ShouldBeCollidingBody()
        {
            Body body = CreateBody();
            IBody dynamicBody = Substitute.For<IBody>();
            dynamicBody.type.Returns(BodyType.DynamicBody);

            Assert.That(body.ShouldCollide(dynamicBody) == true);
        }
       
        [Test]
        public void ApplyingForces()
        {
            Body body = CreateBody();
            //Applying force
            body.ApplyForce(Vector2.one, new Vector2(-0.5f, 0.5f));

            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(-1f));

            //Applying force to Center
            body.current.ChangeImpulse(Vector2.zero, 0f);
            body.ApplyForceToCenter(Vector2.one);

            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(0f));

            //Applying Torque
            body.current.ChangeImpulse(Vector2.zero, 0f);
            body.ApplyTorque(1f);

            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(1f));
        }
    
        [Test]
        public void ApplyingImpulses()
        {
            Body body = CreateBody();
            //Applying Impulse
            body.current.ChangeVelocity(Vector2.zero, 0f);
            body.ApplyLinearImpulse(Vector2.one, new Vector2(-0.5f, 0.5f));

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.one));
            Assert.That(body.angularVelocity, Is.EqualTo(0f));

            //Applying Impulse to Center
            body.current.ChangeVelocity(Vector2.zero, 0f);
            body.ApplyLinearImpulseToCenter(Vector2.one);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.one));
            Assert.That(body.angularVelocity, Is.EqualTo(0f));

            //Apply Angular Impulse
            body.current.ChangeVelocity(Vector2.zero, 0f);
            body.ApplyAngularImpulse(1f);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(body.angularVelocity, Is.EqualTo(0f));
        }

        [Test]
        public void Tracking()
        {
            Body body = CreateBody();

            body.Step(0.2f);
            Assert.That(body.current.tick, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen() == false);

            body.Foresee(0.3f);
            Assert.That(body.current.tick, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(0.5f).Within(0.001f));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.futur.torque, Is.EqualTo(0f));

            body.Foresee(0.5f);
            body.futur.ChangeImpulse(Vector2.one, 1f);
            Assert.That(body.current.tick, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(1f).Within(0.001f));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.one));
            Assert.That(body.futur.torque, Is.EqualTo(1f));

            body.Foresee(0.5f);
            body.futur.ChangeImpulse(Vector2.one, 1f);
            Assert.That(body.current.tick, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(1.5f).Within(0.001f));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.one));
            Assert.That(body.futur.torque, Is.EqualTo(1f));

            body.Foresee(0.5f);
            body.futur.ChangeImpulse(Vector2.down, -1f);
            Assert.That(body.current.tick, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(2f).Within(0.001f));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.down));
            Assert.That(body.futur.torque, Is.EqualTo(-1f));

            body.Step(0.3f);
            Assert.That(body.current.tick, Is.EqualTo(0.5f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen());

            body.Step(0.4f);
            Assert.That(body.current.tick, Is.EqualTo(0.9f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen());

            body.Step(0.2f);
            Assert.That(body.current.tick, Is.EqualTo(1.1f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen());

            body.Step(0.6f);
            Assert.That(body.current.tick, Is.EqualTo(1.7f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen());

            body.Step(0.5f);
            Assert.That(body.current.tick, Is.EqualTo(2.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.down));
            Assert.That(body.torque, Is.EqualTo(-1f));
            Assert.That(body.IsForeseen() == false);

            body.Foresee(0.5f);
            body.futur.ChangeImpulse(Vector2.up, 2f);
            Assert.That(body.current.tick, Is.EqualTo(2.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.down));
            Assert.That(body.torque, Is.EqualTo(-1f));
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(2.7f).Within(0.001f));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.up));
            Assert.That(body.futur.torque, Is.EqualTo(2f));

            body.current.ChangeImpulse(Vector2.zero, 0f);
            body.ApplyForceToCenter(Vector2.left);
            body.ApplyTorque(3f);
            Assert.That(body.force, Is.EqualTo(Vector2.left));
            Assert.That(body.torque, Is.EqualTo(3f));
            Assert.That(body.IsForeseen() == false);

            body.Step(0.5f);
            Assert.That(body.current.tick, Is.EqualTo(2.7f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.left));
            Assert.That(body.torque, Is.EqualTo(3f));
            Assert.That(body.IsForeseen() == false);

            body.StepBack(1.5f);
            Assert.That(body.current.tick, Is.EqualTo(1.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(2.2f).Within(0.001f));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.left));
            Assert.That(body.futur.torque, Is.EqualTo(3f));

            bool wasCalled = false;
            body.FuturCleared += (sender, args) => wasCalled = true;
            body.keep(-1f, 0.9f);
            Assert.That(body.current.tick, Is.EqualTo(1.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen());
            Assert.That(wasCalled);
            Assert.That(body.futur.tick, Is.EqualTo(2f).Within(0.001f));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.down));
            Assert.That(body.futur.torque, Is.EqualTo(-1f));

            wasCalled = false;
            body.keep(-1f, 0.8f);
            Assert.That(body.current.tick, Is.EqualTo(1.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen() == true);
            Assert.That(wasCalled == false);
            Assert.That(body.futur.tick, Is.EqualTo(2f).Within(0.001f));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.down));
            Assert.That(body.futur.torque, Is.EqualTo(-1f));

            wasCalled = false;
            body.ClearFutur();
            Assert.That(body.current.tick, Is.EqualTo(1.2f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen() == false);
            Assert.That(wasCalled);

            body.keep(0.3f, -1f);
            body.StepBack(0.3f);
            Assert.That(body.current.tick, Is.EqualTo(0.9f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));

            body.Step(0.3f);
            body.keep(0.2f,-1f);
            body.StepBack(0.2f);
            Assert.That(body.current.tick, Is.EqualTo(1f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));


            body.Step(0.2f);
            body.current.ChangeImpulse(Vector2.down, -1f);
            body.ForgetPast();
            body.StepBack(0.3f);
            Assert.That(body.current.tick, Is.EqualTo(0.9f).Within(0.001f));
            Assert.That(body.force, Is.EqualTo(Vector2.down));
            Assert.That(body.torque, Is.EqualTo(-1f));

        }
    }
}
