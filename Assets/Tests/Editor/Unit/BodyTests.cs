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
           return new Body(0, Vector2.zero, 0f, BodyType.DynamicBody, null);
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
        public void ChangingStates()
        {
            Body body = CreateBody();
            //Changing Impulse
            body.ChangeImpulse(Vector2.one, 1f);

            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));

            //Changing Velocity
            body.ChangeVelocity(Vector2.up, 2f);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.up));
            Assert.That(body.angularVelocity, Is.EqualTo(2f));

            //Changing Situation
            body.ChangeSituation(Vector2.down, -1f);

            Assert.That(body.position, Is.EqualTo(Vector2.down));
            Assert.That(body.angle, Is.EqualTo(-1f));
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
            body.ChangeImpulse(Vector2.zero, 0f);
            body.ApplyForceToCenter(Vector2.one);

            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(0f));

            //Applying Torque
            body.ChangeImpulse(Vector2.zero, 0f);
            body.ApplyTorque(1f);

            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(1f));
        }
    
        [Test]
        public void ApplyingImpulses()
        {
            Body body = CreateBody();
            //Applying Impulse
            body.ChangeVelocity(Vector2.zero, 0f);
            body.ApplyLinearImpulse(Vector2.one, new Vector2(-0.5f, 0.5f));

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.one));
            Assert.That(body.angularVelocity, Is.EqualTo(0f));

            //Applying Impulse to Center
            body.ChangeVelocity(Vector2.zero, 0f);
            body.ApplyLinearImpulseToCenter(Vector2.one);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.one));
            Assert.That(body.angularVelocity, Is.EqualTo(0f));

            //Apply Angular Impulse
            body.ChangeVelocity(Vector2.zero, 0f);
            body.ApplyAngularImpulse(1f);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(body.angularVelocity, Is.EqualTo(0f));
        }

        [Test]
        public void Tracking()
        {
            Body body = CreateBody();

            body.Step();
            Assert.That(body.current.tick, Is.EqualTo(1));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
            Assert.That(body.past.tick, Is.EqualTo(0f));
            Assert.That(body.past.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.past.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen() == false);

            body.Foresee();
            Assert.That(body.current.tick, Is.EqualTo(1));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
            Assert.That(body.past.tick, Is.EqualTo(0f));
            Assert.That(body.past.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.past.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(2));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.futur.torque, Is.EqualTo(0f));

            body.Foresee();
            body.futur.ChangeImpulse(Vector2.one, 1f);
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(3));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.one));
            Assert.That(body.futur.torque, Is.EqualTo(1f));

            body.Foresee(2);
            body.futur.ChangeImpulse(Vector2.one, 1f);
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(5));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.one));
            Assert.That(body.futur.torque, Is.EqualTo(1f));

            body.Foresee();
            body.futur.ChangeImpulse(Vector2.down, -1f);
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(6));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.down));
            Assert.That(body.futur.torque, Is.EqualTo(-1f));

            body.Step(2);
            Assert.That(body.current.tick, Is.EqualTo(3));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.past.tick, Is.EqualTo(0));
            Assert.That(body.past.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.past.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen());

            body.Step();
            Assert.That(body.current.tick, Is.EqualTo(4));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen());

            body.Step();
            Assert.That(body.current.tick, Is.EqualTo(5));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen());

            body.Step();
            Assert.That(body.current.tick, Is.EqualTo(6));
            Assert.That(body.force, Is.EqualTo(Vector2.down));
            Assert.That(body.torque, Is.EqualTo(-1f));
            Assert.That(body.IsForeseen() == false);

            body.Foresee();
            body.futur.ChangeImpulse(Vector2.up, 2f);
            Assert.That(body.current.tick, Is.EqualTo(6));
            Assert.That(body.force, Is.EqualTo(Vector2.down));
            Assert.That(body.torque, Is.EqualTo(-1f));
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(7));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.up));
            Assert.That(body.futur.torque, Is.EqualTo(2f));

            body.ChangeImpulse(Vector2.zero, 0f);
            body.ApplyForceToCenter(Vector2.left);
            body.ApplyTorque(3f);
            Assert.That(body.current.tick, Is.EqualTo(6));
            Assert.That(body.force, Is.EqualTo(Vector2.left));
            Assert.That(body.torque, Is.EqualTo(3f));
            Assert.That(body.IsForeseen() == false);

            body.Step();
            Assert.That(body.current.tick, Is.EqualTo(7));
            Assert.That(body.force, Is.EqualTo(Vector2.left));
            Assert.That(body.torque, Is.EqualTo(3f));
            Assert.That(body.IsForeseen() == false);

            body.RollBack(4);
            Assert.That(body.current.tick, Is.EqualTo(4));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.past.tick, Is.EqualTo(0));
            Assert.That(body.past.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.past.torque, Is.EqualTo(0f));
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(6));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.left));
            Assert.That(body.futur.torque, Is.EqualTo(3f));

            bool wasCalled = false;
            body.FuturCleared += (sender, args) => wasCalled = true;
            body.ClearFutur();
            Assert.That(body.current.tick, Is.EqualTo(4));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen() == false);
            Assert.That(wasCalled);
    
            body.RollBack(2);
            Assert.That(body.current.tick, Is.EqualTo(2));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));
            body.RollBack(1);
            Assert.That(body.current.tick, Is.EqualTo(1));
            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));

            body.Step(2);
            body.ForgetPast(body.current.tick);
            Assert.That(body.current.tick, Is.EqualTo(3));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.past.tick, Is.EqualTo(3));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));

            body.Step();
            body.ChangeImpulse(Vector2.down, -1f);
            body.ForgetPast(body.current.tick);
            Assert.That(body.current.tick, Is.EqualTo(4));
            Assert.That(body.force, Is.EqualTo(Vector2.down));
            Assert.That(body.torque, Is.EqualTo(-1f));
            body.RollBack(0);
            Assert.That(body.current.tick, Is.EqualTo(0));
            Assert.That(body.force, Is.EqualTo(Vector2.down));
            Assert.That(body.torque, Is.EqualTo(-1f));

        }
    }
}
