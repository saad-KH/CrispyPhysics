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
            IWorld world = Substitute.For<IWorld>();
            body = new Body(
                BodyType.DynamicBody,
                world,
                Vector2.zero,
                0f
            );
        }

        [Test]
        public void CreatingBody()
        {
            Assert.That(body.IsFixedRotation() == false, "Body isn't fixedly rotated");
            Assert.That(body.IsSleepingAllowed() == false, "Body shouldn't asleep");
            Assert.That(body.IsAwake() == true, "Body is awake");
            Assert.That(body.IsActive() == true, "Body is active");
            Assert.That(body.IsIslandBound() == false, "Body is bound to no island");

            Assert.That(body.type, Is.EqualTo(BodyType.DynamicBody), "Body is dynamic");
            Assert.That(body.world != null, "Body is attached to world");

            Assert.That(body.position, Is.EqualTo(Vector2.zero), "Body is rightly placed");
            Assert.That(body.angle, Is.EqualTo(0f), "Body is rightly oriented");

            Assert.That(
                body.linearVelocity, Is.EqualTo(Vector2.zero),"Body's linear Velocity is null");
            Assert.That(
                body.angularVelocity, Is.EqualTo(0f), "Body's angular velocity is null");

            Assert.That(body.mass, Is.EqualTo(1f), "Body has a 1 unit mass");
            Assert.That(body.invMass, Is.EqualTo(1f), "Body has a 1 unit inv mass");

            Assert.That(body.linearDamping, Is.EqualTo(0f), "Body has no linear dampling");
            Assert.That(body.angularDamping, Is.EqualTo(0f), "Body has no angular dampling");
            Assert.That(body.gravityScale, Is.EqualTo(1f), "Body has a full gravity Scale");

            Assert.That(body.force, Is.EqualTo(Vector2.zero), "Body has no starting force");
            Assert.That(body.torque, Is.EqualTo(0f), "Body has no starting torque");

            Assert.That(body.sleepTime, Is.EqualTo(0f), "Body's sleep time is null");

            Assert.That(body.GetInertia(), Is.EqualTo(0f), "Body's rotational inertia is null");
        }

        [Test]
        public void SettingMass()
        {
            body.SetMass(80f);

            Assert.That(body.mass, Is.EqualTo(80f), "Body has a 80 unit mass");
            Assert.That(
                body.invMass, 
                Is.EqualTo(1f/80f).Within(0.001f), 
                "Body has a 1/80 unit inv mass");

            body.SetMass(0f);

            Assert.That(body.mass, Is.EqualTo(1f), "Body has a 1 unit mass");
            Assert.That(
                body.invMass,
                Is.EqualTo(1f).Within(0.001f),
                "Body has a 1 unit inv mass");
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

            Assert.That(
                body.ShouldCollide(dynamicBody) == true,
                "Body should be colliding with another dynamic body");
        }

        [Test]
        public void ChangingOperativeValues()
        {
            body.ChangeSituation(Vector2.one, Mathf.PI/4f);
            Assert.That(body.position, Is.EqualTo(Vector2.one), "Body is now placed at (1,1)");
            Assert.That(
                body.angle, 
                Is.EqualTo(Mathf.PI / 4f).Within(0.001f), 
                "Body is now oriented at pi/4");

            body.ChangeSituation(Vector2.zero, 0f);
            Assert.That(body.position, Is.EqualTo(Vector2.zero), "Body is now placed at (0,0)");
            Assert.That(body.angle, Is.EqualTo(0f), "Body is now oriented at 0 randian");

            body.SetAwake(false);
            body.ChangeVelocity(Vector2.one, Mathf.PI / 4f);
            Assert.That(
                body.IsAwake() == true,
                "Body has been awaken by velocity change");
            Assert.That(
                body.linearVelocity,
                Is.EqualTo(Vector2.one), 
                "Body has now linear velocity of (1,1)");
            Assert.That(
                body.angularVelocity, 
                Is.EqualTo(Mathf.PI / 4f).Within(0.001f),
                "Body has now angular velocity of pi/4");

            body.SetAwake(false);
            body.ChangeVelocity(Vector2.zero, 0f);
            Assert.That(
                body.IsAwake() == false,
                "Body has not been awaken by velocity change");
            Assert.That(
               body.linearVelocity,
               Is.EqualTo(Vector2.zero),
               "Body has now linear velocity of (0,0)");
            Assert.That(
                body.angularVelocity,
                Is.EqualTo(0f),
                "Body has now angular velocity of 0f");

            body.ChangeImpulse(Vector2.one, Mathf.PI / 4f);
            Assert.That(
                body.force,
                Is.EqualTo(Vector2.one),
                "Body has now a force of (1,1)");
            Assert.That(
                body.torque,
                Is.EqualTo(Mathf.PI / 4f).Within(0.001f),
                "Body has now a torque of pi/4");

            body.ChangeImpulse(Vector2.zero, 0f);
            Assert.That(
                body.force,
                Is.EqualTo(Vector2.zero),
                "Body has now a force of (0,0)");
            Assert.That(
                body.torque,
                Is.EqualTo(0f),
                "Body has now a torque of 0f");
        }

        [Test]
        public void ChangingStates()
        {
            body.SetActive(false);
            Assert.That(body.IsActive() == false, "Body is now inactive");
            body.SetActive(true);
            Assert.That(body.IsActive() == true, "Body is now active");

            body.SetAwake(false);
            Assert.That(body.IsAwake() == false, "Body is now asleep");
            body.SetAwake(true);
            Assert.That(body.IsAwake() == true, "Body is now awake");

            body.SetSleepingAllowed(true);
            Assert.That(body.IsSleepingAllowed() == true, "Body now can sleep");
            body.SetSleepingAllowed(false);
            Assert.That(body.IsSleepingAllowed() == false, "Body now cannot sleep");

            body.SetFixedRotation(true);
            Assert.That(body.IsFixedRotation() == true, "Body is now fixedly rotated");
            body.SetFixedRotation(false);
            Assert.That(body.IsFixedRotation() == false, "Body is now fixedly rotated");

            body.SetIslandBound(true);
            Assert.That(body.IsIslandBound() == true, "Body is now bound to an island");
            body.SetIslandBound(false);
            Assert.That(body.IsIslandBound() == false, "Body is now bound to no island");
        }

        [Test]
        public void TransposingValues()
        {
            body.ChangeSituation(Vector2.one, Mathf.PI/4f);
            Assert.That(
                body.GetWorldPoint(Vector2.one),
                OwnNUnit.Is.EqualTo(new Vector2(1f, 2.414f)).Within(0.001f),
                "World point is rightly computed");

            Assert.That(
                body.GetWorldVector(Vector2.one),
                OwnNUnit.Is.EqualTo(new Vector2(0f, 1.414f)).Within(0.001f),
                "World vector is rightly computed");

            Assert.That(
                body.GetLocalPoint(Vector2.zero),
                OwnNUnit.Is.EqualTo(new Vector2(-1.414f, 0f)).Within(0.001f),
                "Local point is rightly computed");

            Assert.That(
                body.GetLocalVector(Vector2.one),
                OwnNUnit.Is.EqualTo(new Vector2(1.414f, 0f)).Within(0.001f),
                "Local vector is rightly computed");

            body.ChangeSituation(Vector2.zero, 0f);
            body.ChangeVelocity(Vector2.zero, 0f);
        }

        [Test]
        public void ApplyingForces()
        {
            //Applying force
            body.ChangeImpulse(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyForce(
             Vector2.one,
             new Vector2(-0.5f,0.5f),
             false);

            Assert.That(
                body.force,
                Is.EqualTo(Vector2.zero),
                "Applying force should not wake a sleeping body if not specified");

            body.ApplyForce(
             Vector2.one,
             new Vector2(-0.5f, 0.5f),
             true);

            Assert.That(
                body.force,
                Is.EqualTo(Vector2.one),
                "Applying force should produce the specified output force");

            Assert.That(
                body.torque,
                Is.EqualTo(-1f),
                "Applying force should produce the specified output torque");

            //Applying force to Center
            body.ChangeImpulse(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyForceToCenter(
             Vector2.one,
             false);

            Assert.That(
                body.force,
                Is.EqualTo(Vector2.zero),
                "Applying force to center should not wake a sleeping body if not specified");

            body.ApplyForceToCenter(
             Vector2.one,
             true);

            Assert.That(
                body.force,
                Is.EqualTo(Vector2.one),
                "Applying force to center should produce the specified output force");

            Assert.That(
                body.torque,
                Is.EqualTo(0f),
                "Applying force to center should not produce no torque");

            //Applying Torque
            body.ChangeImpulse(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyTorque(
             1f,
             false);

            Assert.That(
                body.torque,
                Is.EqualTo(0f),
                "Applying torque should not wake a sleeping body if not specified");

            body.ApplyTorque(
             1f,
             true);

            Assert.That(
                body.force,
                Is.EqualTo(Vector2.zero),
                "Applying torque should not produce any force");

            Assert.That(
                body.torque,
                Is.EqualTo(1f),
                "Applying torque should produce the specified torque");
        }

        [Test]
        public void ApplyingImpulses()
        {
            //Applying Impulse
            body.ChangeVelocity(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyLinearImpulse(
             Vector2.one,
             new Vector2(-0.5f, 0.5f),
             false);

            Assert.That(
                body.linearVelocity,
                Is.EqualTo(Vector2.zero),
                "Applying linear impulse should not wake a sleeping body if not specified");

            body.ApplyLinearImpulse(
             Vector2.one,
             new Vector2(-0.5f, 0.5f),
             true);

            Assert.That(
                body.linearVelocity,
                Is.EqualTo(Vector2.one),
                "Applying linear impulse should produce the specified output linear velocity");

            Assert.That(
                body.angularVelocity,
                Is.EqualTo(0f),
                "Applying linear impulse should produce the specified output angular velocity");

            //Applying Impulse to Center
            body.ChangeVelocity(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyLinearImpulseToCenter(
             Vector2.one,
             false);

            Assert.That(
                body.linearVelocity,
                Is.EqualTo(Vector2.zero),
                "Applying linear impulse to center should not wake a sleeping body if not specified");

            body.ApplyLinearImpulseToCenter(
             Vector2.one,
             true);

            Assert.That(
                body.linearVelocity,
                Is.EqualTo(Vector2.one),
                "Applying linear impulse should produce the specified output linear velocity");

            Assert.That(
                body.angularVelocity,
                Is.EqualTo(0f),
                "Applying linear impulse not should produce any angular velocity change");

            //Apply Angular Impulse
            body.ChangeVelocity(Vector2.zero, 0f);
            body.SetAwake(false);
            body.ApplyAngularImpulse(
             1f,
             false);

            Assert.That(
                body.angularVelocity,
                Is.EqualTo(0f),
                "Applying angular impulse should not wake a sleeping body if not specified");

            body.ApplyAngularImpulse(
             1f,
             true);

            Assert.That(
                body.linearVelocity,
                Is.EqualTo(Vector2.zero),
                "Applying angular impulse should not produce any linear velocity change");

            Assert.That(
                body.angularVelocity,
                Is.EqualTo(0f),
                "Applying angular impulse should produce the specified angular velocity");
        }


    }
}
