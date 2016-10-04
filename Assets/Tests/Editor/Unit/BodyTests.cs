using NUnit.Framework;
using NSubstitute;
using UnityEngine;

namespace CrispyPhysics
{
    using Internal;
    [TestFixture]
    public class BodyTests
    {
        [Test]
        public void CreatingBody()
        {
            Body body = new Body(0, 0, Vector2.zero, 0f, BodyType.Dynamic, null);
            Assert.That(body.islandBound == false);

            Assert.That(body.type, Is.EqualTo(BodyType.Dynamic));

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

            IShape shape = ShapeFactory.CreateCircle(1f);
            BodyDefintion bodyDef = new BodyDefintion(
                BodyType.Dynamic, shape, 5f,
                0.2f, 0.3f, 0.5f);

            Body specifiedBody = new Body(0, 0, Vector2.one, 1f, bodyDef);


            Assert.That(specifiedBody.type, Is.EqualTo(BodyType.Dynamic));

            Assert.That(specifiedBody.position, Is.EqualTo(Vector2.one));
            Assert.That(specifiedBody.angle, Is.EqualTo(1f));

            Assert.That(specifiedBody.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(specifiedBody.angularVelocity, Is.EqualTo(0f));

            Assert.That(specifiedBody.mass, Is.EqualTo(5f));
            Assert.That(specifiedBody.invMass, Is.EqualTo(1f/5f).Within(0.001f));

            Assert.That(specifiedBody.linearDamping, Is.EqualTo(0.2f));
            Assert.That(specifiedBody.angularDamping, Is.EqualTo(0.3f));
            Assert.That(specifiedBody.gravityScale, Is.EqualTo(0.5f));

            Assert.That(specifiedBody.force, Is.EqualTo(Vector2.zero));
            Assert.That(specifiedBody.torque, Is.EqualTo(0f));

            Assert.That(specifiedBody.GetInertia(), Is.EqualTo(2.5f));

            bodyDef.shape = ShapeFactory.CreateEdge(Vector2.left, Vector2.right);
            specifiedBody = new Body(0, 0, Vector2.one, 1f, bodyDef);
            Assert.That(specifiedBody.GetInertia(), Is.EqualTo(0f));

            bodyDef.shape = ShapeFactory.CreateBox(1f, 1f);
            specifiedBody = new Body(0, 0, Vector2.one, 1f, bodyDef);
            Assert.That(specifiedBody.GetInertia(), Is.EqualTo(3.333f).Within(0.001f));

            bodyDef.type = BodyType.Static;
            specifiedBody = new Body(0, 0, Vector2.one, 1f, bodyDef);
            Assert.That(specifiedBody.mass, Is.EqualTo(0f).Within(0.001f));
            Assert.That(specifiedBody.invMass, Is.EqualTo(0f).Within(0.001f));
            Assert.That(specifiedBody.GetInertia(), Is.EqualTo(0f).Within(0.001f));
        }

        [Test]
        public void ShouldBeCollidingBody()
        {
            Body body = new Body(0, 0, Vector2.zero, 0f, BodyType.Dynamic, null);
            IBody dynamicDummy = Substitute.For<IBody>();
            dynamicDummy.type.Returns(BodyType.Dynamic);

            IBody staticDummy = Substitute.For<IBody>();
            staticDummy.type.Returns(BodyType.Static);

            IBody kineticDummy = Substitute.For<IBody>();
            kineticDummy.type.Returns(BodyType.Kinematic);


            Assert.That(body.ShouldCollide(dynamicDummy) == true);
            Assert.That(body.ShouldCollide(staticDummy) == true);
            Assert.That(body.ShouldCollide(kineticDummy) == true);

            Body staticBody = new Body(0, 0, Vector2.zero, 0f, BodyType.Static, null);

            Assert.That(staticBody.ShouldCollide(dynamicDummy) == true);
            Assert.That(staticBody.ShouldCollide(staticDummy) == false);
            Assert.That(staticBody.ShouldCollide(kineticDummy) == false);

            Body kineticBody = new Body(0, 0, Vector2.zero, 0f, BodyType.Static, null);

            Assert.That(kineticBody.ShouldCollide(dynamicDummy) == true);
            Assert.That(kineticBody.ShouldCollide(staticDummy) == false);
            Assert.That(kineticBody.ShouldCollide(kineticDummy) == false);
        }

        [Test]
        public void ChangingStates()
        {
            Body body = new Body(
                0, 0, Vector2.zero, 0f,
                BodyType.Dynamic, ShapeFactory.CreateCircle(1f));
            Body staticBody = new Body(
                0, 0, Vector2.zero, 0f,
                BodyType.Static, ShapeFactory.CreateEdge(Vector2.left, Vector2.right));
            Body kinematicBody = new Body(
                0, 0, Vector2.zero, 0f,
                BodyType.Kinematic, ShapeFactory.CreateBox(1f, 1f));
            //Changing Impulse
            body.ChangeImpulse(Vector2.one, 1f);
            staticBody.ChangeImpulse(Vector2.one, 1f);
            kinematicBody.ChangeImpulse(Vector2.one, 1f);

            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(staticBody.force, Is.EqualTo(Vector2.zero));
            Assert.That(staticBody.torque, Is.EqualTo(0f));
            Assert.That(kinematicBody.force, Is.EqualTo(Vector2.one));
            Assert.That(kinematicBody.torque, Is.EqualTo(1f));

            //Changing Velocity
            body.ChangeVelocity(Vector2.up, 2f);
            staticBody.ChangeVelocity(Vector2.up, 2f);
            kinematicBody.ChangeVelocity(Vector2.up, 2f);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.up));
            Assert.That(body.angularVelocity, Is.EqualTo(2f));
            Assert.That(staticBody.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(staticBody.angularVelocity, Is.EqualTo(0f));
            Assert.That(kinematicBody.linearVelocity, Is.EqualTo(Vector2.up));
            Assert.That(kinematicBody.angularVelocity, Is.EqualTo(2f));

            //Changing Situation
            body.ChangeSituation(Vector2.down, -1f);
            staticBody.ChangeSituation(Vector2.down, -1f);
            kinematicBody.ChangeSituation(Vector2.down, -1f);

            Assert.That(body.position, Is.EqualTo(Vector2.down));
            Assert.That(body.angle, Is.EqualTo(-1f));
            Assert.That(staticBody.position, Is.EqualTo(Vector2.down));
            Assert.That(staticBody.angle, Is.EqualTo(-1f));
            Assert.That(kinematicBody.position, Is.EqualTo(Vector2.down));
            Assert.That(kinematicBody.angle, Is.EqualTo(-1f));
        }

        [Test]
        public void ApplyingForces()
        {
            Body body = new Body(
                0, 0, Vector2.zero, 0f,
                BodyType.Dynamic, ShapeFactory.CreateCircle(1f));
            Body staticBody = new Body(
                0, 0, Vector2.zero, 0f,
                BodyType.Static, ShapeFactory.CreateEdge(Vector2.left, Vector2.right));
            Body kinematicBody = new Body(
                0, 0, Vector2.zero, 0f,
                BodyType.Kinematic, ShapeFactory.CreateBox(1f, 1f));

            //Applying force
            body.ApplyForce(Vector2.one, new Vector2(-0.5f, 0.5f));
            staticBody.ApplyForce(Vector2.one, new Vector2(-0.5f, 0.5f));
            kinematicBody.ApplyForce(Vector2.one, new Vector2(-0.5f, 0.5f));

            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(-1f));
            Assert.That(staticBody.force, Is.EqualTo(Vector2.zero));
            Assert.That(staticBody.torque, Is.EqualTo(0f));
            Assert.That(kinematicBody.force, Is.EqualTo(Vector2.zero));
            Assert.That(kinematicBody.torque, Is.EqualTo(0f));

            //Applying force to Center
            body.ChangeImpulse(Vector2.zero, 0f);
            body.ApplyForceToCenter(Vector2.one);
            staticBody.ChangeImpulse(Vector2.zero, 0f);
            staticBody.ApplyForceToCenter(Vector2.one);
            kinematicBody.ChangeImpulse(Vector2.zero, 0f);
            kinematicBody.ApplyForceToCenter(Vector2.one);

            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(0f));
            Assert.That(staticBody.force, Is.EqualTo(Vector2.zero));
            Assert.That(staticBody.torque, Is.EqualTo(0f));
            Assert.That(kinematicBody.force, Is.EqualTo(Vector2.zero));
            Assert.That(kinematicBody.torque, Is.EqualTo(0f));

            //Applying Torque
            body.ChangeImpulse(Vector2.zero, 0f);
            body.ApplyTorque(1f);
            staticBody.ChangeImpulse(Vector2.zero, 0f);
            staticBody.ApplyTorque(1f);
            kinematicBody.ChangeImpulse(Vector2.zero, 0f);
            kinematicBody.ApplyTorque(1f);

            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(staticBody.force, Is.EqualTo(Vector2.zero));
            Assert.That(staticBody.torque, Is.EqualTo(0f));
            Assert.That(kinematicBody.force, Is.EqualTo(Vector2.zero));
            Assert.That(kinematicBody.torque, Is.EqualTo(0f));
        }
    
        [Test]
        public void ApplyingImpulses()
        {
            Body body = new Body(
                0, 0, Vector2.zero, 0f,
                BodyType.Dynamic, ShapeFactory.CreateCircle(1f));
            Body staticBody = new Body(
                0, 0, Vector2.zero, 0f, 
                BodyType.Static, ShapeFactory.CreateEdge(Vector2.left, Vector2.right));
            Body kinematicBody = new Body(
                0, 0, Vector2.zero, 0f, 
                BodyType.Kinematic, ShapeFactory.CreateBox(1f, 1f));

            //Applying Impulse
            body.ChangeVelocity(Vector2.zero, 0f);
            body.ApplyLinearImpulse(Vector2.one, new Vector2(-0.5f, 0.5f));
            staticBody.ChangeVelocity(Vector2.zero, 0f);
            staticBody.ApplyLinearImpulse(Vector2.one, new Vector2(-0.5f, 0.5f));
            kinematicBody.ChangeVelocity(Vector2.zero, 0f);
            kinematicBody.ApplyLinearImpulse(Vector2.one, new Vector2(-0.5f, 0.5f));

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.one));
            Assert.That(body.angularVelocity, Is.EqualTo(-2f));
            Assert.That(staticBody.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(staticBody.angularVelocity, Is.EqualTo(0f));
            Assert.That(kinematicBody.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(kinematicBody.angularVelocity, Is.EqualTo(0f));

            //Applying Impulse to Center
            body.ChangeVelocity(Vector2.zero, 0f);
            body.ApplyLinearImpulseToCenter(Vector2.one);
            staticBody.ChangeVelocity(Vector2.zero, 0f);
            staticBody.ApplyLinearImpulseToCenter(Vector2.one);
            kinematicBody.ChangeVelocity(Vector2.zero, 0f);
            kinematicBody.ApplyLinearImpulseToCenter(Vector2.one);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.one));
            Assert.That(body.angularVelocity, Is.EqualTo(0f));
            Assert.That(staticBody.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(staticBody.angularVelocity, Is.EqualTo(0f));
            Assert.That(kinematicBody.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(kinematicBody.angularVelocity, Is.EqualTo(0f));

            //Apply Angular Impulse
            body.ChangeVelocity(Vector2.zero, 0f);
            body.ApplyAngularImpulse(1f);
            staticBody.ChangeVelocity(Vector2.zero, 0f);
            staticBody.ApplyAngularImpulse(1f);
            body.ChangeVelocity(Vector2.zero, 0f);
            body.ApplyAngularImpulse(1f);

            Assert.That(body.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(body.angularVelocity, Is.EqualTo(2f));
            Assert.That(staticBody.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(staticBody.angularVelocity, Is.EqualTo(0f));
            Assert.That(kinematicBody.linearVelocity, Is.EqualTo(Vector2.zero));
            Assert.That(kinematicBody.angularVelocity, Is.EqualTo(0f));
        }

        [Test]
        public void Tracking()
        {
            Body body = new Body(0, 0, Vector2.zero, 0f, BodyType.Dynamic, null);

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
