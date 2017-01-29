using NUnit.Framework;
using NSubstitute;
using UnityEngine;

namespace CrispyPhysics
{
    using Internal;
    using System;
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
            Assert.That(body.friction, Is.EqualTo(0f));
            Assert.That(body.restitution, Is.EqualTo(1f));
            Assert.That(body.sensor == false);

            Assert.That(body.force, Is.EqualTo(Vector2.zero));
            Assert.That(body.torque, Is.EqualTo(0f));

            Assert.That(body.enduringContact == false);

            Assert.That(body.islandBound == false);
            Assert.That(body.islandIndex, Is.EqualTo(0));

            Assert.That(body.GetInertia(), Is.EqualTo(0f));

            Assert.That(body.GetHashCode() == body.GetHashCode());

            IShape shape = ShapeFactory.CreateCircle(1f);
            BodyDefintion bodyDef = new BodyDefintion(
                BodyType.Dynamic, shape, 5f,
                0.2f, 0.3f, 0.5f,
                0.2f, 0.8f, true);

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
            Assert.That(specifiedBody.friction, Is.EqualTo(0.2f));
            Assert.That(specifiedBody.restitution, Is.EqualTo(0.8f));
            Assert.That(specifiedBody.sensor == true);

            Assert.That(specifiedBody.force, Is.EqualTo(Vector2.zero));
            Assert.That(specifiedBody.torque, Is.EqualTo(0f));

            Assert.That(specifiedBody.enduringContact == false);

            Assert.That(specifiedBody.GetInertia(), Is.EqualTo(2.5f));

            bodyDef.shape = ShapeFactory.CreateEdge(Vector2.left, Vector2.right);
            specifiedBody = new Body(0, 0, Vector2.one, 1f, bodyDef);
            Assert.That(specifiedBody.GetInertia(), Is.EqualTo(0f));

            bodyDef.shape = ShapeFactory.CreateBox(1f, 1f);
            specifiedBody = new Body(0, 0, Vector2.one, 1f, bodyDef);
            Assert.That(specifiedBody.GetInertia(), Is.EqualTo(3.333f).Within(0.001f));

            Assert.That(body.GetHashCode() == specifiedBody.GetHashCode());

            bodyDef.type = BodyType.Static;
            specifiedBody = new Body(1, 0, Vector2.one, 1f, bodyDef);
            Assert.That(specifiedBody.mass, Is.EqualTo(0f).Within(0.001f));
            Assert.That(specifiedBody.invMass, Is.EqualTo(0f).Within(0.001f));
            Assert.That(specifiedBody.GetInertia(), Is.EqualTo(0f).Within(0.001f));

            Assert.That(body.GetHashCode() != specifiedBody.GetHashCode());
        }

        [Test]
        public void handlingContactEvents()
        {
            Body body = new Body(0, 0, Vector2.zero, 0f, BodyType.Dynamic, null);

            IContact sentContact = Substitute.For<IContact>();
            IContactMomentum sentContactMomentum = Substitute.For<IContactMomentum>();
            IContact receivedContact = null;
            IContactMomentum receivedMomentum = null;
            IContactHandlerDelegate contactDelegate = (contact, momentum) =>
            {
                receivedContact = sentContact;
                receivedMomentum = momentum;
            };

            receivedContact = null;
            body.ContactStartForeseen += contactDelegate;
            body.NotifyContactStartForeseen(sentContact, sentContactMomentum);
            Assert.That(receivedContact, Is.EqualTo(sentContact));
            Assert.That(receivedMomentum, Is.EqualTo(receivedMomentum));

            receivedContact = null;
            receivedMomentum = null;
            body.ContactStartForeseen -= contactDelegate;
            body.NotifyContactStartForeseen(sentContact, sentContactMomentum);
            Assert.That(receivedContact, Is.EqualTo(null));
            Assert.That(receivedMomentum, Is.EqualTo(null));

            receivedContact = null;
            receivedMomentum = null;
            body.ContactEndForeseen += contactDelegate;
            body.NotifyContactEndForeseen(sentContact, sentContactMomentum);
            Assert.That(receivedContact, Is.EqualTo(sentContact));
            Assert.That(receivedMomentum, Is.EqualTo(receivedMomentum));

            receivedContact = null;
            receivedMomentum = null;
            body.ContactEndForeseen -= contactDelegate;
            body.NotifyContactEndForeseen(sentContact, sentContactMomentum);
            Assert.That(receivedContact, Is.EqualTo(null));
            Assert.That(receivedMomentum, Is.EqualTo(null));

            receivedContact = null;
            receivedMomentum = null;
            body.ContactStarted += contactDelegate;
            body.NotifyContactStarted(sentContact, sentContactMomentum);
            Assert.That(receivedContact, Is.EqualTo(sentContact));
            Assert.That(receivedMomentum, Is.EqualTo(receivedMomentum));

            receivedContact = null;
            receivedMomentum = null;
            body.ContactStarted -= contactDelegate;
            body.NotifyContactStarted(sentContact, sentContactMomentum);
            Assert.That(receivedContact, Is.EqualTo(null));
            Assert.That(receivedMomentum, Is.EqualTo(null));

            receivedContact = null;
            receivedMomentum = null;
            body.ContactEnded += contactDelegate;
            body.NotifyContactEnded(sentContact, sentContactMomentum);
            Assert.That(receivedContact, Is.EqualTo(sentContact));
            Assert.That(receivedMomentum, Is.EqualTo(sentContactMomentum));

            receivedContact = null;
            receivedMomentum = null;
            body.ContactEnded -= contactDelegate;
            body.NotifyContactEnded(sentContact, sentContactMomentum);
            Assert.That(receivedContact, Is.EqualTo(null));
            Assert.That(receivedMomentum, Is.EqualTo(null));

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
            bool bodyWasModfified = false;
            body.ExternalChange += (sender, args) => bodyWasModfified = true;
            body.ChangeSituation(Vector2.down, -1f);

            Assert.That(body.position, Is.EqualTo(Vector2.down));
            Assert.That(body.angle, Is.EqualTo(-1f));
            Assert.That(bodyWasModfified);


            bool staticBodyWasModfified = false;
            staticBody.ExternalChange += (sender, args) => staticBodyWasModfified = true;
            staticBody.ChangeSituation(Vector2.down, -1f);

            Assert.That(staticBody.position, Is.EqualTo(Vector2.down));
            Assert.That(staticBody.angle, Is.EqualTo(-1f));
            Assert.That(staticBodyWasModfified);


            bool kinematicBodyWasModfified = false;
            kinematicBody.ExternalChange += (sender, args) => kinematicBodyWasModfified = true;
            kinematicBody.ChangeSituation(Vector2.down, -1f);

            Assert.That(kinematicBody.position, Is.EqualTo(Vector2.down));
            Assert.That(kinematicBody.angle, Is.EqualTo(-1f));
            Assert.That(kinematicBodyWasModfified);
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
            body.current.ChangeTickDt(0.01f);

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

            body.Foresee();
            body.futur.ChangeImpulse(Vector2.up, -1f);
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(4));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.up));
            Assert.That(body.futur.torque, Is.EqualTo(-1f));

            body.ClearFutur(4);
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(3));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.one));
            Assert.That(body.futur.torque, Is.EqualTo(1f));

            body.Foresee();
            body.futur.ChangeImpulse(Vector2.up, -1f);
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(4));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.up));
            Assert.That(body.futur.torque, Is.EqualTo(-1f));

            body.Foresee();
            body.futur.ChangeImpulse(Vector2.left, -2f);
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(5));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.left));
            Assert.That(body.futur.torque, Is.EqualTo(-2f));

            body.ClearFutur(4);
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
            Assert.That(body.futur.tick, Is.EqualTo(6));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.down));
            Assert.That(body.futur.torque, Is.EqualTo(-1f));

            body.Step();
            Assert.That(body.current.tick, Is.EqualTo(4));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen());

            int i = 0;
            foreach (IMomentum momentum in body.MomentumIterator(0, 6))
            {
                if (i == 0)
                {
                    Assert.That(momentum.tick, Is.EqualTo(0));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.zero));
                    Assert.That(momentum.torque, Is.EqualTo(0f));
                }
                else if (i == 1)
                {
                    Assert.That(momentum.tick, Is.EqualTo(3));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.one));
                    Assert.That(momentum.torque, Is.EqualTo(1f));
                }
                else if (i == 2)
                {
                    Assert.That(momentum.tick, Is.EqualTo(4));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.one));
                    Assert.That(momentum.torque, Is.EqualTo(1f));
                }
                else if (i == 3)
                {
                    Assert.That(momentum.tick, Is.EqualTo(6));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.down));
                    Assert.That(momentum.torque, Is.EqualTo(-1f));
                }
                i++;
            }

            i = 0;
            foreach (IMomentum momentum in body.MomentumIterator(3, 6))
            {
                if (i == 0)
                {
                    Assert.That(momentum.tick, Is.EqualTo(3));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.one));
                    Assert.That(momentum.torque, Is.EqualTo(1f));
                }
                else if (i == 1)
                {
                    Assert.That(momentum.tick, Is.EqualTo(4));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.one));
                    Assert.That(momentum.torque, Is.EqualTo(1f));
                }
                else if (i == 2)
                {
                    Assert.That(momentum.tick, Is.EqualTo(6));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.down));
                    Assert.That(momentum.torque, Is.EqualTo(-1f));
                }
                i++;
            }

            i = 0;
            foreach (IMomentum momentum in body.MomentumIterator(6, 0))
            {
                if (i == 3)
                {
                    Assert.That(momentum.tick, Is.EqualTo(0));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.zero));
                    Assert.That(momentum.torque, Is.EqualTo(0f));
                }
                else if (i == 2)
                {
                    Assert.That(momentum.tick, Is.EqualTo(3));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.one));
                    Assert.That(momentum.torque, Is.EqualTo(1f));
                }
                else if (i == 1)
                {
                    Assert.That(momentum.tick, Is.EqualTo(4));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.one));
                    Assert.That(momentum.torque, Is.EqualTo(1f));
                }
                else if (i == 0)
                {
                    Assert.That(momentum.tick, Is.EqualTo(6));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.down));
                    Assert.That(momentum.torque, Is.EqualTo(-1f));
                }
                i++;
            }

            i = 0;
            foreach (IMomentum momentum in body.MomentumIterator(6, 3))
            {
                if (i == 2)
                {
                    Assert.That(momentum.tick, Is.EqualTo(3));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.one));
                    Assert.That(momentum.torque, Is.EqualTo(1f));
                }
                else if (i == 1)
                {
                    Assert.That(momentum.tick, Is.EqualTo(4));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.one));
                    Assert.That(momentum.torque, Is.EqualTo(1f));
                }
                else if (i == 0)
                {
                    Assert.That(momentum.tick, Is.EqualTo(6));
                    Assert.That(momentum.force, Is.EqualTo(Vector2.down));
                    Assert.That(momentum.torque, Is.EqualTo(-1f));
                }
                i++;
            }

            Assert.That(body.IndexForTick(5), Is.EqualTo(2));

            IMomentum fetchedMomentum = body.MomentumForTick(5);
            Assert.That(fetchedMomentum.tick, Is.EqualTo(4));
            Assert.That(fetchedMomentum.force, Is.EqualTo(Vector2.one));
            Assert.That(fetchedMomentum.torque, Is.EqualTo(1f));

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
            body.ClearFutur();
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

            body.ClearFutur();
            Assert.That(body.current.tick, Is.EqualTo(4));
            Assert.That(body.force, Is.EqualTo(Vector2.one));
            Assert.That(body.torque, Is.EqualTo(1f));
            Assert.That(body.IsForeseen() == false);
    
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
            Assert.That(body.past.tick, Is.EqualTo(0));
            Assert.That(body.past.force, Is.EqualTo(Vector2.down));
            Assert.That(body.past.torque, Is.EqualTo(-1f));
            Assert.That(body.IsForeseen());
            Assert.That(body.futur.tick, Is.EqualTo(4));
            Assert.That(body.futur.force, Is.EqualTo(Vector2.down));
            Assert.That(body.futur.torque, Is.EqualTo(-1f));


            Assert.Throws<ArgumentOutOfRangeException>(
                delegate { body.CrispAtTick(0, Vector2.up, 1f); });

            Assert.That(body.CrispAtTick(3, Vector2.up, 1f, 0.25f) == false);

            Assert.That(body.current.tick, Is.EqualTo(0));
            Assert.That(body.position, Is.EqualTo(Vector2.zero));
            Assert.That(body.angle, Is.EqualTo(0f));

            Assert.That(body.futur.tick, Is.EqualTo(4));
            Assert.That(body.futur.position, Is.EqualTo(Vector2.zero));
            Assert.That(body.futur.angle, Is.EqualTo(0f));

            Assert.That(body.CrispAtTick(4, Vector2.up, 1f, 1f) == true);

            Assert.That(body.current.tick, Is.EqualTo(0));
            Assert.That(body.position, Is.EqualTo(Vector2.zero));
            Assert.That(body.angle, Is.EqualTo(0f));

            Assert.That(body.MomentumForTick(1).tick, Is.EqualTo(1));
            Assert.That(body.MomentumForTick(1).position, Is.EqualTo(new Vector2(0f,0.25f)));
            Assert.That(body.MomentumForTick(1).angle, Is.EqualTo(0.25f));
            Assert.That(body.MomentumForTick(1).linearVelocity, Is.EqualTo(new Vector2(0f, 25f)));
            Assert.That(body.MomentumForTick(1).angularVelocity, Is.EqualTo(25f));

            Assert.That(body.MomentumForTick(2).tick, Is.EqualTo(2));
            Assert.That(body.MomentumForTick(2).position, Is.EqualTo(new Vector2(0f, 0.5f)));
            Assert.That(body.MomentumForTick(2).angle, Is.EqualTo(0.5f));
            Assert.That(body.MomentumForTick(2).linearVelocity, Is.EqualTo(new Vector2(0f, 50f)));
            Assert.That(body.MomentumForTick(2).angularVelocity, Is.EqualTo(50f));

            Assert.That(body.MomentumForTick(3).tick, Is.EqualTo(3));
            Assert.That(body.MomentumForTick(3).position, Is.EqualTo(new Vector2(0f, 0.75f)));
            Assert.That(body.MomentumForTick(3).angle, Is.EqualTo(0.75f));
            Assert.That(body.MomentumForTick(3).linearVelocity, Is.EqualTo(new Vector2(0f, 75f)));
            Assert.That(body.MomentumForTick(3).angularVelocity, Is.EqualTo(75f));

            Assert.That(body.MomentumForTick(4).tick, Is.EqualTo(4));
            Assert.That(body.MomentumForTick(4).position, Is.EqualTo(new Vector2(0f, 1f)));
            Assert.That(body.MomentumForTick(4).angle, Is.EqualTo(1f));
            Assert.That(body.MomentumForTick(4).linearVelocity, Is.EqualTo(new Vector2(0f, 100f)));
            Assert.That(body.MomentumForTick(4).angularVelocity, Is.EqualTo(100f));

        }
    }
}
