using NUnit.Framework;
using UnityEngine;

namespace CrispyPhysics
{
    using Internal;
    using System;
    [TestFixture]
    public class ContactSolverTests
    {
        [Test]
        public void usingVelocityContraintPoint()
        {
            VelocityConstraintPoint vcp = new VelocityConstraintPoint(
                Vector2.up, Vector2.down,
                1f, 2f, 3f, 4f, 5f);

            Assert.That(vcp.rA, Is.EqualTo(Vector2.up));
            Assert.That(vcp.rB, Is.EqualTo(Vector2.down));
            Assert.That(vcp.normalImpulse, Is.EqualTo(1f));
            Assert.That(vcp.tangentImpulse, Is.EqualTo(2f));
            Assert.That(vcp.normalMass, Is.EqualTo(3f));
            Assert.That(vcp.tangentMass, Is.EqualTo(4f));
            Assert.That(vcp.velocityBias, Is.EqualTo(5f));
        }

        [Test]
        public void usingContactPositionConstraint()
        {
            ContactPositionConstraint cpc = new ContactPositionConstraint();

            cpc.normal = Vector2.up;
            cpc.point = Vector2.right;
            cpc.indexA = 1;
            cpc.indexB = 2;
            cpc.invMassA = 1f;
            cpc.invMassB = 2f;
            cpc.centerA = Vector2.down;
            cpc.centerB = Vector2.left;
            cpc.invIA = 3f;
            cpc.invIB = 4f;
            cpc.type = Manifold.Type.FaceB;
            cpc.radiusA = 5f;
            cpc.radiusB = 6f;

            Assert.That(cpc.points != null);
            Assert.That(cpc.pointCount, Is.EqualTo(0));
            Assert.That(cpc.normal, Is.EqualTo(Vector2.up));
            Assert.That(cpc.point, Is.EqualTo(Vector2.right));
            Assert.That(cpc.indexA, Is.EqualTo(1));
            Assert.That(cpc.indexB, Is.EqualTo(2));
            Assert.That(cpc.invMassA, Is.EqualTo(1f));
            Assert.That(cpc.invMassB, Is.EqualTo(2f));
            Assert.That(cpc.centerA, Is.EqualTo(Vector2.down));
            Assert.That(cpc.centerB, Is.EqualTo(Vector2.left));
            Assert.That(cpc.invIA, Is.EqualTo(3f));
            Assert.That(cpc.invIB, Is.EqualTo(4f));
            Assert.That(cpc.type, Is.EqualTo(Manifold.Type.FaceB));
            Assert.That(cpc.radiusA, Is.EqualTo(5f));
            Assert.That(cpc.radiusB, Is.EqualTo(6f));

            cpc.AddPoint(new Vector2(5f, 4f));
            Assert.That(cpc.pointCount, Is.EqualTo(1));
            Assert.That(cpc.points[0], Is.EqualTo(new Vector2(5f, 4f)));
        }

        [Test]
        public void usingContactVelocityConstraint()
        {
            ContactVelocityConstraint cvc = new ContactVelocityConstraint();

            cvc.contactIndex = 1;
            cvc.normal = Vector2.up;
            cvc.normalMass = Matrix2x2.identity;
            cvc.K = Matrix2x2.zero;
            cvc.indexA = 1;
            cvc.indexB = 2;
            cvc.invMassA = 1f;
            cvc.invMassB = 2f;
            cvc.invIA = 3f;
            cvc.invIB = 4f;
            cvc.friction = 0.2f;
            cvc.restitution = 0.8f;
            cvc.tangentSpeed = 5f;

            Assert.That(cvc.points != null);
            Assert.That(cvc.pointCount, Is.EqualTo(0));
            Assert.That(cvc.normal, Is.EqualTo(Vector2.up));
            Assert.That(cvc.normalMass, Is.EqualTo(Matrix2x2.identity));
            Assert.That(cvc.K, Is.EqualTo(Matrix2x2.zero));
            Assert.That(cvc.indexA, Is.EqualTo(1));
            Assert.That(cvc.indexB, Is.EqualTo(2));
            Assert.That(cvc.invMassA, Is.EqualTo(1f));
            Assert.That(cvc.invMassB, Is.EqualTo(2f));
            Assert.That(cvc.invIA, Is.EqualTo(3f));
            Assert.That(cvc.invIB, Is.EqualTo(4f));
            Assert.That(cvc.friction, Is.EqualTo(0.2f));
            Assert.That(cvc.restitution, Is.EqualTo(0.8f));
            Assert.That(cvc.tangentSpeed, Is.EqualTo(5f));

            VelocityConstraintPoint vcp = new VelocityConstraintPoint();
            cvc.AddPoint(vcp);
            Assert.That(cvc.pointCount, Is.EqualTo(1));
            Assert.That(cvc.points[0], Is.EqualTo(vcp));

            VelocityConstraintPoint popedVcp = cvc.PopPoint();
            Assert.That(cvc.pointCount, Is.EqualTo(0));
            Assert.That(popedVcp, Is.EqualTo(vcp));

            Assert.Throws<InvalidOperationException>(
                delegate { cvc.PopPoint();});
        }

        [Test]
        public void creatingContactSolver()
        {
            TimeStep ts = new TimeStep(0.01f, 100f, 1f, new Vector2(0f, -10f), 8, 3, 100f, 360f);
            Contact[] contacts = new Contact[1];
            Position[] positions = new Position[2];
            Velocity[] velocities = new Velocity[2];

            Assert.Throws<ArgumentOutOfRangeException>(
                delegate { new ContactSolver(ts, 0, null, null, null); });

            Assert.Throws<ArgumentNullException>(
                delegate { new ContactSolver(ts, 1, null, null, null); });

            Assert.Throws<ArgumentNullException>(
                delegate { new ContactSolver(ts, 1, contacts, null, null); });

            Assert.Throws<ArgumentNullException>(
                delegate { new ContactSolver(ts, 1, contacts, positions, null); });


            IShape circleShape = new CircleShape(Vector2.zero, 1f);
            IShape edgeShape = new EdgeShape(Vector2.left, Vector2.right);

            Body circleBody = new Body(
                0, 0, new Vector2(0.5f, 1f), 0f, BodyType.Dynamic, circleShape,
                1f, 0f, 0f, 1f, 0.2f, 0.8f, false);

            circleBody.internalFutur.ChangeVelocity(Vector2.one * -1, 0.5f);

            Body edgeBody = new Body(
               0, 0, Vector2.zero, 0f, BodyType.Static, edgeShape,
               1f, 0f, 0f, 1f, 0.4f, 0.6f, false);

            Contact contact = ContactFactory.CreateContact(0, circleBody, edgeBody);
            Manifold mf = contact.Evaluate(
                contact.internalFirstBody.internalFutur.transform, 
                contact.internalSecondBody.internalFutur.transform);
            contact.futur.Change(mf, 0f, true,
                contact.internalFirstBody.internalFutur.position,
                contact.internalSecondBody.internalFutur.position);

            contacts[0] = contact;

            circleBody.islandIndex = 0;
            edgeBody.islandIndex = 1;

            positions[0] = new Position(circleBody.futur.position, circleBody.futur.angle);
            positions[1] = new Position(edgeBody.futur.position, edgeBody.futur.angle);

            velocities[0] = new Velocity(circleBody.futur.linearVelocity, circleBody.futur.angularVelocity);
            velocities[1] = new Velocity(edgeBody.futur.linearVelocity, edgeBody.futur.angularVelocity);



            ContactSolver cs = new ContactSolver(ts, 1, contacts, positions, velocities);

            Assert.That(cs.step, Is.EqualTo(ts));
            Assert.That(cs.count, Is.EqualTo(1));
            Assert.That(cs.contacts, Is.EqualTo(contacts));
            Assert.That(cs.positions, Is.EqualTo(positions));
            Assert.That(cs.velocities, Is.EqualTo(velocities));
            Assert.That(cs.positionConstraints != null);
            Assert.That(cs.positionConstraints.Length, Is.EqualTo(1));
            Assert.That(cs.velocityConstraints != null);
            Assert.That(cs.velocityConstraints.Length, Is.EqualTo(1));

            Assert.That(cs.positionConstraints[0].centerA, Is.EqualTo(Vector2.zero));
            Assert.That(cs.positionConstraints[0].centerB, Is.EqualTo(Vector2.zero));
            Assert.That(cs.positionConstraints[0].indexA, Is.EqualTo(1));
            Assert.That(cs.positionConstraints[0].indexB, Is.EqualTo(0));
            Assert.That(cs.positionConstraints[0].invIA, Is.EqualTo(0f));
            Assert.That(cs.positionConstraints[0].invIB, Is.EqualTo(2f));
            Assert.That(cs.positionConstraints[0].invMassA, Is.EqualTo(0f));
            Assert.That(cs.positionConstraints[0].invMassB, Is.EqualTo(1f));
            Assert.That(cs.positionConstraints[0].normal, Is.EqualTo(Vector2.up));
            Assert.That(cs.positionConstraints[0].point, Is.EqualTo(Vector2.left));
            Assert.That(cs.positionConstraints[0].pointCount, Is.EqualTo(1));
            Assert.That(cs.positionConstraints[0].points[0], Is.EqualTo(Vector2.zero));
            Assert.That(cs.positionConstraints[0].radiusA, Is.EqualTo(Constants.polygonRadius));
            Assert.That(cs.positionConstraints[0].radiusB, Is.EqualTo(1f));

            Assert.That(cs.velocityConstraints[0].contactIndex, Is.EqualTo(0));
            Assert.That(cs.velocityConstraints[0].normal, Is.EqualTo(Vector2.zero));
            Assert.That(cs.velocityConstraints[0].normalMass, Is.EqualTo(Matrix2x2.zero));
            Assert.That(cs.velocityConstraints[0].K, Is.EqualTo(Matrix2x2.zero));
            Assert.That(cs.velocityConstraints[0].indexA, Is.EqualTo(1));
            Assert.That(cs.velocityConstraints[0].indexB, Is.EqualTo(0));
            Assert.That(cs.velocityConstraints[0].invIA, Is.EqualTo(0f));
            Assert.That(cs.velocityConstraints[0].invIB, Is.EqualTo(2f));
            Assert.That(cs.velocityConstraints[0].invMassA, Is.EqualTo(0f));
            Assert.That(cs.velocityConstraints[0].invMassB, Is.EqualTo(1f));
            Assert.That(
                cs.velocityConstraints[0].friction, 
                Is.EqualTo(Mathf.Sqrt(circleBody.friction * edgeBody.friction)).Within(0.001f));
            Assert.That(cs.velocityConstraints[0].restitution, Is.EqualTo(0.8f));
            Assert.That(cs.velocityConstraints[0].tangentSpeed, Is.EqualTo(0f));
            Assert.That(cs.velocityConstraints[0].pointCount, Is.EqualTo(1));
            Assert.That(cs.velocityConstraints[0].points[0].rA, Is.EqualTo(Vector2.zero));
            Assert.That(cs.velocityConstraints[0].points[0].rB, Is.EqualTo(Vector2.zero));
            Assert.That(cs.velocityConstraints[0].points[0].normalImpulse, Is.EqualTo(0f));
            Assert.That(cs.velocityConstraints[0].points[0].tangentImpulse, Is.EqualTo(0f));
            Assert.That(cs.velocityConstraints[0].points[0].normalMass, Is.EqualTo(0f));
            Assert.That(cs.velocityConstraints[0].points[0].tangentMass, Is.EqualTo(0f));
            Assert.That(cs.velocityConstraints[0].points[0].velocityBias, Is.EqualTo(0f));

            cs.InitializeVelocityConstraints();

            Assert.That(
                cs.velocityConstraints[0].points[0].rA, 
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 0.005f)).Within(0.001f));
            Assert.That(
                cs.velocityConstraints[0].points[0].rB,
                OwnNUnit.Is.EqualTo(new Vector2(0.0f, -0.995f)).Within(0.001f));
            Assert.That(cs.velocityConstraints[0].points[0].normalImpulse, Is.EqualTo(0f));
            Assert.That(cs.velocityConstraints[0].points[0].tangentImpulse, Is.EqualTo(0f));
            Assert.That(cs.velocityConstraints[0].points[0].normalMass, Is.EqualTo(1f));
            Assert.That(cs.velocityConstraints[0].points[0].tangentMass, Is.EqualTo(0.335f).Within(0.001f));
            Assert.That(cs.velocityConstraints[0].points[0].velocityBias, Is.EqualTo(0f));
        }

        [Test]
        public void usingContactSolver()
        {
            TimeStep ts = new TimeStep(0.01f, 100f, 1f, new Vector2(0f, -10f), 8, 3, 100f, 360f);
            Contact[] contacts = new Contact[1];
            Position[] positions = new Position[2];
            Velocity[] velocities = new Velocity[2];

            IShape circleShape = new CircleShape(Vector2.zero, 1f);
            IShape edgeShape = new EdgeShape(Vector2.left, Vector2.right);

            Body circleBody = new Body(
                0, 0, new Vector2(0.5f, 1f), 0f, BodyType.Dynamic, circleShape,
                1f, 0f, 0f, 1f, 0.2f, 0.8f, false);

            circleBody.internalFutur.ChangeVelocity(Vector2.one * -10, 0.5f);

            Body edgeBody = new Body(
               0, 0, Vector2.zero, 0f, BodyType.Static, edgeShape,
               1f, 0f, 0f, 1f, 0.4f, 0.6f, false);

            Contact contact = ContactFactory.CreateContact(0, circleBody, edgeBody);
            Manifold mf = contact.Evaluate(
                contact.internalFirstBody.internalFutur.transform, 
                contact.internalSecondBody.internalFutur.transform);
            contact.futur.Change(mf, 0f, true,
                contact.internalFirstBody.internalFutur.position,
                contact.internalSecondBody.internalFutur.position);

            contacts[0] = contact;

            circleBody.islandIndex = 0;
            edgeBody.islandIndex = 1;

            positions[0] = new Position(circleBody.futur.position, circleBody.futur.angle);
            positions[1] = new Position(edgeBody.futur.position, edgeBody.futur.angle);

            velocities[0] = new Velocity(circleBody.futur.linearVelocity, circleBody.futur.angularVelocity);
            velocities[1] = new Velocity(edgeBody.futur.linearVelocity, edgeBody.futur.angularVelocity);



            ContactSolver cs = new ContactSolver(ts, 1, contacts, positions, velocities);

            cs.InitializeVelocityConstraints();

            for (uint i = 0; i < ts.velocityIterations; i++)
                cs.SolveVelocityConstraints();

            Assert.That(
                velocities[0].linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-6.8112f, 8.0f)).Within(0.001f));

            Assert.That(velocities[0].angularVelocity, Is.EqualTo(6.845f).Within(0.001f));

            Assert.That(velocities[1].linearVelocity, Is.EqualTo(Vector2.zero));

            Assert.That(velocities[1].angularVelocity, Is.EqualTo(0f));

            for (uint i = 0; i < ts.positionIterations; i++)
                cs.SolvePositionConstraints();

            Assert.That(
                positions[0].center,
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 1.002f)).Within(0.001f));

            Assert.That(positions[0].angle, Is.EqualTo(0f).Within(0.001f));

            Assert.That(positions[1].center, Is.EqualTo(Vector2.zero));

            Assert.That(positions[1].angle, Is.EqualTo(0f));
        }
    }
}