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
        }
    }
}