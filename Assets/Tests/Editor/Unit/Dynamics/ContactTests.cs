using NUnit.Framework;
using UnityEngine;

namespace CrispyPhysics
{
    using Internal;
    using System;
    [TestFixture]
    public class ContactTests
    {
        [Test]
        public void usingContactFactory()
        {
            Assert.Throws<ArgumentNullException>(
                delegate { ContactFactory.CreateContact(0, null, null); });

            Body bodyA = new Body(0, 0, Vector2.zero, 0f, BodyType.Dynamic, null);

            Assert.Throws<ArgumentNullException>(
                delegate { ContactFactory.CreateContact(0, bodyA, null); });

            Body bodyB = new Body(1, 0, Vector2.zero, 0f, BodyType.Dynamic, null);

            Assert.Throws<ArgumentException>(
                delegate { ContactFactory.CreateContact(0, bodyA, bodyB); });

            IShape shapeA = ShapeFactory.CreateCircle(1f);
            bodyA = new Body(
                0, 0, Vector2.zero, 0f, BodyType.Dynamic, shapeA,
                1f, 0f, 0f, 1f, 0.2f, 0.8f, false);

            Assert.Throws<ArgumentException>(
                delegate { ContactFactory.CreateContact(0, bodyA, bodyB); });

            IShape shapeB = ShapeFactory.CreateCircle(1f);
            bodyB = new Body(
                0, 0, Vector2.zero, 0f, BodyType.Dynamic, shapeB,
                1f, 0f, 0f, 1f, 0.4f, 0.6f, false);

            Contact contact = ContactFactory.CreateContact(0, bodyA, bodyB);

            Assert.That(contact is CircleContact);
            Assert.That(contact.bodyA, Is.EqualTo(bodyA));
            Assert.That(contact.firstBody, Is.EqualTo(bodyA));
            Assert.That(contact.bodyB, Is.EqualTo(bodyB));
            Assert.That(contact.secondBody, Is.EqualTo(bodyB));
            Assert.That(
                contact.friction, 
                Is.EqualTo(Mathf.Sqrt(bodyA.friction * bodyB.friction)));
            Assert.That(contact.restitution, Is.EqualTo(0.8f));
            Assert.That(contact.islandBound == false);


            shapeB = ShapeFactory.CreateEdge(Vector2.left, Vector2.right);
            bodyB = new Body( 0, 0, Vector2.zero, 0f, BodyType.Dynamic, shapeB);
            contact = ContactFactory.CreateContact(0, bodyA, bodyB);
            Assert.That(contact is EdgeAndCircleContact);
            contact = ContactFactory.CreateContact(0, bodyB, bodyA);
            Assert.That(contact is EdgeAndCircleContact);

            shapeB = ShapeFactory.CreateBox(1f, 1f);
            bodyB = new Body(0, 0, Vector2.zero, 0f, BodyType.Dynamic, shapeB);
            contact = ContactFactory.CreateContact(0, bodyA, bodyB);
            Assert.That(contact is PolygonAndCircleContact);
            contact = ContactFactory.CreateContact(0, bodyB, bodyA);
            Assert.That(contact is PolygonAndCircleContact);
        }

        [Test]
        public void Tracking()
        {
            IShape shapeA = ShapeFactory.CreateCircle(1f);
            Body bodyA = new Body(0, 0, Vector2.zero, 0f, BodyType.Dynamic, shapeA);

            IShape shapeB = ShapeFactory.CreateCircle(1f);
            Body bodyB = new Body(0, 0, Vector2.zero, 0f, BodyType.Dynamic, shapeB);
            Contact contact = ContactFactory.CreateContact(0, bodyA, bodyB);

            contact.Step();
            Assert.That(contact.current.tick, Is.EqualTo(1));
            Assert.That(contact.current.manifold, Is.EqualTo(null));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.current.isTouching == false);
            Assert.That(contact.past.tick, Is.EqualTo(0f));
            Assert.That(contact.past.manifold, Is.EqualTo(null));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.past.isTouching == false);
            Assert.That(contact.IsForeseen() == false);
            Assert.That(contact.IsDroppable() == true);

            contact.Foresee();
            Assert.That(contact.current.tick, Is.EqualTo(1));
            Assert.That(contact.current.manifold, Is.EqualTo(null));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.current.isTouching == false);
            Assert.That(contact.past.tick, Is.EqualTo(0f));
            Assert.That(contact.past.manifold, Is.EqualTo(null));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.past.isTouching == false);
            Assert.That(contact.IsForeseen());
            Assert.That(contact.futur.tick, Is.EqualTo(2f));
            Assert.That(contact.futur.manifold, Is.EqualTo(null));
            Assert.That(contact.futur.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.futur.isTouching == false);
            Assert.That(contact.IsDroppable() == true);

            Manifold mf = new Manifold(Manifold.Type.Circles, Vector2.zero, Vector2.up);
            contact.Foresee();
            contact.futur.Change(mf, 1f, true);
            Assert.That(contact.IsForeseen());
            Assert.That(contact.futur.tick, Is.EqualTo(3));
            Assert.That(contact.futur.manifold, Is.EqualTo(mf));
            Assert.That(contact.futur.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contact.futur.isTouching == true);
            Assert.That(contact.IsDroppable() == false);

            contact.Foresee(2);
            contact.futur.Change(mf, 2f, true);
            Assert.That(contact.IsForeseen());
            Assert.That(contact.futur.tick, Is.EqualTo(5));
            Assert.That(contact.futur.manifold, Is.EqualTo(mf));
            Assert.That(contact.futur.tangentSpeed, Is.EqualTo(2f));
            Assert.That(contact.futur.isTouching == true);
            Assert.That(contact.IsDroppable() == false);

            contact.Foresee();
            contact.futur.Change(null, 0f, false);
            Assert.That(contact.IsForeseen());
            Assert.That(contact.futur.tick, Is.EqualTo(6));
            Assert.That(contact.futur.manifold, Is.EqualTo(null));
            Assert.That(contact.futur.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.futur.isTouching == false);
            Assert.That(contact.IsDroppable() == false);

            contact.Step(2);
            Assert.That(contact.current.tick, Is.EqualTo(3));
            Assert.That(contact.current.manifold, Is.EqualTo(mf));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contact.current.isTouching == true);
            Assert.That(contact.past.tick, Is.EqualTo(0f));
            Assert.That(contact.past.manifold, Is.EqualTo(null));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.past.isTouching == false);
            Assert.That(contact.IsForeseen());
            Assert.That(contact.futur.tick, Is.EqualTo(6));
            Assert.That(contact.futur.manifold, Is.EqualTo(null));
            Assert.That(contact.futur.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.futur.isTouching == false);
            Assert.That(contact.IsDroppable() == false);

            contact.Step();
            Assert.That(contact.current.tick, Is.EqualTo(4));
            Assert.That(contact.current.manifold, Is.EqualTo(mf));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contact.current.isTouching == true);
            Assert.That(contact.IsForeseen() == true);
            Assert.That(contact.IsDroppable() == false);

            contact.Step();
            Assert.That(contact.current.tick, Is.EqualTo(5));
            Assert.That(contact.current.manifold, Is.EqualTo(mf));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(2f));
            Assert.That(contact.current.isTouching == true);
            Assert.That(contact.IsForeseen() == true);
            Assert.That(contact.IsDroppable() == false);

            contact.Step();
            Assert.That(contact.current.tick, Is.EqualTo(6));
            Assert.That(contact.current.manifold, Is.EqualTo(null));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.current.isTouching == false);
            Assert.That(contact.IsForeseen() == false);
            Assert.That(contact.IsDroppable() == false);

            contact.Foresee();
            contact.futur.Change(mf, 3f, true);
            Assert.That(contact.current.tick, Is.EqualTo(6));
            Assert.That(contact.current.manifold, Is.EqualTo(null));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.current.isTouching == false);
            Assert.That(contact.past.tick, Is.EqualTo(0f));
            Assert.That(contact.past.manifold, Is.EqualTo(null));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.past.isTouching == false);
            Assert.That(contact.IsForeseen());
            Assert.That(contact.futur.tick, Is.EqualTo(7));
            Assert.That(contact.futur.manifold, Is.EqualTo(mf));
            Assert.That(contact.futur.tangentSpeed, Is.EqualTo(3f));
            Assert.That(contact.futur.isTouching == true);
            Assert.That(contact.IsDroppable() == false);

            contact.Step();
            Assert.That(contact.current.tick, Is.EqualTo(7));
            Assert.That(contact.current.manifold, Is.EqualTo(mf));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(3f));
            Assert.That(contact.current.isTouching == true);
            Assert.That(contact.IsForeseen() == false);
            Assert.That(contact.IsDroppable() == false);

            contact.RollBack(4);
            Assert.That(contact.current.tick, Is.EqualTo(4));
            Assert.That(contact.current.manifold, Is.EqualTo(mf));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contact.current.isTouching == true);
            Assert.That(contact.past.tick, Is.EqualTo(0f));
            Assert.That(contact.past.manifold, Is.EqualTo(null));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.past.isTouching == false);
            Assert.That(contact.IsForeseen());
            Assert.That(contact.futur.tick, Is.EqualTo(7));
            Assert.That(contact.futur.manifold, Is.EqualTo(mf));
            Assert.That(contact.futur.tangentSpeed, Is.EqualTo(3f));
            Assert.That(contact.futur.isTouching == true);
            Assert.That(contact.IsDroppable() == false);

            contact.ClearFutur();
            Assert.That(contact.current.tick, Is.EqualTo(4));
            Assert.That(contact.current.manifold, Is.EqualTo(mf));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contact.current.isTouching == true);
            Assert.That(contact.past.tick, Is.EqualTo(0f));
            Assert.That(contact.past.manifold, Is.EqualTo(null));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.past.isTouching == false);
            Assert.That(contact.IsForeseen() == false);
            Assert.That(contact.IsDroppable() == false);

            contact.RollBack(2);
            Assert.That(contact.current.tick, Is.EqualTo(2));
            Assert.That(contact.current.manifold, Is.EqualTo(null));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.current.isTouching == false);
            Assert.That(contact.past.tick, Is.EqualTo(0f));
            Assert.That(contact.past.manifold, Is.EqualTo(null));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.past.isTouching == false);
            Assert.That(contact.IsForeseen() == true);
            Assert.That(contact.futur.tick, Is.EqualTo(3));
            Assert.That(contact.futur.manifold, Is.EqualTo(mf));
            Assert.That(contact.futur.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contact.futur.isTouching == true);
            Assert.That(contact.IsDroppable() == false);

            contact.RollBack(1);
            Assert.That(contact.current.tick, Is.EqualTo(1));
            Assert.That(contact.current.manifold, Is.EqualTo(null));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.current.isTouching == false);
            Assert.That(contact.past.tick, Is.EqualTo(0f));
            Assert.That(contact.past.manifold, Is.EqualTo(null));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.past.isTouching == false);
            Assert.That(contact.IsForeseen() == true);
            Assert.That(contact.futur.tick, Is.EqualTo(3));
            Assert.That(contact.futur.manifold, Is.EqualTo(mf));
            Assert.That(contact.futur.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contact.futur.isTouching == true);
            Assert.That(contact.IsDroppable() == false);

            contact.Step(2);
            contact.ForgetPast(contact.current.tick);
            Assert.That(contact.current.tick, Is.EqualTo(3));
            Assert.That(contact.current.manifold, Is.EqualTo(mf));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contact.current.isTouching == true);
            Assert.That(contact.past.tick, Is.EqualTo(3));
            Assert.That(contact.past.manifold, Is.EqualTo(mf));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contact.past.isTouching == true);
            Assert.That(contact.IsForeseen() == false);
            Assert.That(contact.IsDroppable() == false);

            contact.Step();
            contact.current.Change(null, 0f, false);
            contact.ForgetPast(contact.current.tick);
            Assert.That(contact.current.tick, Is.EqualTo(4));
            Assert.That(contact.current.manifold, Is.EqualTo(null));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.current.isTouching == false);
            Assert.That(contact.past.tick, Is.EqualTo(4));
            Assert.That(contact.past.manifold, Is.EqualTo(null));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.past.isTouching == false);
            Assert.That(contact.IsForeseen() == false);
            Assert.That(contact.IsDroppable() == true);
            contact.RollBack(0);
            Assert.That(contact.current.tick, Is.EqualTo(0));
            Assert.That(contact.current.manifold, Is.EqualTo(null));
            Assert.That(contact.current.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.current.isTouching == false);
            Assert.That(contact.past.tick, Is.EqualTo(0));
            Assert.That(contact.past.manifold, Is.EqualTo(null));
            Assert.That(contact.past.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.past.isTouching == false);
            Assert.That(contact.IsForeseen() == true);
            Assert.That(contact.futur.tick, Is.EqualTo(4));
            Assert.That(contact.futur.manifold, Is.EqualTo(null));
            Assert.That(contact.futur.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contact.futur.isTouching == false);
            Assert.That(contact.IsDroppable() == true);
        }

        [Test]
        public void EvaluatingCollisions()
        {
            //Circle With Circle

            CircleShape circle = new CircleShape(Vector2.zero, 1f);
            CircleShape otherCircle = new CircleShape(Vector2.zero, 1f);

            Body bodyA = new Body(
                0, 0, Vector2.zero, 0f, BodyType.Dynamic, circle,
                1f, 0f, 0f, 1f, 0.2f, 0.8f, false);
            Body bodyB = new Body(
                0, 0, Vector2.zero, 0f, BodyType.Dynamic, otherCircle,
                1f, 0f, 0f, 1f, 0.4f, 0.6f, false);

            Contact contact = ContactFactory.CreateContact(0, bodyA, bodyB);

            Manifold mf = contact.Evaluate(
                new Transformation(new Vector2(1f, 1f), 0f),
                new Transformation(new Vector2(-1f, 1f), 0f));

            Assert.That(mf != null);
            Assert.That(mf.type, Is.EqualTo(Manifold.Type.Circles));
            Assert.That(mf.point, Is.EqualTo(Vector2.zero));
            Assert.That(mf.normal, Is.EqualTo(Vector2.zero));
            Assert.That(mf.pointCount, Is.EqualTo(1));
            Assert.That(mf.points[0].point, Is.EqualTo(Vector2.zero));

            mf = contact.Evaluate(
                new Transformation(new Vector2(2f, 1f), 0f),
                new Transformation(new Vector2(-2f, 1f), 0f));

            Assert.That(mf == null);

            //Circle With Polygon
            PolygonShape polygon = new PolygonShape();
            polygon.SetAsBox(1f, 1f);

            bodyB = new Body(
                0, 0, Vector2.zero, 0f, BodyType.Dynamic, polygon,
                1f, 0f, 0f, 1f, 0.4f, 0.6f, false);

            contact = ContactFactory.CreateContact(0, bodyA, bodyB);

            mf = contact.Evaluate(
                new Transformation(new Vector2(1f, 1f), 0f),
                new Transformation(new Vector2(-1f, 1f), 0f));

            Assert.That(mf != null);
            Assert.That(mf.type, Is.EqualTo(Manifold.Type.FaceA));
            Assert.That(mf.point, Is.EqualTo(Vector2.left));
            Assert.That(mf.normal, Is.EqualTo(Vector2.left));
            Assert.That(mf.pointCount, Is.EqualTo(1));
            Assert.That(mf.points[0].point, Is.EqualTo(Vector2.zero));

            mf = contact.Evaluate(
                new Transformation(new Vector2(-2f, 1f), 0f),
                new Transformation(new Vector2(2f, 1f), 0f));

            Assert.That(mf == null);

            //Circle With Edge
            EdgeShape edge = new EdgeShape(new Vector2(0f, -1f), new Vector2(0f, 1f));
            bodyB = new Body(
                0, 0, Vector2.zero, 0f, BodyType.Dynamic, edge,
                1f, 0f, 0f, 1f, 0.4f, 0.6f, false);

            contact = ContactFactory.CreateContact(0, bodyA, bodyB);

            mf = contact.Evaluate(
                new Transformation(new Vector2(1f, 1f), 0f),
                new Transformation(new Vector2(0f, 1f), 0f));

            Assert.That(mf != null);
            Assert.That(mf.type, Is.EqualTo(Manifold.Type.FaceA));
            Assert.That(mf.point, Is.EqualTo(new Vector2(0f, -1f)));
            Assert.That(mf.normal, Is.EqualTo(Vector2.left));
            Assert.That(mf.pointCount, Is.EqualTo(1));
            Assert.That(mf.points[0].point, Is.EqualTo(Vector2.zero));
            Assert.That(mf.points[0].id.feature.indexA, Is.EqualTo(0));
            Assert.That(mf.points[0].id.feature.typeA, Is.EqualTo((byte)ContactFeature.Type.Face));

            mf = contact.Evaluate(
               new Transformation(new Vector2(1f, 1f), 0f),
               new Transformation(new Vector2(0f, 2f), 0f));

            Assert.That(mf != null);
            Assert.That(mf.type, Is.EqualTo(Manifold.Type.Circles));
            Assert.That(mf.point, Is.EqualTo(new Vector2(0f, 1f)));
            Assert.That(mf.normal, Is.EqualTo(Vector2.zero));
            Assert.That(mf.pointCount, Is.EqualTo(1));
            Assert.That(mf.points[0].point, Is.EqualTo(Vector2.zero));
            Assert.That(mf.points[0].id.feature.indexA, Is.EqualTo(1));
            Assert.That(mf.points[0].id.feature.typeA, Is.EqualTo((byte)ContactFeature.Type.Vertex));

            mf = contact.Evaluate(
               new Transformation(new Vector2(1f, 1f), 0f),
               new Transformation(new Vector2(0f, 0f), 0f));

            Assert.That(mf != null);
            Assert.That(mf.type, Is.EqualTo(Manifold.Type.Circles));
            Assert.That(mf.point, Is.EqualTo(new Vector2(0f, -1f)));
            Assert.That(mf.normal, Is.EqualTo(Vector2.zero));
            Assert.That(mf.pointCount, Is.EqualTo(1));
            Assert.That(mf.points[0].point, Is.EqualTo(Vector2.zero));
            Assert.That(mf.points[0].id.feature.indexA, Is.EqualTo(0));
            Assert.That(mf.points[0].id.feature.typeA, Is.EqualTo((byte)ContactFeature.Type.Vertex));

            mf = contact.Evaluate(
               new Transformation(new Vector2(2f, 1f), 0f),
               new Transformation(new Vector2(-1f, 1f), 0f));

            Assert.That(mf == null);
        }

    }
}