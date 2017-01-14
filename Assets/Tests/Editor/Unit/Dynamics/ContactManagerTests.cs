using NUnit.Framework;
using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics
{
    using Internal;
    [TestFixture]
    public class ContactManagerTests
    {
        private List<Body> bodies;
        private List<Contact> contacts;

        private IEnumerable<Body> BodyIterator(uint start = 0, uint end = 0)
        {
            if (end == 0)
                end = (uint)bodies.Count;
            for (int i = (int)start; i < (int)end; i++)
                yield return bodies[i];
        }

        private IEnumerable<Contact> ContactIterator()
        {
            foreach (Contact contact in contacts)
                yield return contact;
        }

        private void InitBodyContainer()
        {
            bodies = new List<Body>();
        }

        private void InitContactContainer()
        {
            contacts = new List<Contact>();
        }

        [Test]
        public void usingContactManager()
        {
            IShape circleShape = new CircleShape(Vector2.zero, 1f);
            IShape edgeShape = new EdgeShape(Vector2.left, Vector2.right);
            IShape polygonShape = ShapeFactory.CreateBox(1f, 1f);

            Body circleBody = new Body(
                0, 0, new Vector2(0.5f, 1f), 0f, BodyType.Dynamic, circleShape,
                1f, 0f, 0f, 1f, 0.2f, 0.8f, false);

            circleBody.futur.ChangeVelocity(Vector2.one * -10, 0.5f);

            Body edgeBody = new Body(
               1, 0, Vector2.zero, 0f, BodyType.Static, edgeShape,
               1f, 0f, 0f, 1f, 0.4f, 0.6f, false);

            Body sensorEdgeBody = new Body(
               1, 0, new Vector2(0f, 2f), 0f, BodyType.Static, edgeShape,
               1f, 0f, 0f, 1f, 0.4f, 0.6f, true);

            Body polygonBody = new Body(
                2, 0, new Vector2(10.0f, 10.0f), 0f, BodyType.Dynamic, polygonShape);

            Body ghostBody = new Body(2, 0, Vector2.zero, 0f, BodyType.Dynamic, null);



            InitBodyContainer();
            bodies.Add(circleBody);
            bodies.Add(edgeBody);
            bodies.Add(sensorEdgeBody);
            bodies.Add(ghostBody);
            bodies.Add(polygonBody);

            InitContactContainer();

            NewPairDelegate pairDel = (bodyA, bodyB) => contacts.Add(
                ContactFactory.CreateContact(bodyA.futur.tick, bodyA, bodyB));

            bool contactStartForeseen = false;
            ContactHandlerDelegate contactStartForeseenDel = (contact, args) => contactStartForeseen = true;

            bool contactEndForeseen = false;
            ContactHandlerDelegate contactEndForeseenDel = (contact, args) => contactEndForeseen = true;

            Assert.Throws<ArgumentNullException>(
               delegate { new ContactManager(null, null, null); });

            Assert.Throws<ArgumentNullException>(
               delegate { new ContactManager(new BodyIteratorDelegate(BodyIterator), null, null); });

            Assert.Throws<ArgumentNullException>(
               delegate { new ContactManager(new BodyIteratorDelegate(BodyIterator), pairDel, null); });

            ContactManager cm = new ContactManager(
                new BodyIteratorDelegate(BodyIterator),
                pairDel,
                new ContactIteratorDelegate(ContactIterator),
                contactStartForeseenDel,
                contactEndForeseenDel
                );

            cm.FindNewContacts();

            Assert.That(contactStartForeseen == false);
            Assert.That(contactEndForeseen == false);

            Assert.That(contacts.Count == 2);

            Assert.That(contacts[0].bodyA, Is.EqualTo(edgeBody));
            Assert.That(contacts[0].bodyB, Is.EqualTo(circleBody));
            Assert.That(contacts[0].friction, Is.EqualTo(0.282f).Within(0.001f));
            Assert.That(contacts[0].restitution, Is.EqualTo(0.8f).Within(0.001f));
            Assert.That(contacts[0].restitution, Is.EqualTo(0.8f).Within(0.001f));
            Assert.That(contacts[0].futur.tick, Is.EqualTo(0u));
            Assert.That(contacts[0].futur.manifold == null);
            Assert.That(contacts[0].futur.tangentSpeed, Is.EqualTo(0f).Within(0.001f));
            Assert.That(contacts[0].futur.isTouching == false);

            Assert.That(contacts[1].bodyA, Is.EqualTo(sensorEdgeBody));
            Assert.That(contacts[1].bodyB, Is.EqualTo(circleBody));
            Assert.That(contacts[1].friction, Is.EqualTo(0.282f).Within(0.001f));
            Assert.That(contacts[1].restitution, Is.EqualTo(0.8f).Within(0.001f));
            Assert.That(contacts[1].restitution, Is.EqualTo(0.8f).Within(0.001f));
            Assert.That(contacts[1].futur.tick, Is.EqualTo(0u));
            Assert.That(contacts[1].futur.manifold == null);
            Assert.That(contacts[1].futur.tangentSpeed, Is.EqualTo(0f).Within(0.001f));
            Assert.That(contacts[1].futur.isTouching == false);

            cm.Collide();
            Assert.That(contactStartForeseen == true);
            Assert.That(contactEndForeseen == false);

            Assert.That(contacts[0].futur.tick, Is.EqualTo(0u));
            Assert.That(contacts[0].futur.manifold != null);
            Assert.That(contacts[0].futur.tangentSpeed, Is.EqualTo(0f).Within(0.001f));
            Assert.That(contacts[0].futur.isTouching == true);
            Assert.That(contacts[0].bodyA.futur.enduringContact == true);
            Assert.That(contacts[0].bodyB.futur.enduringContact == true);

            Assert.That(contacts[1].futur.tick, Is.EqualTo(0u));
            Assert.That(contacts[1].futur.manifold == null);
            Assert.That(contacts[1].futur.tangentSpeed, Is.EqualTo(0f).Within(0.001f));
            Assert.That(contacts[1].futur.isTouching == true);
            Assert.That(contacts[1].bodyA.futur.enduringContact == true);
            Assert.That(contacts[1].bodyB.futur.enduringContact == true);

            circleBody.ChangeSituation(new Vector2(-10f, -10f), 0f);

            cm.Collide();
            Assert.That(contactStartForeseen == true);
            Assert.That(contactEndForeseen == true);

            Assert.That(contacts[0].futur.tick, Is.EqualTo(0u));
            Assert.That(contacts[0].futur.manifold == null);
            Assert.That(contacts[0].futur.tangentSpeed, Is.EqualTo(0f).Within(0.001f));
            Assert.That(contacts[0].futur.isTouching == false);
            Assert.That(contacts[0].bodyA.futur.enduringContact == false);
            Assert.That(contacts[0].bodyB.futur.enduringContact == false);

            Assert.That(contacts[1].futur.tick, Is.EqualTo(0u));
            Assert.That(contacts[1].futur.manifold == null);
            Assert.That(contacts[1].futur.tangentSpeed, Is.EqualTo(0f).Within(0.001f));
            Assert.That(contacts[1].futur.isTouching == false);
            Assert.That(contacts[1].bodyA.futur.enduringContact == false);
            Assert.That(contacts[1].bodyB.futur.enduringContact == false);
        }
    }
}
