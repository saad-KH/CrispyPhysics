using NUnit.Framework;
using NSubstitute;
using UnityEngine;

namespace CrispyPhysics
{
    using Internal;
    [TestFixture]
    public class ContactMomenntumTests
    {
        [Test]
        public void CreatingContactMomentums()
        {
            ContactMomentum contactMomentum = new ContactMomentum(0);

            Assert.That(contactMomentum.tick, Is.EqualTo(0));
            Assert.That(contactMomentum.manifold, Is.EqualTo(null));
            Assert.That(contactMomentum.tangentSpeed, Is.EqualTo(0f));
            Assert.That(contactMomentum.isTouching == false);

            Manifold mf = new Manifold(Manifold.Type.Circles, Vector2.zero, Vector2.up);
            contactMomentum = new ContactMomentum(1, mf, 1f, true);

            Assert.That(contactMomentum.tick, Is.EqualTo(1));
            Assert.That(contactMomentum.manifold, Is.EqualTo(mf));
            Assert.That(contactMomentum.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contactMomentum.isTouching);

            ContactMomentum newContactMomentum = new ContactMomentum(2, contactMomentum);

            Assert.That(newContactMomentum.tick, Is.EqualTo(2));
            Assert.That(newContactMomentum.manifold, Is.EqualTo(mf));
            Assert.That(newContactMomentum.tangentSpeed, Is.EqualTo(1f));
            Assert.That(newContactMomentum.isTouching);
        }


        [Test]
        public void ChangingOperativeValues()
        {
            ContactMomentum contactMomentum = new ContactMomentum(0);

            Manifold mf = new Manifold(Manifold.Type.Circles, Vector2.zero, Vector2.up);
            contactMomentum.Change(mf, 1f, true);

            Assert.That(contactMomentum.tick, Is.EqualTo(0));
            Assert.That(contactMomentum.manifold, Is.EqualTo(mf));
            Assert.That(contactMomentum.tangentSpeed, Is.EqualTo(1f));
            Assert.That(contactMomentum.isTouching);
        }

        [Test]
        public void ComparingMomentums()
        {
            ContactMomentum contactMomentum = new ContactMomentum(0);

            Manifold mf = new Manifold(Manifold.Type.Circles, Vector2.zero, Vector2.up);
            contactMomentum.Change(mf, 1f, true);

            ContactMomentum sameMomentum = new ContactMomentum(1, contactMomentum);
            Assert.That(sameMomentum.Same(contactMomentum));

            contactMomentum.Change(mf, 2f, true);
            Assert.That(sameMomentum.Same(contactMomentum) == false);

        }
    }
}
