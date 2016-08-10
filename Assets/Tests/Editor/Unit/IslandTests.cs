using NUnit.Framework;
using NSubstitute;
using UnityEngine;
using System;

namespace CrispyPhysics
{
    using Internal;
    [TestFixture]
    public class IslandTests
    {
        [Test]
        public void SettingUpIsland()
        {
            Assert.Throws<ArgumentException>(
                delegate { new Island(0, 0); });

            IInternalBody body1 = Substitute.For<IInternalBody>();
            IInternalBody body2 = Substitute.For<IInternalBody>();
            IContact contact1 = Substitute.For<IContact>();
            IContact contact2 = Substitute.For<IContact>();

            Island island = new Island();

            island.Add(body1);
            Assert.Throws<InvalidOperationException>(() => island.Add(body2));

            island.Add(contact1);
            Assert.Throws<InvalidOperationException>(() => island.Add(contact2));

            island.Clear();
            Assert.DoesNotThrow(() => island.Add(body2));
            Assert.DoesNotThrow(() => island.Add(contact2));
        }

        [Test]
        public void SolvingBody()
        {
            Island island = new Island();

            IInternalMomentum futurMomentum = Substitute.For<IInternalMomentum>();

            futurMomentum.force.Returns(Vector2.one);
            futurMomentum.torque.Returns(1f);
            futurMomentum.linearVelocity.Returns(Vector2.zero);
            futurMomentum.angularVelocity.Returns(0f);
            futurMomentum.position.Returns(Vector2.zero);
            futurMomentum.angle.Returns(0f);

            IInternalBody body = Substitute.For<IInternalBody>();
            island.Add(body);

            body.invMass.Returns(1f);
            body.linearDamping.Returns(0.2f);
            body.angularDamping.Returns(0.2f);
            body.gravityScale.Returns(1f);
            body.futur.Returns(futurMomentum);

            TimeStep step = new TimeStep();
            step.dt = 1f;
            step.velocityIterations = 8;
            step.positionIterations = 3;
            step.invDt = 1f / step.dt;
            step.dtRatio = 0f;

            
            island.Solve(step, new Vector2(0f, -9.8f));

            body.Received(1).Foresee(
                Arg.Is<float>(x => Calculus.Approximately(x, 1f, 0)));

            futurMomentum.Received(1).ChangeSituation(
                Arg.Is<Vector2>(
                    v =>    Calculus.Approximately(v.x, 0.833f, 0.001f)
                        &&  Calculus.Approximately(v.y, -7.333f, 0.001f)), 
                Arg.Is<float>(x => Calculus.Approximately(x, 0.833f, 0.001f)));

            futurMomentum.Received(1).ChangeVelocity(
                Arg.Is<Vector2>(
                    v =>    Calculus.Approximately(v.x, 0.833f, 0.001f)
                        &&  Calculus.Approximately(v.y, -7.333f, 0.001f)),
                Arg.Is<float>(x => Calculus.Approximately(x, 0.833f, 0.001f)));

        }
    }
}