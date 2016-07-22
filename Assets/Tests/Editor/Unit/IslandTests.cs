using NUnit.Framework;
using NSubstitute;
using UnityEngine;
using System;

namespace CrispyPhysics
{
    [TestFixture]
    public class IslandTests
    {
        [Test]
        public void SettingUpIslands()
        {
            Assert.Throws<ArgumentException>(
                delegate { new Island(0, 0); });

            IBody body1 = Substitute.For<IBody>();
            IBody body2 = Substitute.For<IBody>();
            IContact contact1 = Substitute.For<IContact>();
            IContact contact2 = Substitute.For<IContact>();

            IIsland island = new Island();

            island.Add(body1);
            Assert.Throws<InvalidOperationException>(
               () => island.Add(body2),
               "Body Capacity should not be overflown");

            island.Add(contact1);
            Assert.Throws<InvalidOperationException>(
               () => island.Add(contact2),
               "Contact Capacity should not be overflown");

            island.Clear();
            Assert.DoesNotThrow(
               () => island.Add(body2),
               "Body Capacity has been cleared");
            Assert.DoesNotThrow(
               () => island.Add(contact2),
               "Contact Capacity has been cleared");
        }

        [Test]
        public void SolvingBody()
        {
            IIsland island = new Island();

            IBody body = Substitute.For<IBody>();
            island.Add(body);

            body.invMass.Returns(1f);
            body.force.Returns(Vector2.one);
            body.torque.Returns(1f);
            body.linearDamping.Returns(0.2f);
            body.angularDamping.Returns(0.2f);
            body.gravityScale.Returns(1f);

            TimeStep step = new TimeStep();
            step.dt = 1f;
            step.velocityIterations = 8;
            step.positionIterations = 3;
            step.invDt = 1f / step.dt;
            step.dtRatio = 0f;

            
            island.Solve(step, new Vector2(0f, -9.8f), false);
            body.Received(1).ChangeSituation(
                Arg.Is<Vector2>(
                    v =>    Mathf.Abs(v.x - 0.833f) < 0.001f 
                        &&  Mathf.Abs(v.y - -7.333f) < 0.001f), 
                Arg.Is<float>(x => Mathf.Abs(x - 0.833f) < 0.001f));

            body.Received(1).ChangeVelocity(
                Arg.Is<Vector2>(
                    v => Mathf.Abs(v.x - 0.833f) < 0.001f
                        && Mathf.Abs(v.y - -7.333f) < 0.001f),
                Arg.Is<float>(x => Mathf.Abs(x - 0.833f) < 0.001f));

        }

        [Test]
        public void SolvingSleepingBody()
        {
            IIsland island = new Island();
            
            TimeStep step = new TimeStep();
            step.dt = Physics2D.timeToSleep + 1f;
            step.velocityIterations = 8;
            step.positionIterations = 3;
            step.invDt = 1f / step.dt;
            step.dtRatio = 0f;

            IBody body = Substitute.For<IBody>();
            island.Add(body);

            body.invMass.Returns(1f);
            body.IsSleepingAllowed().Returns(true);

            //Sleeping Body
            island.Solve(step, Vector2.zero, true);
            body.Received(1).SetAwake(false);
            body.Received(1).sleepTime = step.dt;


            //Awaking Body
            body.ClearReceivedCalls();

            body.linearVelocity.Returns(Vector2.one);
            body.angularVelocity.Returns(1f);

            island.Solve(step, Vector2.zero, true);
            body.Received(1).sleepTime = 0f;
        }
    }
}
