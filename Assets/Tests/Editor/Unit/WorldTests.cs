using NUnit.Framework;
using NSubstitute;
using UnityEngine;
using System;

namespace CrispyPhysics
{
	[TestFixture]
	public class WorldTests
	{
		[Test]
		public void SettingUpWorld()
		{
			IWorld world = new World();

			Assert.That(world.GetAutoClearForces() == true);
			Assert.That(world.IsCrisped() == false);
			Assert.That(world.IsLocked() == false);

			world.SetAutoClearForces(false);
			Assert.That(world.GetAutoClearForces() == false);
			world.SetAutoClearForces(true);
			Assert.That(world.GetAutoClearForces() == true);

			IBody body = Substitute.For<IBody>();
			world.Add(body);

			world.SetAllowSleeping(false);
			world.SetAllowSleeping(true);
			body.Received(1).SetAwake(true);

			world.ClearForces();
			body.Received(1).ChangeImpulse(Vector2.zero, 0f);
		}

		[Test]
		public void SteppingWorld()
		{
			IWorld world = new World(0.1f, 0.1f);

			IBody body = Substitute.For<IBody>();
			world.Add(body);
			body.IsAwake().Returns(true);
			body.IsActive().Returns(true);

			IBody sleepingBody = Substitute.For<IBody>();
			world.Add(sleepingBody);
			sleepingBody.IsAwake().Returns(false);

			IBody inactiveBody = Substitute.For<IBody>();
			world.Add(inactiveBody);
			inactiveBody.IsActive().Returns(false);

			world.Step(0.05f, 8, 3);
            Assert.That(world.tick, Is.EqualTo(0.05f).Within(0.001f));
            body.Received(1).SetIslandBound(false);
			body.Received(1).SetIslandBound(true);
			body.Received(1).SetAwake(true);

			sleepingBody.Received(1).SetIslandBound(false);
			sleepingBody.Received(0).SetIslandBound(true);

			inactiveBody.Received(1).SetIslandBound(false);
			inactiveBody.Received(0).SetIslandBound(true);
		}

		[Test]
		public void CrispingWorld()
		{
            IWorld world = new World(0.1f, 0.1f);

            IBody body = Substitute.For<IBody>();
            world.Add(body);
            body.IsAwake().Returns(true);
            body.IsActive().Returns(true);
            body.mass.Returns(1f);
            body.gravityScale.Returns(1f);
            body.angularVelocity.Returns(1f);
            body.linearDamping.Returns(0.2f);
            body.angularDamping.Returns(0.2f);

            body
                .When(x => x.ChangeSituation(Arg.Any<Vector2>(), Arg.Any<float>()))
                .Do(x => {
                    body.position.Returns(x.Arg<Vector2>());
                    body.angle.Returns(x.Arg<float>());});

            body
                .When(x => x.ChangeVelocity(Arg.Any<Vector2>(), Arg.Any<float>()))
                .Do(x => {
                    body.linearVelocity.Returns(x.Arg<Vector2>());
                    body.angularVelocity.Returns(x.Arg<float>());
                });

            Physics2D.gravity = new Vector2(10f, -10f);

            world.Step(0.05f, 8, 3);

            Assert.That(world.IsCrisped() == false);
            Assert.That(world.tickRatio, Is.EqualTo(0.5f).Within(0.001f));

            body.Received(1).ChangeSituation(
                Arg.Is<Vector2>(
                    v =>    Mathf.Abs(v.x - 0.024f) < 0.001f
                        &&  Mathf.Abs(v.y - -0.024f) < 0.001f),
                Arg.Is<float>(x => Mathf.Abs(x - 0.049f) < 0.001f));

            body.Received(1).ChangeVelocity(
                Arg.Is<Vector2>(
                    v =>    Mathf.Abs(v.x - 0.495f) < 0.001f
                        &&  Mathf.Abs(v.y - -0.495f) < 0.001f),
                Arg.Is<float>(x => Mathf.Abs(x - 0.990f) < 0.001f));

            body.ClearReceivedCalls();
            world.Step(0.05f, 8, 3);

            Assert.That(world.IsCrisped() == true);
            Assert.That(world.tickRatio, Is.EqualTo(0f).Within(0.001f));

            body.Received(1).ChangeSituation(
                Arg.Is<Vector2>(
                    v => Mathf.Abs(v.x - 0.074f) < 0.001f
                        && Mathf.Abs(v.y - -0.074f) < 0.001f),
                Arg.Is<float>(x => Mathf.Abs(x - 0.098f) < 0.001f));

            body.Received(1).ChangeSituation(
                Arg.Is<Vector2>(
                    v => Mathf.Abs(v.x - 0.1f) < 0.001f
                        && Mathf.Abs(v.y - -0.1f) < 0.001f),
                Arg.Is<float>(x => Mathf.Abs(x - 0.098f) < 0.001f));

            body.Received(1).ChangeVelocity(
                Arg.Is<Vector2>(
                    v => Mathf.Abs(v.x - 0.985f) < 0.001f
                        && Mathf.Abs(v.y - -0.985f) < 0.001f),
                Arg.Is<float>(x => Mathf.Abs(x - 0.980f) < 0.001f));

            body.ClearReceivedCalls();
            world.Step(0.05f, 8, 3);

            Assert.That(world.IsCrisped() == false);
            Assert.That(world.tickRatio, Is.EqualTo(0.5f).Within(0.001f));

            body.Received(1).ChangeSituation(
                Arg.Is<Vector2>(
                    v => Mathf.Abs(v.x - 0.173f) < 0.001f
                        && Mathf.Abs(v.y - -0.173f) < 0.001f),
                Arg.Is<float>(x => Mathf.Abs(x - 0.147f) < 0.001f));

            body.Received(1).ChangeVelocity(
                Arg.Is<Vector2>(
                    v => Mathf.Abs(v.x - 1.470f) < 0.001f
                        && Mathf.Abs(v.y - -1.470f) < 0.001f),
                Arg.Is<float>(x => Mathf.Abs(x - 0.970f) < 0.001f));
        }
	}
}
