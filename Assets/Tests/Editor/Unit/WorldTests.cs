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
			IWorld world = new World(1f, 0.1f);

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

			world.Step(0.5f, 8, 3);
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
			
		}
	}
}
