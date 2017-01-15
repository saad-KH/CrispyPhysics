using NUnit.Framework;
using NSubstitute;
using UnityEngine;
using System;

namespace CrispyPhysics
{
    using Internal;
    [TestFixture]
	public class WorldTests
	{
		[Test]
		public void SettingUpWorld()
		{
			World world = new World(0.01f, new Vector2(-10f, -10f));

			Assert.That(world.fixedStep, Is.EqualTo(0.01f));
            Assert.That(world.gravity, Is.EqualTo(new Vector2(-10f, -10f)));
            Assert.That(world.velocityIterations, Is.EqualTo(8));
            Assert.That(world.positionIterations, Is.EqualTo(3));
            Assert.That(world.maxTranslationSpeed, Is.EqualTo(100));
            Assert.That(world.maxRotationSpeed, Is.EqualTo(360));
            Assert.That(world.tick, Is.EqualTo(0));
            Assert.That(world.pastTick, Is.EqualTo(0));
            Assert.That(world.futurTick, Is.EqualTo(0));

            WorldDefinition worldDef = new WorldDefinition(
                0.1f,
                new Vector2(0f, -10f), 5, 5,
                50, 180);

            World specificWorld = new World(worldDef);

            Assert.That(specificWorld.fixedStep, Is.EqualTo(0.1f));
            Assert.That(specificWorld.gravity, Is.EqualTo(new Vector2(0f, -10f)));
            Assert.That(specificWorld.velocityIterations, Is.EqualTo(5));
            Assert.That(specificWorld.positionIterations, Is.EqualTo(5));
            Assert.That(specificWorld.maxTranslationSpeed, Is.EqualTo(50));
            Assert.That(specificWorld.maxRotationSpeed, Is.EqualTo(180));
            Assert.That(specificWorld.tick, Is.EqualTo(0));
            Assert.That(specificWorld.pastTick, Is.EqualTo(0));
            Assert.That(specificWorld.futurTick, Is.EqualTo(0));

        }

        [Test]
        public void SteppingWorld()
        {
            World world = new World(0.01f, new Vector2(-10f, -10f));

            Body body = 
                    world.CreateBody(Vector2.zero, 0f, BodyType.Dynamic, 
                    ShapeFactory.CreateCircle(1f),
                    1f, 0.2f, 0.2f, 1f, 0.2f, 0.95f)
                as Body;

            Body colliderBody =
                    world.CreateBody(new Vector2(-2.5f, -2.5f), 0f, BodyType.Static,
                    ShapeFactory.CreateEdge(new Vector2(-2f, 2f), new Vector2(2f, -2f)),
                    1, 0, 0, 1, 0, 0.8f)
                as Body;

            world.Step();
            Assert.That(world.tick, Is.EqualTo(1));
            Assert.That(world.pastTick, Is.EqualTo(1));
            Assert.That(world.futurTick, Is.EqualTo(1));

            world.Step(3);
            Assert.That(world.tick, Is.EqualTo(4));
            Assert.That(world.pastTick, Is.EqualTo(4));
            Assert.That(world.futurTick, Is.EqualTo(4));

            world.Step(3, 50, 10 , 50);
            Assert.That(world.tick, Is.EqualTo(7));
            Assert.That(world.pastTick, Is.EqualTo(4));
            Assert.That(world.futurTick, Is.EqualTo(17));

            while (world.tick < 100)
                world.Step(1, 50, 10, 50);

            Assert.That(world.tick, Is.EqualTo(100));
            Assert.That(world.pastTick, Is.EqualTo(50));
            Assert.That(world.futurTick, Is.EqualTo(150));

            Assert.That(body.current.tick, Is.EqualTo(100));
            Assert.That(body.current.tickDt, Is.EqualTo(0.01f));
            Assert.That(
                body.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(1.473f, 1.473f)).Within(0.001f));
            Assert.That(
                body.position,
                OwnNUnit.Is.EqualTo(new Vector2(-0.457f, -0.457f)).Within(0.001f));
            Assert.That(body.enduringContact == false);

            Assert.That(body.past.tick, Is.EqualTo(world.pastTick));
            Assert.That(body.past.tickDt, Is.EqualTo(0.01f));
            Assert.That(
                body.past.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-4.753f, -4.753f)).Within(0.001f));
            Assert.That(
                body.past.position,
                OwnNUnit.Is.EqualTo(new Vector2(-1.231f, -1.231f)).Within(0.001f));
            Assert.That(body.past.enduringContact == false);

            Assert.That(body.futur.tick, Is.EqualTo(world.futurTick));
            Assert.That(body.futur.tickDt, Is.EqualTo(0.01f));
            Assert.That(
                body.futur.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-3.419f, -3.419f)).Within(0.001f));
            Assert.That(
                body.futur.position,
                OwnNUnit.Is.EqualTo(new Vector2(-0.988f, -0.988f)).Within(0.001f));
            Assert.That(body.futur.enduringContact == false);

            Assert.That(body.MomentumForTick(62).enduringContact == true);

            Assert.That(colliderBody.current.tick, Is.EqualTo(100));
            Assert.That(
                colliderBody.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(0f, 0f)).Within(0.001f));
            Assert.That(
                colliderBody.position,
                OwnNUnit.Is.EqualTo(new Vector2(-2.5f, -2.5f)).Within(0.001f));
            Assert.That(colliderBody.enduringContact == false);

            Assert.That(colliderBody.past.tick, Is.EqualTo(world.pastTick));
            Assert.That(
                colliderBody.past.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(0f, 0f)).Within(0.001f));
            Assert.That(
                colliderBody.past.position,
                OwnNUnit.Is.EqualTo(new Vector2(-2.5f, -2.5f)).Within(0.001f));
            Assert.That(colliderBody.past.enduringContact == false);

            Assert.That(colliderBody.futur.tick, Is.EqualTo(world.futurTick));
            Assert.That(
                colliderBody.linearVelocity,
                 OwnNUnit.Is.EqualTo(new Vector2(0f, 0f)).Within(0.001f));
            Assert.That(
                colliderBody.futur.position,
                OwnNUnit.Is.EqualTo(new Vector2(-2.5f, -2.5f)).Within(0.001f));
            Assert.That(colliderBody.futur.enduringContact == false);

            Assert.That(colliderBody.MomentumForTick(62).enduringContact == true);

            world.RollBack(50);
            Assert.That(world.tick, Is.EqualTo(50));
            Assert.That(world.pastTick, Is.EqualTo(50));
            Assert.That(world.futurTick, Is.EqualTo(150));

            Assert.That(body.current.tick, Is.EqualTo(50));
            Assert.That(
                body.current.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-4.753f, -4.753f)).Within(0.001f));
            Assert.That(
                body.current.position,
                OwnNUnit.Is.EqualTo(new Vector2(-1.231f, -1.231f)).Within(0.001f));

            Assert.That(body.past.tick, Is.EqualTo(50));
            Assert.That(
                body.past.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-4.753f, -4.753f)).Within(0.001f));
            Assert.That(
                body.past.position,
                OwnNUnit.Is.EqualTo(new Vector2(-1.231f, -1.231f)).Within(0.001f));

            Assert.That(body.futur.tick, Is.EqualTo(150));
            Assert.That(
                body.futur.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-3.419f, -3.419f)).Within(0.001f));
            Assert.That(
                body.futur.position,
                OwnNUnit.Is.EqualTo(new Vector2(-0.988f, -0.988f)).Within(0.001f));

            world.Step(100);
            Assert.That(world.tick, Is.EqualTo(150));
            Assert.That(world.pastTick, Is.EqualTo(150));
            Assert.That(world.futurTick, Is.EqualTo(150));

            Assert.That(body.current.tick, Is.EqualTo(150));
            Assert.That(
                body.current.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-3.419f, -3.419f)).Within(0.001f));
            Assert.That(
                body.current.position,
                OwnNUnit.Is.EqualTo(new Vector2(-0.988f, -0.988f)).Within(0.001f));
            Assert.That(body.current.enduringContact == false);

            Assert.That(body.past.tick, Is.EqualTo(150));
            Assert.That(
                body.past.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-3.419f, -3.419f)).Within(0.001f));
            Assert.That(
                body.past.position,
                OwnNUnit.Is.EqualTo(new Vector2(-0.988f, -0.988f)).Within(0.001f));
            Assert.That(body.past.enduringContact == false);

            Assert.That(body.futur.tick, Is.EqualTo(150));
            Assert.That(
                body.futur.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-3.419f, -3.419f)).Within(0.001f));
            Assert.That(
                body.futur.position,
                OwnNUnit.Is.EqualTo(new Vector2(-0.988f, -0.988f)).Within(0.001f));
            Assert.That(body.futur.enduringContact == false);


            while (world.tick < 200)
                world.Step(1, 50, 10, 50);

            Assert.That(body.current.tick, Is.EqualTo(200));
            Assert.That(
                body.current.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(1.783f, 1.783f)).Within(0.001f));
            Assert.That(
                body.current.position,
                OwnNUnit.Is.EqualTo(new Vector2(-0.768f, -0.768f)).Within(0.001f));
            Assert.That(body.current.enduringContact == false);

            Assert.That(body.past.tick, Is.EqualTo(150));
            Assert.That(
                body.past.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-3.419f, -3.419f)).Within(0.001f));
            Assert.That(
                body.past.position,
                OwnNUnit.Is.EqualTo(new Vector2(-0.988f, -0.988f)).Within(0.001f));
            Assert.That(body.past.enduringContact == false);

            Assert.That(body.futur.tick, Is.EqualTo(250));
            Assert.That(
                body.futur.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-3.139f, -3.139f)).Within(0.001f));
            Assert.That(
                body.futur.position,
                OwnNUnit.Is.EqualTo(new Vector2(-1.153f, -1.153f)).Within(0.001f));
            Assert.That(body.futur.enduringContact == false);

            Assert.That(body.MomentumForTick(170).enduringContact == true);

            body.ChangeSituation(Vector2.zero, 0f);
            Assert.That(world.tick, Is.EqualTo(200));
            Assert.That(world.pastTick, Is.EqualTo(150));
            Assert.That(world.futurTick, Is.EqualTo(200));

            Assert.That(body.current.tick, Is.EqualTo(200));
            Assert.That(
                body.current.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(1.783f, 1.783f)).Within(0.001f));
            Assert.That(
                body.current.position,
                OwnNUnit.Is.EqualTo(new Vector2(0f, 0f)).Within(0.001f));

            Assert.That(body.past.tick, Is.EqualTo(150));
            Assert.That(
                body.past.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-3.419f, -3.419f)).Within(0.001f));
            Assert.That(
                body.past.position,
                OwnNUnit.Is.EqualTo(new Vector2(-0.988f, -0.988f)).Within(0.001f));

            Assert.That(body.futur.tick, Is.EqualTo(250));
            Assert.That(
                body.futur.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(-3.139f, -3.139f)).Within(0.001f));
            Assert.That(
                body.futur.position,
                OwnNUnit.Is.EqualTo(new Vector2(-1.153f, -1.153f)).Within(0.001f));

            world.Step(1, 0, 0, 1);
            Assert.That(world.tick, Is.EqualTo(201));
            Assert.That(world.pastTick, Is.EqualTo(200));
            Assert.That(world.futurTick, Is.EqualTo(201));

            Assert.That(body.current.tick, Is.EqualTo(201));
            Assert.That(
                body.current.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(1.680f, 1.680f)).Within(0.001f));
            Assert.That(
                body.current.position,
                OwnNUnit.Is.EqualTo(new Vector2(0.016f, 0.016f)).Within(0.001f));

            Assert.That(body.past.tick, Is.EqualTo(200));
            Assert.That(
                body.past.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(1.783f, 1.783f)).Within(0.001f));
            Assert.That(
                body.past.position,
                OwnNUnit.Is.EqualTo(new Vector2(0f, 0f)).Within(0.001f));

            Assert.That(body.futur.tick, Is.EqualTo(201));
            Assert.That(
                body.futur.linearVelocity,
                OwnNUnit.Is.EqualTo(new Vector2(1.680f, 1.680f)).Within(0.001f));
            Assert.That(
                body.futur.position,
                OwnNUnit.Is.EqualTo(new Vector2(0.016f, 0.016f)).Within(0.001f));
        }

        /*
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
        }*/
    }
}
