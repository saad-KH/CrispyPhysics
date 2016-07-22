using NUnit.Framework;
using UnityEngine;

namespace CrispyPhysics
{
    [TestFixture]
    public class MathTests
    {
        [Test]
        public void UsingRotation()
        {
            Rotation rot = new Rotation(Mathf.PI / 3);

            Assert.That(
                rot.sine, 
                Is.EqualTo(0.866f).Within(0.001f),
                "Rotation's sine is constructed correctly");
            Assert.That(
                rot.cosine, 
                Is.EqualTo(0.5f).Within(0.001f),
                "Rotation's cosine is constructed correctly");

           rot.Set(Mathf.PI / 4);

            Assert.That(
                rot.sine,
                Is.EqualTo(0.707f).Within(0.001f),
                "Rotation's sine is set correctly");
            Assert.That(
                rot.cosine,
                Is.EqualTo(0.707f).Within(0.001f),
                "Rotation's cosine is set correctly");

            rot.SetIdentity();

            Assert.That(
                rot.sine,
                Is.EqualTo(0f).Within(0.001f),
                "Identity rotation's Sine is set correctly");
            Assert.That(
                rot.cosine,
                Is.EqualTo(0f).Within(0.001f),
                "Identity rotation's Cosine is set correctly");

            rot.Set(Mathf.PI / 3);

            Assert.That(
                rot.GetAngle(),
                Is.EqualTo(1.047f).Within(0.001f),
                "Rotation's angle is computed correctly");

            Assert.That(
                rot.GetXAxis(),
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 0.866f)).Within(0.001f),
                "Rotation's X axis is computed correctly");

            Assert.That(
                rot.GetYAxis(),
                OwnNUnit.Is.EqualTo(new Vector2(-0.866f, 0.5f)).Within(0.001f),
                "Rotation's Y axis is computed correctly");
        }

        [Test]
        public void UsingSweep()
        {
            Sweep sweep = new Sweep();
            sweep.Reset(Vector2.zero, 0f);

            Assert.That(sweep.center0, Is.EqualTo(Vector2.zero), "Center 0 Is reset");
            Assert.That(sweep.angle0, Is.EqualTo(0f), "Angle 0 Is reset");
            Assert.That(sweep.alpha0, Is.EqualTo(0f), "Alpha Is reset");

            sweep.center = new Vector2(1f, 1f);
            sweep.angle = Mathf.PI / 4;
            Transformation trans = sweep.GetTransform(0.1f);

            Assert.That(
                trans.position,
                OwnNUnit.Is.EqualTo(new Vector2(0.1f, 0.1f)).Within(0.001f),
                "Sweep position's interpolation is well computed");
            Assert.That(
                trans.rotation.sine,
                Is.EqualTo(0.078f).Within(0.001f),
                "Sweep rotation's Sine's interpolation is well computed");
            Assert.That(
                 trans.rotation.cosine,
                 Is.EqualTo(0.996f).Within(0.001f),
                 "Sweep rotation's Cosine's interpolation is well computed");

            sweep.Advance(0.5f);

            Assert.That(
                sweep.center0,
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 0.5f)).Within(0.001f),
                "Sweep position's advances");
            Assert.That(
                sweep.angle0,
                Is.EqualTo(0.392f).Within(0.001f),
                "Sweep angle advances");
            Assert.That(
                 sweep.alpha0,
                 Is.EqualTo(0.5f).Within(0.001f),
                 "Sweep alpha0 advances");

            sweep.angle0 += Mathf.PI * 2f;
            sweep.angle += Mathf.PI * 4f;
            sweep.Normalize();

            Assert.That(
                sweep.angle0,
                Is.EqualTo(0.392f).Within(0.001f),
                "Angle 0 is normalized correctly");
            Assert.That(
                 sweep.angle,
                 Is.EqualTo(0.785f).Within(0.001f),
                 "Angle is Normalized correctly");


        } 

        [Test]
        public void ComputingWithCalculus()
        {
            Vector2 rotMulVec = Calculus.Mul(
                new Rotation(Mathf.PI / 3f),
                new Vector2(0.707f, 0.707f));
            rotMulVec.Normalize();

            Assert.That(
                rotMulVec,
                OwnNUnit.Is.EqualTo(new Vector2(-0.258f, 0.965f)).Within(0.001f),
                "Vector is well rotated");

            Vector2 rotMulTVec = Calculus.MulT(
                new Rotation(Mathf.PI / 3f),
                rotMulVec);
            rotMulTVec.Normalize();

            Assert.That(
                rotMulTVec,
                OwnNUnit.Is.EqualTo(new Vector2(0.707f, 0.707f)).Within(0.001f),
                "Vector is well rotated inversly");

            Vector2 transMulVec = Calculus.Mul(
                new Transformation(
                    new Vector2(0.5f, 1.0f),
                    new Rotation(Mathf.PI / 3f)),
                new Vector2(0.707f, 0.707f));
            transMulVec.Normalize();

            Assert.That(
                transMulVec,
                OwnNUnit.Is.EqualTo(new Vector2(0.121f, 0.992f)).Within(0.001f),
                "Vector is well transformed");

            Vector2 transMulTVec = Calculus.MulT(
                new Transformation(
                    new Vector2(0.5f, 1.0f),
                    new Rotation(Mathf.PI / 3f)),
                transMulVec);
            transMulTVec.Normalize();

            Assert.That(
                transMulTVec,
                OwnNUnit.Is.EqualTo(new Vector2(-0.516f, 0.856f)).Within(0.001f),
                "Vector is well transformed inversly");

            float vecCrossVec = Calculus.Cross(
                new Vector2(1f, 0f),
                new Vector2(0.707f, 0.707f));

            Assert.That(
                vecCrossVec,
                Is.EqualTo(0.707f).Within(0.001f),
                "Vector is well crossed with vector");


        }
    }
}
