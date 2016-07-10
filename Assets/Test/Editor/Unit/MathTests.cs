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
                "Rotation's Sine is constructed correctly");
            Assert.That(
                rot.cosine, 
                Is.EqualTo(0.5f).Within(0.001f),
                "Rotation's Cosine is constructed correctly");

           rot.Set(Mathf.PI / 4);

            Assert.That(
                rot.sine,
                Is.EqualTo(0.707f).Within(0.001f),
                "Rotation's Sine is set correctly");
            Assert.That(
                rot.cosine,
                Is.EqualTo(0.707f).Within(0.001f),
                "Rotation's Cosine is set correctly");

            rot.SetIdentity();

            Assert.That(
                rot.sine,
                Is.EqualTo(0f).Within(0.001f),
                "Identity Rotation's Sine is set correctly");
            Assert.That(
                rot.cosine,
                Is.EqualTo(0f).Within(0.001f),
                "Identity Rotation's Cosine is set correctly");

            rot.Set(Mathf.PI / 3);

            Assert.That(
                rot.GetAngle(),
                Is.EqualTo(1.047f).Within(0.001f),
                "Rotation's Angle is computed correctly");

            Assert.That(
                rot.GetXAxis().x,
                Is.EqualTo(0.5f).Within(0.001f),
                "Rotation's X Axis's X is computed correctly");

            Assert.That(
                rot.GetXAxis().y,
                Is.EqualTo(0.866f).Within(0.001f),
                "Rotation's X Axis's Y is computed correctly");

            Assert.That(
                rot.GetYAxis().x,
                Is.EqualTo(-0.866f).Within(0.001f),
                "Rotation's Y Axis's X is computed correctly");

            Assert.That(
                rot.GetYAxis().y,
                Is.EqualTo(0.5f).Within(0.001f),
                "Rotation's Y Axis's Y is computed correctly");
        }

        [Test]
        public void UsingSweep()
        {
            Sweep sweep = new Sweep();
            sweep.Reset(Vector2.zero, 0f);

            Assert.That(sweep.center0, Is.EqualTo(Vector2.zero), "Center 0 Is Reset");
            Assert.That(sweep.angle0, Is.EqualTo(0f), "Angle 0 Is Reset");
            Assert.That(sweep.alpha0, Is.EqualTo(0f), "Alpha Is Reset");

            sweep.center = new Vector2(1f, 1f);
            sweep.angle = Mathf.PI / 4;
            Transformation trans = sweep.GetTransform(0.1f);

            Assert.That(
                trans.position.x, 
                Is.EqualTo(0.1f).Within(0.001f), 
                "Sweep Position's X's interpolation is well computed");
            Assert.That(
                 trans.position.y,
                 Is.EqualTo(0.1f).Within(0.001f),
                 "Sweep Position's Y's interpolation is well computed");
            Assert.That(
                trans.rotation.sine,
                Is.EqualTo(0.078f).Within(0.001f),
                "Sweep Rotation's Sine's interpolation is well computed");
            Assert.That(
                 trans.rotation.cosine,
                 Is.EqualTo(0.996f).Within(0.001f),
                 "Sweep Rotation's Cosine's interpolation is well computed");

            sweep.Advance(0.5f);

            Assert.That(
                sweep.center0.x,
                Is.EqualTo(0.5f).Within(0.001f),
                "Sweep Position's X advances");
            Assert.That(
                 sweep.center0.y,
                 Is.EqualTo(0.5f).Within(0.001f),
                 "Sweep Position's Y advances");
            Assert.That(
                sweep.angle0,
                Is.EqualTo(0.392f).Within(0.001f),
                "Sweep Angle advances");
            Assert.That(
                 sweep.alpha0,
                 Is.EqualTo(0.5f).Within(0.001f),
                 "Sweep Alpha0 advances");

            sweep.angle0 += Mathf.PI * 2f;
            sweep.angle += Mathf.PI * 4f;

            sweep.Normalize();
            Assert.That(
                sweep.angle0,
                Is.EqualTo(0.392f).Within(0.001f),
                "Angle 0 is normalized Correctly");
            Assert.That(
                 sweep.angle,
                 Is.EqualTo(0.785f).Within(0.001f),
                 "Angle is Normalized Correctly");


        } 

        [Test]
        public void ComputingWithCalculus()
        {
            Vector2 rotMulVec = Calculus.Mul(
                new Rotation(Mathf.PI / 3f),
                new Vector2(0.707f, 0.707f));

            rotMulVec.Normalize();
            Assert.That(
                rotMulVec.x,
                Is.EqualTo(-0.258f).Within(0.001f),
                "Vector'X is Well Rotated");
            Assert.That(
                rotMulVec.y,
                Is.EqualTo(0.965f).Within(0.001f),
                "Vector'Y is Well Rotated");

            Vector2 rotMulTVec = Calculus.MulT(
                new Rotation(Mathf.PI / 3f),
                rotMulVec);

            rotMulTVec.Normalize();
            Assert.That(
                rotMulTVec.x,
                Is.EqualTo(0.707f).Within(0.001f),
                "Vector'X is Well Rotated Inversly");
            Assert.That(
                rotMulTVec.y,
                Is.EqualTo(0.707f).Within(0.001f),
                "Vector'Y is Well Rotated Inversly");

            Vector2 transMulVec = Calculus.Mul(
                new Transformation(
                    new Vector2(0.5f, 1.0f),
                    new Rotation(Mathf.PI / 3f)),
                new Vector2(0.707f, 0.707f));

            transMulVec.Normalize();
            Assert.That(
                transMulVec.x,
                Is.EqualTo(0.121f).Within(0.001f),
                "Vector'X is Well Transformed");
            Assert.That(
                transMulVec.y,
                Is.EqualTo(0.992f).Within(0.001f),
                "Vector'Y is Well Transformed");

            Vector2 transMulTVec = Calculus.MulT(
                new Transformation(
                    new Vector2(0.5f, 1.0f),
                    new Rotation(Mathf.PI / 3f)),
                transMulVec);

            transMulTVec.Normalize();
            Assert.That(
                transMulTVec.x,
                Is.EqualTo(-0.516f).Within(0.001f),
                "Vector'X is Well Transformed Inversly");
            Assert.That(
                transMulTVec.y,
                Is.EqualTo(0.856f).Within(0.001f),
                "Vector'Y is Well Transformed Inversly");

            Vector2 scaCrossVec = Calculus.Cross(
                0.5f,
                new Vector2(0.707f, 0.707f));

            Assert.That(
                scaCrossVec.x,
                Is.EqualTo(-0.353f).Within(0.001f),
                "Vector'X is Well Crossed With Scalar");
            Assert.That(
                scaCrossVec.y,
                Is.EqualTo(0.353f).Within(0.001f),
                "Vector'Y is Well Crossed With Scalar");

            float vecCrossVec = Calculus.Cross(
                new Vector2(1f, 0f),
                new Vector2(0.707f, 0.707f));

            Assert.That(
                vecCrossVec,
                Is.EqualTo(0.207f).Within(0.001f),
                "Vector is Well Crossed With Vector");


        }
    }
}
