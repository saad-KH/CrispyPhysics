using NUnit.Framework;
using UnityEngine;


namespace CrispyPhysics
{
    using Internal;

    [TestFixture]
    public class CollisionTests
    {

        [Test]
        public void usingManifoldStructure()
        {
            ContactFeature cf = new ContactFeature(1, 2, 
                (byte)ContactFeature.Type.Vertex,
                (byte)ContactFeature.Type.Face);

            Assert.That(cf.indexA, Is.EqualTo(1));
            Assert.That(cf.indexB, Is.EqualTo(2));
            Assert.That(cf.typeA, Is.EqualTo((byte)ContactFeature.Type.Vertex));
            Assert.That(cf.typeB, Is.EqualTo((byte)ContactFeature.Type.Face));

            ContactFeature sameCF = new ContactFeature(1, 2,
                (byte)ContactFeature.Type.Vertex,
                (byte)ContactFeature.Type.Face);
            ContactFeature unsameCF = new ContactFeature(2, 1, 
                (byte)ContactFeature.Type.Face,
                (byte)ContactFeature.Type.Vertex);

            Assert.That(cf.Same(sameCF));
            Assert.That(cf.Same(unsameCF) == false);

            ContactID cId = new ContactID(0, cf);

            Assert.That(cId.key, Is.EqualTo(0));
            Assert.That(cId.feature, Is.EqualTo(cf));

            ContactID sameCId = new ContactID(0, sameCF);
            ContactID unsameCId = new ContactID(1, unsameCF);

            Assert.That(cId.Same(sameCId));
            Assert.That(cId.Same(unsameCId) == false);

            ManifoldPoint mfp = new ManifoldPoint(cId, Vector2.zero);

            Assert.That(mfp.point, Is.EqualTo(Vector2.zero));
            Assert.That(mfp.normalImpulse, Is.EqualTo(0f));
            Assert.That(mfp.tangentImpulse, Is.EqualTo(0f));
            Assert.That(mfp.id, Is.EqualTo(cId));

            mfp = new ManifoldPoint(cId, Vector2.zero, 1f, 1f);

            Assert.That(mfp.point, Is.EqualTo(Vector2.zero));
            Assert.That(mfp.normalImpulse, Is.EqualTo(1f));
            Assert.That(mfp.tangentImpulse, Is.EqualTo(1f));
            Assert.That(mfp.id, Is.EqualTo(cId));

            ManifoldPoint sameMfp = new ManifoldPoint(sameCId, Vector2.zero, 1f, 1f);
            ManifoldPoint unsameMfp = new ManifoldPoint(unsameCId, Vector2.up, 0f, 0f);

            Assert.That(mfp.Same(sameMfp));
            Assert.That(mfp.Same(unsameMfp) == false);

            Manifold manifold = new Manifold(Manifold.Type.Circles, Vector2.zero, Vector2.right);

            Assert.That(manifold.type, Is.EqualTo(Manifold.Type.Circles));
            Assert.That(manifold.point, Is.EqualTo(Vector2.zero));
            Assert.That(manifold.normal, Is.EqualTo(Vector2.right));
            Assert.That(manifold.pointCount, Is.EqualTo(0));

            manifold.Set(Manifold.Type.FaceA, Vector2.one, Vector2.up);

            Assert.That(manifold.type, Is.EqualTo(Manifold.Type.FaceA));
            Assert.That(manifold.point, Is.EqualTo(Vector2.one));
            Assert.That(manifold.normal, Is.EqualTo(Vector2.up));
            Assert.That(manifold.pointCount, Is.EqualTo(0));

            Manifold sameManifold = new Manifold(Manifold.Type.FaceA, Vector2.one, Vector2.up);
            Manifold unsameManifold = new Manifold(Manifold.Type.FaceB, Vector2.down, Vector2.left);

            Assert.That(manifold.Same(sameManifold));
            Assert.That(manifold.Same(unsameManifold) == false);

            manifold.AddPoint(mfp);
            Assert.That(manifold.pointCount, Is.EqualTo(1));
            Assert.That(manifold.points[0], Is.EqualTo(mfp));

            Assert.That(manifold.Same(sameManifold) == false);

            sameManifold.AddPoint(mfp);
            Assert.That(manifold.Same(sameManifold));
        }

        [Test]
        public void usingWorldManifold()
        {
            WorldManifold worldManifold = new WorldManifold();

            Assert.That(worldManifold.normal, Is.EqualTo(Vector2.zero));
            Assert.That(worldManifold.pointCount, Is.EqualTo(0));
            
            //Circles
            Manifold mf = new Manifold(Manifold.Type.Circles, Vector2.zero, Vector2.zero);
            mf.AddPoint(new ManifoldPoint(new ContactID(0), Vector2.zero));

            worldManifold.Set(
                mf,
                new Transformation(new Vector2(1f, 1f), 0f), 1f,
                new Transformation(new Vector2(-1f, 1f), 0f), 1f);

            Assert.That(worldManifold.normal, OwnNUnit.Is.EqualTo(Vector2.left).Within(0.001f));
            Assert.That(worldManifold.pointCount, Is.EqualTo(1));
            Assert.That(worldManifold.points[0], OwnNUnit.Is.EqualTo(new Vector2(0f, 1f)).Within(0.001f));
            Assert.That(worldManifold.separations[0], Is.EqualTo(0f).Within(0.001f));

            //FaceA With Circle
            mf = new Manifold(Manifold.Type.FaceA, Vector2.right, Vector2.right);
            mf.AddPoint(new ManifoldPoint(new ContactID(0), Vector2.zero));

            worldManifold.Set(
                 mf,
                 new Transformation(new Vector2(-1f, 1f), 0f), Constants.polygonRadius,
                 new Transformation(new Vector2(1f, 1f), 0f), 1f);

            Assert.That(worldManifold.normal, OwnNUnit.Is.EqualTo(Vector2.right).Within(0.001f));
            Assert.That(worldManifold.pointCount, Is.EqualTo(1));
            Assert.That(
                worldManifold.points[0], 
                OwnNUnit.Is.EqualTo(new Vector2(Constants.polygonRadius/2, 1f)).Within(0.001f));
            Assert.That(worldManifold.separations[0], Is.EqualTo(-Constants.polygonRadius).Within(0.001f));

            //FaceA With Polygon
            mf = new Manifold(Manifold.Type.FaceA, Vector2.right, Vector2.right);
            mf.AddPoint(new ManifoldPoint(new ContactID(0),Vector2.left));

            worldManifold.Set(
                 mf,
                 new Transformation(new Vector2(-1f, 1f), 0f), Constants.polygonRadius,
                 new Transformation(new Vector2(1f, 1f), 0f), Constants.polygonRadius);

            Assert.That(worldManifold.normal, OwnNUnit.Is.EqualTo(Vector2.right).Within(0.001f));
            Assert.That(worldManifold.pointCount, Is.EqualTo(1));
            Assert.That(
                worldManifold.points[0],
                OwnNUnit.Is.EqualTo(new Vector2(0f, 1f)).Within(0.001f));
            Assert.That(worldManifold.separations[0], Is.EqualTo(-2*Constants.polygonRadius).Within(0.001f));

            //FaceB With Polygon
            mf = new Manifold(Manifold.Type.FaceB, Vector2.left, Vector2.left);
            mf.AddPoint(new ManifoldPoint(new ContactID(0), Vector2.right));

            worldManifold.Set(
                 mf,
                 new Transformation(new Vector2(-1f, 1f), 0f), Constants.polygonRadius,
                 new Transformation(new Vector2(1f, 1f), 0f), Constants.polygonRadius);

            Assert.That(worldManifold.normal, OwnNUnit.Is.EqualTo(Vector2.right).Within(0.001f));
            Assert.That(worldManifold.pointCount, Is.EqualTo(1));
            Assert.That(
                worldManifold.points[0],
                OwnNUnit.Is.EqualTo(new Vector2(0f, 1f)).Within(0.001f));
            Assert.That(worldManifold.separations[0], Is.EqualTo(-2 * Constants.polygonRadius).Within(0.001f));

        }

        [Test]
        public void usingRaycastInput()
        {
            RayCastInput input = new RayCastInput(Vector2.zero, Vector2.one, 0.5f);

            Assert.That(input.origin, OwnNUnit.Is.EqualTo(Vector2.zero).Within(0.001f));
            Assert.That(input.target, OwnNUnit.Is.EqualTo(Vector2.one).Within(0.001f));
            Assert.That(input.maxFraction, Is.EqualTo(0.5f).Within(0.001f));
        }

        [Test]
        public void usingRaycastOutput()
        {
            RayCastOuput output = new RayCastOuput(Vector2.one, 0.5f);

            Assert.That(output.normal, OwnNUnit.Is.EqualTo(Vector2.one).Within(0.001f));
            Assert.That(output.fraction, Is.EqualTo(0.5f).Within(0.001f));
        }

        [Test]
        public void usingAABB()
        {
            AABB aabb = new AABB(
                new Vector2(0.5f, 1.5f),
                new Vector2(1f, 1f));

            Assert.That(
                aabb.lowerBound, 
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 1f)).Within(0.001f));
            Assert.That(
                aabb.upperBound, 
                OwnNUnit.Is.EqualTo(new Vector2(1f, 1.5f)).Within(0.001f));

            Assert.That(
                aabb.GetCenter(), 
                OwnNUnit.Is.EqualTo(new Vector2(0.75f, 1.25f)).Within(0.001f));

            Assert.That(
                aabb.GetExtents(),
                OwnNUnit.Is.EqualTo(new Vector2(0.25f, 0.25f)).Within(0.001f));

            Assert.That(
                aabb.GetPerimeter(),
                Is.EqualTo(2f).Within(0.001f));


            aabb = AABB.Combine(
                aabb, 
                new AABB( new Vector2(1f, 1f), new Vector2(1.5f, 0.5f)));

            Assert.That(
                aabb.lowerBound,
                OwnNUnit.Is.EqualTo(new Vector2(0.5f, 0.5f)).Within(0.001f));
            Assert.That(
                aabb.upperBound,
                OwnNUnit.Is.EqualTo(new Vector2(1.5f, 1.5f)).Within(0.001f));


            aabb = AABB.Combine(
                new AABB(new Vector2(0f, 0f), new Vector2(-1f, -1f)),
                new AABB(new Vector2(1f, 1f), new Vector2(0f, 0f)));

            Assert.That(
                aabb.lowerBound,
                OwnNUnit.Is.EqualTo(new Vector2(-1f, -1f)).Within(0.001f));
            Assert.That(
                aabb.upperBound,
                OwnNUnit.Is.EqualTo(new Vector2(1f, 1f)).Within(0.001f));

            Assert.That(
                aabb.Contains(new AABB(new Vector2(1f, 1f), new Vector2(0f, 0f))));


            RayCastInput input = 
                new RayCastInput(new Vector2(-2f, -2f), new Vector2(2f, 2f), 1f);

            RayCastOuput output = new RayCastOuput();
            bool hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal, 
                OwnNUnit.Is.EqualTo(new Vector2(-1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.25f).Within(0.001f));


            input = new RayCastInput(new Vector2(2f, 2f), new Vector2(-2f, -2f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.25f).Within(0.001f));


            input = new RayCastInput(new Vector2(1f, 1f), new Vector2(-1f, -1f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0f).Within(0.001f));


            input = new RayCastInput(new Vector2(0f, -2f), new Vector2(0f, 2f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(0f, -1f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.25f).Within(0.001f));


            input = new RayCastInput(new Vector2(0.5f, 1.5f), new Vector2(1.5f, 0.5f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(0f, 1f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.5f).Within(0.001f));


            input = new RayCastInput(new Vector2(-2f, -2f), new Vector2(0.5f, 0.5f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit);
            Assert.That(
                output.normal,
                OwnNUnit.Is.EqualTo(new Vector2(-1f, 0f)).Within(0.001f));
            Assert.That(
               output.fraction,
               Is.EqualTo(0.4f).Within(0.001f));


            input = new RayCastInput(new Vector2(0.5f, 1.5f), new Vector2(1.51f, 0.5f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit == false);


            input = new RayCastInput(new Vector2(0.5f, 0.5f), new Vector2(1f, 1f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit == false);


            input = new RayCastInput(new Vector2(0.5f, 0.5f), new Vector2(1.5f, 1.5f), 1f);
            hit = aabb.RayCast(ref output, input);

            Assert.That(hit == false);


        }

        [Test]
        public void handlingCollisions()
        {
            //Circle With Circle
            CircleShape circle = new CircleShape(Vector2.zero, 1f);
            CircleShape otherCircle = new CircleShape(Vector2.zero, 1f);

            Manifold mf = Collision.CollideCircles(
                circle, new Transformation(new Vector2(1f, 1f), 0f),
                otherCircle, new Transformation(new Vector2(-1f, 1f), 0f));

            Assert.That(mf != null);
            Assert.That(mf.type, Is.EqualTo(Manifold.Type.Circles));
            Assert.That(mf.point, Is.EqualTo(Vector2.zero));
            Assert.That(mf.normal, Is.EqualTo(Vector2.zero));
            Assert.That(mf.pointCount, Is.EqualTo(1));
            Assert.That(mf.points[0].point, Is.EqualTo(Vector2.zero));

            mf = Collision.CollideCircles(
                circle, new Transformation(new Vector2(2f, 1f), 0f),
                otherCircle, new Transformation(new Vector2(-2f, 1f), 0f));

            Assert.That(mf == null);

            //Circle With Polygon
            PolygonShape polygon = new PolygonShape();
            polygon.SetAsBox(1f, 1f);

            mf = Collision.CollidePolygionAndCircle(
                polygon, new Transformation(new Vector2(-1f, 1f), 0f),
                circle, new Transformation(new Vector2(1f, 1f), 0f));

            Assert.That(mf != null);
            Assert.That(mf.type, Is.EqualTo(Manifold.Type.FaceA));
            Assert.That(mf.point, Is.EqualTo(Vector2.right));
            Assert.That(mf.normal, Is.EqualTo(Vector2.right));
            Assert.That(mf.pointCount, Is.EqualTo(1));
            Assert.That(mf.points[0].point, Is.EqualTo(Vector2.zero));

            mf = Collision.CollidePolygionAndCircle(
                polygon, new Transformation(new Vector2(-2f, 1f), 0f),
                circle, new Transformation(new Vector2(2f, 1f), 0f));

            Assert.That(mf == null);

            //Circle With Edge
            EdgeShape edge = new EdgeShape(new Vector2(0f, -1f), new Vector2(0f, 1f));

            mf = Collision.CollideEdgeAndCircle(
                edge, new Transformation(new Vector2(0f, 1f), 0f),
                circle, new Transformation(new Vector2(1f, 1f), 0f));

            Assert.That(mf != null);
            Assert.That(mf.type, Is.EqualTo(Manifold.Type.FaceA));
            Assert.That(mf.point, Is.EqualTo(new Vector2(0f, -1f)));
            Assert.That(mf.normal, Is.EqualTo(Vector2.right));
            Assert.That(mf.pointCount, Is.EqualTo(1));
            Assert.That(mf.points[0].point, Is.EqualTo(Vector2.zero));
            Assert.That(mf.points[0].id.feature.indexA, Is.EqualTo(0));
            Assert.That(mf.points[0].id.feature.typeA, Is.EqualTo((byte)ContactFeature.Type.Face));

            mf = Collision.CollideEdgeAndCircle(
               edge, new Transformation(new Vector2(0f, 2f), 0f),
               circle, new Transformation(new Vector2(1f, 1f), 0f));

            Assert.That(mf != null);
            Assert.That(mf.type, Is.EqualTo(Manifold.Type.Circles));
            Assert.That(mf.point, Is.EqualTo(new Vector2(0f, -1f)));
            Assert.That(mf.normal, Is.EqualTo(Vector2.zero));
            Assert.That(mf.pointCount, Is.EqualTo(1));
            Assert.That(mf.points[0].point, Is.EqualTo(Vector2.zero));
            Assert.That(mf.points[0].id.feature.indexA, Is.EqualTo(0));
            Assert.That(mf.points[0].id.feature.typeA, Is.EqualTo((byte)ContactFeature.Type.Vertex));

            mf = Collision.CollideEdgeAndCircle(
               edge, new Transformation(new Vector2(0f, 0f), 0f),
               circle, new Transformation(new Vector2(1f, 1f), 0f));

            Assert.That(mf != null);
            Assert.That(mf.type, Is.EqualTo(Manifold.Type.Circles));
            Assert.That(mf.point, Is.EqualTo(new Vector2(0f, 1f)));
            Assert.That(mf.normal, Is.EqualTo(Vector2.zero));
            Assert.That(mf.pointCount, Is.EqualTo(1));
            Assert.That(mf.points[0].point, Is.EqualTo(Vector2.zero));
            Assert.That(mf.points[0].id.feature.indexA, Is.EqualTo(1));
            Assert.That(mf.points[0].id.feature.typeA, Is.EqualTo((byte)ContactFeature.Type.Vertex));

            mf = Collision.CollideEdgeAndCircle(
                edge, new Transformation(new Vector2(-1f, 1f), 0f),
                circle, new Transformation(new Vector2(2f, 1f), 0f));

            Assert.That(mf == null);
        }
    }
}
