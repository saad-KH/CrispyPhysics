using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region Contact
    public struct ContactFeature
    {
        public enum Type
        {
            Vertex = 0,
            Face = 1
        }

        public readonly byte indexA;
        public readonly byte indexB;
        public readonly byte typeA;
        public readonly byte typeB;

        public ContactFeature(byte indexA, byte indexB, byte typeA, byte typeB)
        {
            this.indexA = indexA;
            this.indexB = indexB;
            this.typeA = typeA;
            this.typeB = typeB;
        }
    }

    public struct ContactID
    {
        public readonly ContactFeature feature;
        public readonly int key;

        public ContactID(int key)
        {
            this.key = key;
            feature = new ContactFeature();
        }

        public ContactID(int key, ContactFeature feature)
        {
            this.feature = feature;
            this.key = key;
        }
    }

    public struct ManifoldPoint
    {
        public readonly Vector2 localPoint;      ///< usage depends on manifold type
        public readonly float normalImpulse;  ///< the non-penetration impulse
        public readonly float tangentImpulse; ///< the friction impulse
        public readonly ContactID id;         ///< uniquely identifies a contact point between two shapes
        
        public ManifoldPoint(ContactID id, Vector2 localPoint)
        {
            this.id = id;
            this.localPoint = localPoint;
            normalImpulse = 0f;
            tangentImpulse = 0f;
        }

        public ManifoldPoint
            (ContactID id,
            Vector2 localPoint, float normalImpulse, float tangentImpulse)
        {
            this.id = id;
            this.localPoint = localPoint;
            this.normalImpulse = normalImpulse;
            this.tangentImpulse = tangentImpulse;
        }
    };

    public struct Manifold
    {
        public enum Type
        {
            Circles,
            FaceA,
            FaceB
        }

        public readonly ManifoldPoint[] points;
        public readonly Vector2 localNormal; ///< not use for Type::e_points
        public readonly Vector2 localPoint;  ///< usage depends on manifold type
        public readonly Type type;
        public int pointCount { get { return points.Length; } }

        public Manifold(
            Type type,
            Vector2 localPoint, Vector2 localNormal,
            ManifoldPoint[] points)
        {
            if(points == null)
                throw new System.ArgumentException("Specified Points should not be null");

            this.localNormal = localNormal;
            this.localPoint = localPoint;
            this.type = type;

            this.points = points;
        }
    }

    public struct WorldManifold
    {
        public readonly Vector2 normal;
        public readonly Vector2[] points;
        public readonly float[] separations;
        /// Evaluate the manifold with supplied transforms. This assumes
        /// modest motion from the original state. This does not change the
        /// point count, impulses, etc. The radii must come from the shapes
        /// that generated the manifold.
        public WorldManifold(
            Manifold manifold,
            Transformation transformA, float radiusA,
            Transformation transformB, float radiusB)
        {
            normal = Vector2.zero;

            points = new Vector2[Constants.maxManifoldPoints];
            separations = new float[Constants.maxManifoldPoints];

            if (manifold.pointCount == 0)
                return;

            switch (manifold.type)
            {
                case Manifold.Type.Circles:
                    {
                        normal = Vector2.zero;

                        Vector2 pointA = Calculus.Mul(
                            transformA,
                            manifold.localPoint);

                        Vector2 pointB = Calculus.Mul(
                            transformB,
                            manifold.points[0].localPoint);

                        if ((pointA - pointB).SqrMagnitude() > Mathf.Epsilon * Mathf.Epsilon)
                        {
                            normal = pointB - pointA;
                            normal.Normalize();
                        }

                        Vector2 cA = pointA + radiusA * normal;
                        Vector2 cB = pointB - radiusB * normal;
                        points[0] = 0.5f * (cA + cB);
                        separations[0] = Calculus.Dot(cB - cA, normal);
                    }
                    break;
                case Manifold.Type.FaceA:
                    {
                        normal = Calculus.Mul(
                            transformA.rotation,
                            manifold.localNormal);

                        Vector2 planePoint = Calculus.Mul(
                            transformA,
                            manifold.localPoint);

                        for (int i = 0; i < manifold.pointCount; i++)
                        {
                            Vector2 clipPoint = Calculus.Mul(
                                transformB,
                                manifold.points[i].localPoint);

                            Vector2 cA = clipPoint
                                + (radiusA - Calculus.Dot(clipPoint - planePoint, normal)) * normal;
                            Vector2 cB = clipPoint - radiusB * normal;
                            points[i] = 0.5f * (cA + cB);
                            separations[i] = Calculus.Dot(cB - cA, normal);
                        }
                    }
                    break;
                case Manifold.Type.FaceB:
                    {
                        normal = Calculus.Mul(
                            transformB.rotation,
                            manifold.localNormal);

                        Vector2 planePoint = Calculus.Mul(
                            transformB,
                            manifold.localPoint);

                        for (int i = 0; i < manifold.pointCount; ++i)
                        {
                            Vector2 clipPoint = Calculus.Mul(
                                transformA,
                                manifold.points[i].localPoint);
                            Vector2 cB = clipPoint
                                + (radiusB - Calculus.Dot(clipPoint - planePoint, normal)) * normal;
                            Vector2 cA = clipPoint - radiusA * normal;
                            points[i] = 0.5f * (cA + cB);
                            separations[i] = Calculus.Dot(cA - cB, normal);
                        }
                        // Ensure normal points from A to B.
                        normal = -normal;
                    }
                    break;
            }
        }
    };

    public struct PointState{
        public enum State
        {
            NullState,
            AddState,
            PersistState,
            RemoveState
        }

        public static void GetPointStates(
            ref State[] state1, ref State[] state2,
            Manifold manifold1, Manifold manifold2)
        {
            Debug.Assert(state1.Length == state2.Length);
            Debug.Assert(state1.Length == manifold1.pointCount);
            Debug.Assert(manifold1.pointCount == manifold2.pointCount);

            for (int i = 0; i < state1.Length; i++)
                state1[i] = state2[i] = State.NullState;

            for (int i = 0; i < state2.Length; i++)
                state2[i] = State.NullState;

            // Detect persists and removes.
            for (int i = 0; i < manifold1.pointCount; i++)
            {
                ContactID id = manifold1.points[i].id;

                state1[i] = State.RemoveState;

                for (int j = 0; j < manifold2.pointCount; j++)
                {
                    if (manifold2.points[j].id.key == id.key)
                    {
                        state1[i] = State.PersistState;
                        break;
                    }
                }
            }

            // Detect persists and adds.
            for (int i = 0; i < manifold2.pointCount; ++i)
            {
                ContactID id = manifold2.points[i].id;

                state2[i] = State.AddState;

                for (int j = 0; j < manifold1.pointCount; ++j)
                {
                    if (manifold1.points[j].id.key == id.key)
                    {
                        state2[i] = State.PersistState;
                        break;
                    }
                }
            }
        }
    }

    #endregion
    #region Raycast
    public struct RayCastInput
    {
        public readonly Vector2 origin;
        public readonly Vector2 target;
        public readonly float maxFraction;

        public RayCastInput(Vector2 origin, Vector2 target, float maxFraction)
        {
            this.origin = origin;
            this.target = target;
            this.maxFraction = maxFraction;
        }
    }

    public struct RayCastOuput
    {
        public readonly Vector2 normal;
        public readonly float fraction;

        public RayCastOuput(Vector2 normal, float fraction)
        {
            this.normal = normal;
            this.fraction = fraction;
        }
    }
    #endregion

    #region AABB
    public struct AABB
    {
        public readonly Vector2 lowerBound;
        public readonly Vector2 upperBound;

        public AABB(Vector2 pointA, Vector2 pointB)
        {
            lowerBound = Vector2.Min(pointA, pointB);
            upperBound = Vector2.Max(pointA, pointB);
        }

        public Vector2 GetCenter()
        {
            return 0.5f * (lowerBound + upperBound);
        }

        public Vector2 GetExtents()
        {
            return 0.5f * (upperBound - lowerBound);
        }

        public float GetPerimeter()
        {
            float wx = upperBound.x - lowerBound.x;
            float wy = upperBound.y - lowerBound.y;
            return 2.0f * (wx + wy);
        }


        public bool Contains(AABB aabb)
        {
            bool result = true;
            result = result && lowerBound.x <= aabb.lowerBound.x;
            result = result && lowerBound.y <= aabb.lowerBound.y;
            result = result && aabb.upperBound.x <= upperBound.x;
            result = result && aabb.upperBound.y <= upperBound.y;
            return result;
        }


        public bool RayCast(ref RayCastOuput output, RayCastInput input)
        {
            float tmin = float.MinValue;
            float tmax = float.MaxValue;

            float[] lower = new float[2] { lowerBound.x, lowerBound.y };
            float[] upper = new float[2] { upperBound.x, upperBound.y };
            float[] origin = new float[2] { input.origin.x, input.origin.y };

            float[] direction = new float[2] {
                (input.target - input.origin).x,
                (input.target - input.origin).y};

            float[] normal = new float[2] { 0f, 0f };

            for (int i = 0; i < 2; i++)
            {
                if (Mathf.Abs(direction[i]) < Mathf.Epsilon)
                {
                    if (origin[i] < lower[i] || upper[i] < origin[i])
                        return false;
                }
                else
                {
                    float inv_d = 1.0f / direction[i];
                    float t1 = (lower[i] - origin[i]) * inv_d;
                    float t2 = (upper[i] - origin[i]) * inv_d;

                    float s = -1.0f;

                    if (t1 > t2)
                    {
                        Calculus.Swap(ref t1, ref t2);
                        s = 1f;
                    }

                    if (t1 > tmin)
                    {
                        for (int j = 0; j < normal.Length; j++)
                            normal[j] = 0f;

                        normal[i] = s;
                        tmin = t1;
                    }
                    tmax = Mathf.Min(tmax, t2);

                    if (tmin > tmax)
                        return false;
                }
            }

            if (tmin < 0.0f || input.maxFraction < tmin)
                return false;

            output = new RayCastOuput(
                new Vector2(normal[0], normal[1]), 
                tmin);

            return true;
        }

        public static AABB Combine(AABB aabb1, AABB aabb2)
        {
            return new AABB(
                Vector2.Min(aabb1.lowerBound, aabb2.lowerBound),
                Vector2.Max(aabb1.upperBound, aabb2.upperBound));
        }
    }
    #endregion

    #region Collision
    public class Collision
    {
        public static bool CollideCircles(
            ref Manifold output,
            CircleShape circleA, Transformation transformA,
            CircleShape circleB, Transformation transformB)
        {
            Vector2 pA = Calculus.Mul(transformA, circleA.position);
            Vector2 pB = Calculus.Mul(transformB, circleB.position);

            Vector2 d = pB - pA;
            float distSqr = Calculus.Dot(d, d);
            float rA = circleA.radius, rB = circleB.radius;
            float radius = rA + rB;

            if (distSqr > radius * radius)
                return false;

            ManifoldPoint[] points = new ManifoldPoint[1];
            points[0] = new ManifoldPoint(
                new ContactID(0),
                circleB.position);

            output = new Manifold(
                Manifold.Type.Circles,
                circleA.position, Vector2.zero,
                points);

            return true;
        }

        public static bool CollidePolygionAndCircle(
            ref Manifold output,
            PolygonShape polygonA, Transformation transformA,
            CircleShape circleB, Transformation transformB)
        {

            // Compute circle position in the frame of the polygon.
            Vector2 c = Calculus.Mul(transformB, circleB.position);
            Vector2 cLocal = Calculus.MulT(transformA, c);

            // Find the min separating edge.
            int normalIndex = 0;
            float separation = float.MinValue;
            float radius = polygonA.radius + circleB.radius;
            int vertexCount = polygonA.count;
            Vector2[] vertices = polygonA.vertices;
            Vector2[] normals = polygonA.normals;

            for (int i = 0; i < vertexCount; i++)
            {
                float s = Calculus.Dot(normals[i], cLocal - vertices[i]);

                if (s > radius)
                    return false;

                if (s > separation)
                {
                    separation = s;
                    normalIndex = i;
                }
            }

            // Vertices that subtend the incident face.
            int vertIndex1 = normalIndex;
            int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
            Vector2 v1 = vertices[vertIndex1];
            Vector2 v2 = vertices[vertIndex2];

            ManifoldPoint[] points = new ManifoldPoint[1];
            points[0] = new ManifoldPoint(
                new ContactID(0),
                circleB.position);

            // If the center is inside the polygon ...
            if (separation < Mathf.Epsilon)
            {
                output = new Manifold(
                    Manifold.Type.FaceA,
                    0.5f * (v1 + v2), normals[normalIndex],
                    points);

                return true;
            }

            // Compute barycentric coordinates
            float u1 = Calculus.Dot(cLocal - v1, v2 - v1);
            float u2 = Calculus.Dot(cLocal - v2, v1 - v2);

            if (u1 <= 0.0f)
            {
                if ((cLocal - v1).SqrMagnitude() > radius * radius)
                    return false;

                output = new Manifold(
                    Manifold.Type.FaceA,
                   v1, (cLocal - v1).normalized,
                    points);

                return true;
            }
            else if (u2 <= 0.0f)
            {
                if ((cLocal - v2).SqrMagnitude() > radius * radius)
                    return false;

                output = new Manifold(
                    Manifold.Type.FaceA,
                    v2, (cLocal - v2).normalized,
                    points);

                return true;
            }
            else
            {
                Vector2 faceCenter = 0.5f * (v1 + v2);
                float s = Calculus.Dot(cLocal - faceCenter, normals[vertIndex1]);
                if (s > radius)
                    return false;

                output = new Manifold(
                    Manifold.Type.FaceA,
                    faceCenter, normals[vertIndex1],
                    points);

                return true;
            }
        }

        public static bool CollideEdgeAndCircle(
           ref Manifold output,
           EdgeShape edgeA, Transformation transformA,
           CircleShape circleB, Transformation transformB)
        {
            // Compute circle in frame of edge
            Vector2 Q = Calculus.MulT(
                transformA, 
                Calculus.Mul(transformB, circleB.position));

            Vector2 A = edgeA.vertex1, B = edgeA.vertex2;
            Vector2 e = B - A;

            // Barycentric coordinates
            float u = Calculus.Dot(e, B - Q);
            float v = Calculus.Dot(e, Q - A);

            float radius = edgeA.radius + circleB.radius;

            // Region A
            if (v <= 0.0f)
            {
                Vector2 P = A;
                Vector2 d = Q - P;
                float dd = Calculus.Dot(d, d);
                if (dd > radius * radius)
                    return false;

                // Is there an edge connected to A?
                if (edgeA.hasVertex0)
                {
                    Vector2 A1 = edgeA.vertex0;
                    Vector2 B1 = A;
                    Vector2 e1 = B1 - A1;
                    float u1 = Calculus.Dot(e1, B1 - Q);

                    // Is the circle in Region AB of the previous edge?
                    if (u1 > 0.0f)
                        return false;
                }
                
                ;

                ManifoldPoint[] points = new ManifoldPoint[1];
                points[0] = new ManifoldPoint(
                    new ContactID(
                        0,
                        new ContactFeature(
                            0, 0,
                            (byte)ContactFeature.Type.Vertex, 
                            (byte)ContactFeature.Type.Vertex)),
                    circleB.position);

                output = new Manifold(
                    Manifold.Type.Circles,
                    P, Vector2.zero,
                    points);

                return true;
            }

            // Region B
            if (u <= 0.0f)
            {
                Vector2 P = B;
                Vector2 d = Q - P;
                float dd = Calculus.Dot(d, d);
                if (dd > radius * radius)
                    return false;
                

                // Is there an edge connected to B?
                if (edgeA.hasVertex3)
                {
                    Vector2 B2 = edgeA.vertex3;
                    Vector2 A2 = B;
                    Vector2 e2 = B2 - A2;
                    float v2 = Calculus.Dot(e2, Q - A2);

                    // Is the circle in Region AB of the next edge?
                    if (v2 > 0.0f)
                        return false;
                }

                ManifoldPoint[] points = new ManifoldPoint[1];
                points[0] = new ManifoldPoint(
                    new ContactID(
                        0,
                        new ContactFeature(
                            1, 0,
                            (byte)ContactFeature.Type.Vertex,
                            (byte)ContactFeature.Type.Vertex)),
                    circleB.position);

                output = new Manifold(
                    Manifold.Type.Circles,
                    P, Vector2.zero,
                    points);

                return true;
            }

            // Region AB
            {

                float den = Calculus.Dot(e, e);
                Debug.Assert(den > 0.0f);
                Vector2 P = (1.0f / den) * (u * A + v * B);
                Vector2 d = Q - P;
                float dd = Calculus.Dot(d, d);
                if (dd > radius * radius)
                    return false;

                Vector2 n = new Vector2(-e.y, e.x);
                if (Calculus.Dot(n, Q - A) < 0.0f)
                    n.Set(-n.x, -n.y);
                n.Normalize();

                ManifoldPoint[] points = new ManifoldPoint[1];
                points[0] = new ManifoldPoint(
                    new ContactID(
                        0,
                        new ContactFeature(
                            0, 0,
                            (byte)ContactFeature.Type.Face,
                            (byte)ContactFeature.Type.Vertex)),
                    circleB.position);

                output = new Manifold(
                    Manifold.Type.FaceA,
                    A, n,
                    points);

                return true;
            }
        }


        public static bool TestOverlap(AABB a, AABB b)
        {
            Vector2 d1, d2;
            d1 = b.lowerBound - a.upperBound;
            d2 = a.lowerBound - b.upperBound;

            if (d1.x > 0f || d1.y > 0f)
                return false;

            if (d2.x > 0f || d2.y > 0f)
                return false;

            return true;
        }
    }
    #endregion
}