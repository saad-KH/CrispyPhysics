using System;
using UnityEngine;

namespace CrispyPhysics.Internal
{
    public class PolygonShape : IShape
    {
        public ShapeType type { get { return ShapeType.Polygon; } }
        public float radius { get { return Constants.polygonRadius; } }
        public Vector2 centroid { get; private set; }
        public int count { get; private set; }

        public Vector2[] vertices { get; private set; }
        public Vector2[] normals { get; private set; }

        public PolygonShape()
        {
            count = 0;
            vertices = new Vector2[Constants.maxPolygonVertices];
            normals = new Vector2[Constants.maxPolygonVertices];
        }

        public void SetAsBox(float hx, float hy)
        {
            count = 4;
            vertices[0].Set(-hx, -hy);
            vertices[1].Set(hx, -hy);
            vertices[2].Set(hx, hy);
            vertices[3].Set(-hx, hy);
            normals[0].Set(0.0f, -1.0f);
            normals[1].Set(1.0f, 0.0f);
            normals[2].Set(0.0f, 1.0f);
            normals[3].Set(-1.0f, 0.0f);
            centroid = Vector2.zero;
        }

        public void SetAsBox(float hx, float hy, Vector2 center, float angle)
        {
            count = 4;
            vertices[0].Set(-hx, -hy);
            vertices[1].Set(hx, -hy);
            vertices[2].Set(hx, hy);
            vertices[3].Set(-hx, hy);
            normals[0].Set(0.0f, -1.0f);
            normals[1].Set(1.0f, 0.0f);
            normals[2].Set(0.0f, 1.0f);
            normals[3].Set(-1.0f, 0.0f);
            centroid = center;

            Transformation transform = new Transformation(
                center,
                new Rotation(angle));

            for (int i = 0; i < count; ++i)
            {
                vertices[i] = Calculus.Mul(transform, vertices[i]);
                normals[i] = Calculus.Mul(transform.rotation, normals[i]);
            }
        }

        static Vector2 ComputeCentroid(Vector2[] points, int count)
        {
            Debug.Assert(count >= 3);

            Vector2 c = new Vector2(0f, 0f);
	        float area = 0.0f;

            Vector2 pRef = new Vector2(0f, 0f);
            float inv3 = 1.0f / 3.0f;

	        for (int i = 0; i<count; i++)
	        {
                // Triangle vertices.
                Vector2 p1 = pRef;
                Vector2 p2 = points[i];
                Vector2 p3 = i + 1 < count ? points[i + 1] : points[0];

                Vector2 e1 = p2 - p1;
                Vector2 e2 = p3 - p1;

                float D = Calculus.Cross(e1, e2);

                float triangleArea = 0.5f * D;
                area += triangleArea;

		        c += triangleArea* inv3 * (p1 + p2 + p3);
	        }

            Debug.Assert(area > Mathf.Epsilon);
            c *= 1.0f / area;
            return c;
        }

        public void Set(Vector2[] points, int newCount)
        {
            Debug.Assert(3 <= count && count <= Constants.maxPolygonVertices);

            if (newCount < 3)
            {
                SetAsBox(1.0f, 1.0f);
                return;
            }

            int n = Mathf.Min(newCount, Constants.maxPolygonVertices);

            Vector2[] ps = new Vector2[Constants.maxPolygonVertices];
            int tempCount = 0;
            for (int i = 0; i < n; ++i)
            {
                Vector2 v = points[i];

                bool unique = true;
                for (int j = 0; j < tempCount; ++j)
                {
                    if (    ((v - ps[j]).sqrMagnitude) 
                        <   ((0.5f * Constants.linearSlop) * (0.5f * Constants.linearSlop)))
                    {
                        unique = false;
                        break;
                    }
                }

                if (unique)
                    ps[tempCount++] = v;
            }

            n = tempCount;
            if (n < 3)
            {
                // Polygon is degenerate.
                Debug.Assert(false);
                SetAsBox(1.0f, 1.0f);
                return;
            }

            // Create the convex hull using the Gift wrapping algorithm
            // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

            // Find the right most point on the hull
            int i0 = 0;
            float x0 = ps[0].x;
            for (int i = 1; i < n; i++)
            {
                float x = ps[i].x;
                if (x > x0 || (x == x0 && ps[i].y < ps[i0].y))
                {
                    i0 = i;
                    x0 = x;
                }
            }

            int[] hull = new int[Constants.maxPolygonVertices];
            int m = 0;
            int ih = i0;

            for (;;)
            {
                hull[m] = ih;

                int ie = 0;
                for (int j = 1; j < n; ++j)
                {
                    if (ie == ih)
                    {
                        ie = j;
                        continue;
                    }

                    Vector2 r = ps[ie] - ps[hull[m]];
                    Vector2 v = ps[j] - ps[hull[m]];
                    float c = Calculus.Cross(r, v);
                    if (c < 0.0f)
                        ie = j;

                    // Collinearity check
                    if (c == 0.0f && v.sqrMagnitude > r.sqrMagnitude)
                        ie = j;
                }

                m++;
                ih = ie;

                if (ie == i0)
                    break;
            }

            if (m < 3)
            {
                // Polygon is degenerate.
                Debug.Assert(false);
                SetAsBox(1.0f, 1.0f);
                return;
            }

            count = m;

            // Copy vertices.
            for (int i = 0; i < m; i++)
                vertices[i] = ps[hull[i]];

            // Compute normals. Ensure the edges have non-zero length.
            for (int i = 0; i < m; i++)
            {
                int i1 = i;
                int i2 = i + 1 < m ? i + 1 : 0;
                Vector2 edge = vertices[i2] - vertices[i1];
                Debug.Assert(edge.sqrMagnitude > Mathf.Epsilon * Mathf.Epsilon);
                normals[i] = Calculus.Cross(edge, 1f);
                normals[i].Normalize();
            }

            // Compute the polygon centroid.
            centroid = ComputeCentroid(vertices, m);
        }


        public bool TestPoint(Transformation transform, Vector2 point)
        {
            Vector2 pLocal = Calculus.MulT(transform.rotation, point - transform.position);

            for (int i = 0; i < count; i++)
            {
                float dot = Calculus.Dot(normals[i], pLocal - vertices[i]);
                if (dot > 0.0f)
                    return false;
            }

            return true;
        }

        public bool RayCast(
            ref RayCastOuput output, RayCastInput input,
            Transformation transform)
        {

            // Put the ray into the polygon's frame of reference.
            Vector2 origin = Calculus.MulT(transform.rotation, input.origin - transform.position);
            Vector2 target = Calculus.MulT(transform.rotation, input.target - transform.position);
            Vector2 direction = target - origin;

            float lower = 0.0f, upper = input.maxFraction;

            int index = -1;

	        for (int i = 0; i< count; i++)
	        {
		        // p = p1 + a * d
		        // dot(normal, p - v) = 0
		        // dot(normal, p1 - v) + a * dot(normal, d) = 0
		        float numerator = Calculus.Dot(normals[i], vertices[i] - origin);
                float denominator = Calculus.Dot(normals[i], direction);

		        if (denominator == 0.0f)
		        {	
			        if (numerator< 0.0f)
				        return false;
		        }
		        else
		        {
			        // Note: we want this predicate without division:
			        // lower < numerator / denominator, where denominator < 0
			        // Since denominator < 0, we have to flip the inequality:
			        // lower < numerator / denominator <==> denominator * lower > numerator.
			        if (denominator< 0.0f && numerator<lower* denominator)
			        {
				        // Increase lower.
				        // The segment enters this half-space.
				        lower = numerator / denominator;
				        index = i;
			        }
			        else if (denominator > 0.0f && numerator<upper* denominator)
			        {
				        // Decrease upper.
				        // The segment exits this half-space.
				        upper = numerator / denominator;
			        }
		        }

		        // The use of epsilon here causes the assert on lower to trip
		        // in some cases. Apparently the use of epsilon was to make edge
		        // shapes work, but now those are handled separately.
		        //if (upper < lower - b2_epsilon)
		        if (upper<lower)
			        return false;
	        }

            Debug.Assert(0.0f <= lower && lower <= input.maxFraction);

	        if (index >= 0)
	        {
                output = new RayCastOuput(
                    Calculus.Mul(transform.rotation, 
                    normals[index]), lower);
		        return true;
	        }

	        return false;
        }

        public AABB computeAABB(Transformation transform)
        {
            Vector2 lower = Calculus.Mul(transform, vertices[0]);
            Vector2 upper = lower;

            for (int i = 1; i < count; i++)
            {
                Vector2 v = Calculus.Mul(transform, vertices[i]);
                lower = Vector2.Min(lower, v);
                upper = Vector2.Max(upper, v);
            }

            Vector2 r = new Vector2(radius, radius);
            return new AABB(lower - r, upper + r);
        }

        public MassData computeMassData(float mass)
        {
            // Polygon mass, centroid, and inertia.
            // Let rho be the polygon density in mass per unit area.
            // Then:
            // mass = rho * int(dA)
            // centroid.x = (1/mass) * rho * int(x * dA)
            // centroid.y = (1/mass) * rho * int(y * dA)
            // I = rho * int((x*x + y*y) * dA)
            //
            // We can compute these integrals by summing all the integrals
            // for each triangle of the polygon. To evaluate the integral
            // for a single triangle, we make a change of variables to
            // the (u,v) coordinates of the triangle:
            // x = x0 + e1x * u + e2x * v
            // y = y0 + e1y * u + e2y * v
            // where 0 <= u && 0 <= v && u + v <= 1.
            //
            // We integrate u from [0,1-v] and then v from [0,1].
            // We also need to use the Jacobian of the transformation:
            // D = cross(e1, e2)
            //
            // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
            //
            // The rest of the derivation is handled by computer algebra.

            Vector2 center = new Vector2(0.0f, 0.0f);
            float area = 0.0f;
            float I = 0.0f;

            // s is the reference point for forming triangles.
            // It's location doesn't change the result (except for rounding error).
            Vector2 s = new Vector2(0.0f, 0.0f);

            // This code would put the reference point inside the polygon.
            for (int i = 0; i < count; i++)
                s += vertices[i];

            s *= 1.0f / count;

            const float k_inv3 = 1.0f / 3.0f;

            for (int i = 0; i < count; i++)
            {
                // Triangle vertices.
                Vector2 e1 = vertices[i] - s;
                Vector2 e2 = i + 1 < count ? vertices[i + 1] - s : vertices[0] - s;

                float D = Calculus.Cross(e1, e2);

                float triangleArea = 0.5f * D;
                area += triangleArea;

                // Area weighted centroid
                center += triangleArea * k_inv3 * (e1 + e2);

                float ex1 = e1.x, ey1 = e1.y;
                float ex2 = e2.x, ey2 = e2.y;

                float intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
                float inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

                I += (0.25f * k_inv3 * D) * (intx2 + inty2);
            }

            Debug.Assert(area > Mathf.Epsilon);

            float density = mass / area;
            center *= 1.0f / area;

            I = density * I;

            return new MassData(
                mass, center + s,
                I + mass * (Calculus.Dot(center + s, center + s) - Calculus.Dot(center, center)));
        }

    }
}