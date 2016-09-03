using UnityEngine;

namespace CrispyPhysics.Internal
{
    public class EdgeShape : IInternalShape
    {
        public ShapeType type { get { return ShapeType.Edge; } }
        public float radius { get { return Constants.polygonRadius; } }
        public Vector2 vertex1 { get; private set; }
        public Vector2 vertex2 { get; private set; }
        public Vector2 vertex0 { get; set; }
        public Vector2 vertex3 { get; set; }
        public bool hasVertex0 { get; set; }
        public bool hasVertex3 { get; set; }

        public EdgeShape(Vector2 vertex1, Vector2 vertex2)
        {
            Set(vertex1, vertex2);
        }

        public void Set(Vector2 vertex1, Vector2 vertex2)
        {
            this.vertex1 = vertex1;
            this.vertex2 = vertex2;
            hasVertex0 =  hasVertex3 = false;
        }

        public bool TestPoint(Transformation transform, Vector2 point)
        {
            return false;
        }

        public bool RayCast(
            ref RayCastOuput output, RayCastInput input,
            Transformation transform)
        {
            Vector2 origin = Calculus.MulT(
                transform.rotation, 
                input.origin - transform.position);

            Vector2 target = Calculus.MulT(
                transform.rotation,
                input.target - transform.position);

            Vector2 direction = target - origin;

            Vector2 v1 = vertex1;
            Vector2 v2 = vertex2;
            Vector2 e = v2 - v1;
            Vector2 normal = new Vector2(e.y, -e.x);
            normal.Normalize();

            float numerator = Calculus.Dot(normal, v1 - origin);
            float denominator = Calculus.Dot(normal, direction);

            if (denominator == 0.0f)
                return false;

            float t = numerator / denominator;
            if (t < 0.0f || input.maxFraction < t)
                return false;

            Vector2 q = origin + t * direction;

            Vector2 r = v2 - v1;
            float rr = Calculus.Dot(r, r);
            if (rr == 0.0f)
                return false;

            float s = Calculus.Dot(q - v1, r) / rr;
            if (s < 0.0f || 1.0f < s)
                return false;

            output.fraction = t;
            if (numerator > 0.0f)
                output.normal = -Calculus.Mul(transform.rotation, normal);
            else
                output.normal = Calculus.Mul(transform.rotation, normal);

            return true;
        }

        public AABB computeAABB(Transformation transform)
        {
            Vector2 v1 = Calculus.Mul(transform, vertex1);
            Vector2 v2 = Calculus.Mul(transform, vertex2);

            Vector2 lower = Vector2.Min(v1, v2);
            Vector2 upper = Vector2.Max(v1, v2);

            Vector2 r = new Vector2(radius, radius);
            return new AABB(lower - r, upper + r);
        }

        public MassData computeMassData(float mass)
        {
            return new MassData(0f, 0.5f * (vertex1 + vertex2), 0f);
        }
    }
}
