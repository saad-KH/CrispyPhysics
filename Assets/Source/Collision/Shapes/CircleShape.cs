using UnityEngine;

namespace CrispyPhysics.Internal
{
    public class CircleShape : IShape
    {
        public ShapeType type { get { return ShapeType.Circle; } }
        public float radius { get; private set; }
        public Vector2 position { get; private set; }

        public CircleShape(Vector2 position, float radius)
        {
            this.radius = radius;
            this.position = position;
        }

        public bool TestPoint(Transformation transform, Vector2 point)
        {
            Vector2 center =
                    transform.position
                +   Calculus.Mul(transform.rotation, position);

            Vector2 delta = position - center;
            return Calculus.Dot(delta, delta) <= radius * radius;
        }

        public bool RayCast(
            ref RayCastOuput output, RayCastInput input,
            Transformation transform)
        {
            Vector2 center = 
                    transform.position 
                +   Calculus.Mul(transform.rotation, position);

            Vector2 s = input.origin - center;
            float b = Calculus.Dot(s, s) - radius * radius;

            // Solve quadratic equation.
            Vector2 r = input.target - input.origin;
            float c = Calculus.Dot(s, r);
            float rr = Calculus.Dot(r, r);
            float sigma = c * c - rr * b;

            // Check for negative discriminant and short segment.
            if (sigma < 0.0f || rr < Mathf.Epsilon)
                return false;

            // Find the point of intersection of the line with the circle.
            float a = -(c + Mathf.Sqrt(sigma));

            // Is the intersection point on the segment?
            if (0.0f <= a && a <= input.maxFraction * rr)
            {
                a /= rr;
                output = new RayCastOuput((s + a * r).normalized, a);
                return true;
            }

            return false;
        }

        public AABB computeAABB(Transformation transform)
        {
            Vector2 center =
                    transform.position
                + Calculus.Mul(transform.rotation, position);
            return new AABB(
                new Vector2(center.x - radius, center.y - radius),
                new Vector2(center.x + radius, center.y + radius));
        }

        public MassData computeMassData(float mass)
        {
            return new MassData(
                mass, position,
                mass * ( 0.5f * radius * radius + Calculus.Dot(position, position)));
        }
    }
}