using UnityEngine;

namespace CrispyPhysics
{

    public struct Rotation
    {
        public float sine;
        public float cosine;

        public Rotation(float angle)
        {
            sine = Mathf.Sin(angle);
            cosine = Mathf.Cos(angle);
        }

        public void Set(float angle)
        {
            sine = Mathf.Sin(angle);
            cosine = Mathf.Cos(angle);
        }

        public void SetIdentity()
        {
            sine = 0f;
            cosine = 0f;
        }

        public float GetAngle()
        {
            return Mathf.Atan2(sine, cosine);
        }

        public Vector2 GetXAxis()
        {
            return new Vector2(cosine, sine);
        }

        public Vector2 GetYAxis()
        {
            return new Vector2(-sine, cosine);
        }
    }

    public struct Transformation
    {
        public Vector2 position;
        public Rotation rotation;

        public Transformation(Vector2 position, Rotation rotation)
        {
            this.position = position;
            this.rotation = rotation;
        }

        public void SetIdentity()
        {
            position = Vector2.zero;
            rotation.SetIdentity();
        }

        public void Set(Vector2 position, float angle)
        {
            this.position = position;
            rotation.Set(angle);
        }
    }

    public struct Sweep
    {
        public Vector2 center0, center;
        public float angle0, angle;
        public float alpha0;

        public void Reset(Vector2 center, float angle)
        {
            this.center = center0 = center;
            this.angle = angle0 = angle;
            alpha0 = 0;
        }

        public Transformation GetTransform(float beta)
        {
            Vector2 position = (1.0f - beta) * center0 + beta * center;
            Rotation rotation = new Rotation(
                (1.0f - beta) * angle0 + beta * angle
            );

            return new Transformation(
                position,
                rotation);
        }

        public void Advance(float alpha)
        {
            Debug.Assert(alpha0 < 1f);
            float beta = (alpha - alpha0) / (1f - alpha0);
            center0 += beta * (center - center0);
            angle0 += beta * (angle - angle0);
            alpha0 = alpha;
        }

        public void Normalize()
        {
            float twoPi = 2f * Mathf.PI;
            angle0 -= twoPi * Mathf.Floor(angle0 / twoPi);
            angle -= twoPi * Mathf.Floor(angle / twoPi);
        }
    }

    public class Calculus
    {

        public static Vector2 Mul(Rotation rotation, Vector2 vector)
        {
            return new Vector2(
                rotation.cosine * vector.x - rotation.sine * vector.y,
                rotation.sine * vector.x + rotation.cosine * vector.y
            );
        }

        public static Vector2 MulT(Rotation rotation, Vector2 vector)
        {
            return new Vector2(
                rotation.cosine * vector.x + rotation.sine * vector.y,
                -rotation.sine * vector.x + rotation.cosine * vector.y
            );
        }

        public static Vector2 Mul(Transformation transform, Vector2 vector)
        {
            float x =
                    (transform.rotation.cosine * vector.x - transform.rotation.sine * vector.y)
                + transform.position.x;
            float y =
                    (transform.rotation.sine * vector.x + transform.rotation.cosine * vector.y)
                + transform.position.y;

            return new Vector2(x, y);
        }

        public static Vector2 MulT(Transformation transform, Vector2 vector)
        {
            float px = vector.x - transform.position.x;
            float py = vector.y - transform.position.y;
            float x = transform.rotation.cosine * px + transform.rotation.sine * py;
            float y = -transform.rotation.sine * px + transform.rotation.cosine * py;

            return new Vector2(x, y);
        }

        public static float Cross(Vector2 vectorA, Vector2 vectorB)
        {
            return vectorA.x * vectorB.y - vectorA.y * vectorB.y;
        }
    }
}