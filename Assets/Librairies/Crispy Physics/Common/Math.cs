using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region Rotation
    public struct Rotation
    {
        public static Rotation identity = new Rotation(0f);
        public readonly float sine;
        public readonly float cosine;

        public Rotation(float angle)
        {
            sine = Mathf.Sin(angle);
            cosine = Mathf.Cos(angle);
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
    #endregion;

    #region Transformation
    public struct Transformation
    {
        public static Transformation identity = new Transformation(
            Vector2.zero,
            Rotation.identity);

        public readonly Vector2 position;
        public readonly Rotation rotation;

        public Transformation(Vector2 position, Rotation rotation)
        {
            this.position = position;
            this.rotation = rotation;
        }

        public Transformation(Vector2 position, float angle)
        {
            this.position = position;
            rotation = new Rotation(angle);
        }
    }
    #endregion

    #region Matrices
    public struct Matrix2x2
    {
        public static Matrix2x2 identity = new Matrix2x2(
            Vector2.right,
            Vector2.up);
        public static Matrix2x2 zero = new Matrix2x2(
            Vector2.zero,
            Vector2.zero);

        public readonly Vector2 ex;
        public readonly Vector2 ey;

        public Matrix2x2(Vector2 ex, Vector2 ey)
        {
            this.ex = ex;
            this.ey = ey;
        }

        public Matrix2x2(float a11, float a12, float a21, float a22)
        {
            ex = new Vector2(a11, a21);
            ey = new Vector2(a12, a22);
        }

        public Matrix2x2 GetInverse()
        {
            float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
            float det = a * d - b * c;
            if (det != 0.0f)
                det = 1.0f / det;

            return new Matrix2x2(
                det * d, -det * b,
                -det * c, det * a);
        }

        public Vector2 Solve(Vector2 b)
        {
            float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
            float det = a11 * a22 - a12 * a21;
            if (det != 0.0f)
                det = 1.0f / det;

            return new Vector2(
                det * (a22 * b.x - a12 * b.y),
                det * (a11 * b.y - a21 * b.x));
        }
    }
    #endregion


    #region Calculus
    public class Calculus
    {
        public static bool Approximately(float a, float b, float tolerance = 0)
        {
            if (tolerance < Mathf.Epsilon)
                tolerance = Mathf.Epsilon;
            return Mathf.Abs(a - b) < tolerance;
        }

        public static bool Approximately(Vector2 a, Vector2 b, float tolerance = 0)
        {
            return Approximately(Vector2.SqrMagnitude(a - b), 0f, tolerance * tolerance);
        }

        public static bool Approximately(Rotation a, Rotation b, float tolerance = 0)
        {
            return  (   Approximately(a.sine, b.sine, tolerance)
                    &&  Approximately(a.cosine, b.cosine, tolerance));
        }

        public static bool Approximately(Transformation a, Transformation b, float tolerance = 0)
        {
            return  (   Approximately(a.position, b.position, tolerance)
                    &&  Approximately(a.rotation, b.rotation, tolerance));
        }

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

        public static Vector2 Mul(Matrix2x2 matrix, Vector2 vector)
        {
            return new Vector2(
                matrix.ex.x * vector.x + matrix.ey.x * vector.y,
                matrix.ex.y * vector.x + matrix.ey.y * vector.y);
        }

        public static float Dot(Vector2 vectorA, Vector2 vectorB)
        {
            return vectorA.x * vectorB.x + vectorA.y * vectorB.y;
        }

        public static float Cross(Vector2 vectorA, Vector2 vectorB)
        {
            return vectorA.x * vectorB.y - vectorA.y * vectorB.x;
        }

        public static Vector2 Cross(float s, Vector2 vector)
        {
            return new Vector2(-s * vector.y, s * vector.x);
        }

        public static Vector2 Cross(Vector2 vector, float number)
        {
            return new Vector2(
                number * vector.y,
                -number * vector.x);
        }

        public static Vector2 Abs(Vector2 vector)
        {
            return new Vector2(
                Mathf.Abs(vector.x),
                Mathf.Abs(vector.y));
        }

        public static void Swap<T>(ref T lhs, ref T rhs)
        {
            T temp = lhs;
            lhs = rhs;
            rhs = temp;
        }
    }
    #endregion
}