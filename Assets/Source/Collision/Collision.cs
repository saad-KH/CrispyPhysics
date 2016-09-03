using UnityEngine;

namespace CrispyPhysics.Internal
{
    #region raycast
    public struct RayCastInput
    {
        public Vector2 origin;
        public Vector2 target;
        public float maxFraction;

        public RayCastInput(Vector2 origin, Vector2 target, float maxFraction)
        {
            this.origin = origin;
            this.target = target;
            this.maxFraction = maxFraction;
        }
    }

    public struct RayCastOuput
    {
        public Vector2 normal;
        public float fraction;

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
        public Vector2 lowerBound;
        public Vector2 upperBound;

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

        public void Combine(AABB aabb)
        {
            lowerBound = Vector2.Min(lowerBound, aabb.lowerBound);
            upperBound = Vector2.Max(upperBound, aabb.upperBound);
        }

        public void Combine(AABB aabb1, AABB aabb2)
        {
            lowerBound = Vector2.Min(aabb1.lowerBound, aabb2.lowerBound);
            upperBound = Vector2.Max(aabb1.upperBound, aabb2.upperBound);
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
            float tmin = -float.MaxValue;
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

            output.fraction = tmin;
            output.normal = new Vector2(normal[0], normal[1]);

            return true;
        }


    }
    #endregion
}