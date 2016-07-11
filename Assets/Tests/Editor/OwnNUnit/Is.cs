using UnityEngine;

namespace OwnNUnit
{
    static class Is
    {
        public static Vector2EqualConstraint EqualTo(Vector2 vector)
        {
            return new Vector2EqualConstraint(vector);
        }
    }
}