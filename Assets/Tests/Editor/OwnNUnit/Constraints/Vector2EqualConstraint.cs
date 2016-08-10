using UnityEngine;

namespace OwnNUnit
{
    class Vector2EqualConstraint : NUnit.Framework.Constraints.Constraint
    {
        readonly Vector2 expected;
        double tolerance;

        public Vector2EqualConstraint(Vector2 vector)
        {
            expected = vector;
        }

        public Vector2EqualConstraint Within(double tolerance)
        {
            this.tolerance = tolerance;
            return this;
        }


        public override bool Matches(object obj)
        {
            actual = obj;

            if (!(obj is Vector2))
                return false;

            var other = (Vector2)obj;

            return Mathf.Abs(other.x - expected.x) < tolerance && Mathf.Abs(other.y - expected.y) < tolerance;
        }

        public override void WriteDescriptionTo(NUnit.Framework.Constraints.MessageWriter writer)
        {
            writer.WriteExpectedValue(expected);
            writer.WriteMessageLine("Expected within tolerance '{0}'.", tolerance);
        }

        public override void WriteActualValueTo(NUnit.Framework.Constraints.MessageWriter writer)
        {
            writer.WriteActualValue(actual);
        }
    }
}
