using UnityEngine;
using NUnit.Framework.Constraints;

namespace OwnNUnit
{
    class Vector2EqualConstraint : Constraint
    {
        readonly Vector2 expected;
        double tolerance;

		public override string Description
		{
            get { return expected + " Expected within tolerance " + tolerance; }
		}

        public Vector2EqualConstraint(Vector2 vector)
        {
            expected = vector;
        }

        public Vector2EqualConstraint Within(double tolerance)
        {
            this.tolerance = tolerance;
            return this;
        }

		public override ConstraintResult ApplyTo(object actual)
        {
            bool result = false;
            if ((actual is Vector2))
            {
                Vector2 other = (Vector2)actual;
                result = Mathf.Abs(other.x - expected.x) < tolerance && Mathf.Abs(other.y - expected.y) < tolerance;
            }
            else
                result = false;


            return new ConstraintResult(
                this,
                actual,
                result
            );
        }
    }
}
