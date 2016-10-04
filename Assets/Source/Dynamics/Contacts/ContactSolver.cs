using System;
using System.Collections.Generic;
using UnityEngine;
namespace CrispyPhysics.Internal
{
    #region Constraints
    public struct VelocityConstraintPoint
    {
        public readonly Vector2 rA;
        public readonly Vector2 rB;
        public readonly float normalImpulse;
        public readonly float tangentImpulse;
        public readonly float normalMass;
        public readonly float tangentMass;
        public readonly float velocityBias;

        public VelocityConstraintPoint(
            Vector2 rA, Vector2 rB, 
            float normalImpulse, float tangentImpulse,
            float normalMass, float tangentMass, float velocityBias)
        {
            this.rA = rA;
            this.rB = rB;
            this.normalImpulse = normalImpulse;
            this.tangentImpulse = tangentImpulse;
            this.normalMass = normalMass;
            this.tangentMass = tangentMass;
            this.velocityBias = velocityBias;
        }
    }

    public class ContactPositionConstraint
    {
        public Vector2[] points { get; private set; }
        public uint pointCount { get; private set; }
        public Vector2 normal { get; set; }
        public Vector2 point { get; set; }
        public uint indexA { get; set; }
        public uint indexB { get; set; }
        public float invMassA { get;  set; }
        public float invMassB { get; set; }
        public Vector2 centerA { get; set; }
        public Vector2 centerB { get; set; }
        public float invIA { get;  set; }
        public float invIB { get; set; }
        public Manifold.Type type { get; set; }
        public float radiusA { get; set; }
        public float radiusB { get; set; }

        public ContactPositionConstraint()
        {
            points = new Vector2[Constants.maxManifoldPoints];
            pointCount = 0;
        }

        public void AddPoint(Vector2 point)
        {
            Debug.Assert(pointCount < Constants.maxManifoldPoints);

            points[pointCount] = point;
            pointCount++;
        }
    }

    public class ContactVelocityConstraint
    {
        public VelocityConstraintPoint[] points { get; private set; }
        public uint pointCount { get; private set; }
        public uint contactIndex { get; set; }
        public Vector2 normal { get; set; }
        public Matrix2x2 normalMass { get; set; }
        public Matrix2x2 K { get; set; }
        public uint indexA { get; set; }
        public uint indexB { get; set; }
        public float invMassA { get; set; }
        public float invMassB { get; set; }
        public float invIA { get; set; }
        public float invIB { get; set; }
        public float friction { get; set; }
        public float restitution { get; set; }
        public float tangentSpeed { get; set; }

        public ContactVelocityConstraint()
        {
            points = new VelocityConstraintPoint[Constants.maxManifoldPoints];
            pointCount = 0;
        }

        public void AddPoint(VelocityConstraintPoint point)
        {
            Debug.Assert(pointCount < Constants.maxManifoldPoints);

            points[pointCount] = point;
            pointCount++;
        }

        public VelocityConstraintPoint PopPoint()
        {
            if (pointCount == 0)
                throw new InvalidOperationException("Does not contain any Point");

            VelocityConstraintPoint point = points[pointCount];
            pointCount--;

            return point;
        }
    }

    struct PositionSolverManifold
    {
        public readonly Vector2 normal;
        public readonly Vector2 point;
        public readonly float separation;

        public PositionSolverManifold(
            ContactPositionConstraint pc, 
            Transformation xfA, Transformation xfB, uint index)
        {
            if (pc == null)
                throw new ArgumentNullException("pc shoudld not be null");
            if (pc.pointCount <= 0)
                throw new ArgumentException("pc's point count should be greater than 0");
            if (index >= pc.pointCount)
                throw new ArgumentOutOfRangeException("index should be lesser than pc's point count");

            switch (pc.type)
            {
                case Manifold.Type.Circles:
                    {
                        Vector2 pointA = Calculus.Mul(xfA, pc.point);
                        Vector2 pointB = Calculus.Mul(xfB, pc.points[0]);

                        normal = (pointB - pointA).normalized;
                        point = 0.5f * (pointA + pointB);
                        separation = Calculus.Dot(pointB - pointA, normal) - pc.radiusA - pc.radiusB;
                    }
                    break;
                case Manifold.Type.FaceA:
                    {
                        normal = Calculus.Mul(xfA.rotation, pc.normal);
                        Vector2 planePoint = Calculus.Mul(xfA, pc.point);

                        Vector2 clipPoint = Calculus.Mul(xfB, pc.points[index]);
                        separation = Calculus.Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
                        point = clipPoint;
                    }
                    break;
                case Manifold.Type.FaceB:
                    {
                        normal = Calculus.Mul(xfB.rotation, pc.normal);
                        Vector2 planePoint = Calculus.Mul(xfB, pc.point);

                        Vector2 clipPoint = Calculus.Mul(xfA, pc.points[index]);
                        separation = Calculus.Dot(clipPoint - planePoint, normal) - pc.radiusA - pc.radiusB;
                        point = clipPoint;

                        // Ensure normal points from A to B
                        normal = -normal;
                    }
                    break;
                default:
                    normal = Vector2.zero;
                    point = Vector2.zero;
                    separation = 0f;
                    break;
            }
        }
    }
    #endregion

    #region Solver Defintion
    public struct ContactSolverDefinition
    {
        public readonly TimeStep step;
        public readonly Contact[] contacts;
        public readonly uint count;
        public readonly Position[] positions;
        public readonly Velocity[] velocities;

        public ContactSolverDefinition(
            TimeStep step, Contact[] contacts,
            uint count, Position[] positions, Velocity[] velocities)
        {
            this.step = step;
            this.contacts = contacts;
            this.count = count;
            this.positions = positions;
            this.velocities = velocities;
        }
    };
    #endregion

    #region Solver
    public class ContactSolver
    {
        #region Constructors
        public ContactSolver(
            TimeStep step, Contact[] contacts,
            uint count, Position[] positions, Velocity[] velocities)
        {
            if (contacts == null)
                throw new ArgumentNullException("contacts should not be null");
            if (count <= 0)
                throw new ArgumentOutOfRangeException("Count should be greater than 0");
            if (positions == null)
                throw new ArgumentNullException("positions should not be null");
            if (velocities == null)
                throw new ArgumentNullException("velocities should not be null");

            this.step = step;
            this.contacts = contacts;
            this.count = count;
            this.positions = positions;
            this.velocities = velocities;

            positionConstraints = new ContactPositionConstraint[this.count];
            velocityConstraints = new ContactVelocityConstraint[this.count];

            Debug.Assert(count <= contacts.Length);
            for (uint i = 0; i < count; i++)
            {
                Contact contact = contacts[i];
                Debug.Assert(contact.futur.manifold != null);
                Debug.Assert(contact.futur.manifold.pointCount > 0);
                Debug.Assert(contact.bodyA != null);
                Debug.Assert(contact.bodyB != null);
                Debug.Assert(contact.bodyA.shape != null);
                Debug.Assert(contact.bodyB.shape != null);

                Body bodyA = contact.bodyA;
                Body bodyB = contact.bodyB;
                IShape shapeA = bodyA.shape;
                IShape shapeB = bodyB.shape;
                float radiusA = shapeA.radius;
                float radiusB = shapeB.radius;
                Manifold manifold = contact.futur.manifold;

                uint pointCount = manifold.pointCount;

                ContactVelocityConstraint vc = new ContactVelocityConstraint();
                velocityConstraints[i] = vc;
                vc.friction = contact.friction;
                vc.restitution = contact.restitution;
                vc.tangentSpeed = contact.futur.tangentSpeed;
                vc.indexA = bodyA.islandIndex;
                vc.indexB = bodyB.islandIndex;
                vc.invMassA = bodyA.invMass;
                vc.invMassB = bodyB.invMass;
                vc.invIA = bodyA.invRotationalInertia;
                vc.invIB = bodyB.invRotationalInertia;
                vc.contactIndex = i;
                vc.K = Matrix2x2.zero;
                vc.normalMass = Matrix2x2.zero;

                ContactPositionConstraint pc = new ContactPositionConstraint();
                positionConstraints[i] = pc;
                pc.indexA = bodyA.islandIndex;
                pc.indexB = bodyB.islandIndex;
                pc.invMassA = bodyA.invMass;
                pc.invMassB = bodyB.invMass;
                pc.centerA = bodyA.center;
                pc.centerB = bodyB.center;
                pc.invIA = bodyA.invRotationalInertia;
                pc.invIB = bodyB.invRotationalInertia;
                pc.normal = manifold.normal;
                pc.point = manifold.point;
                pc.radiusA = radiusA;
                pc.radiusB = radiusB;
                pc.type = manifold.type;

                for (uint j = 0; j < pointCount; j++)
                {
                    vc.AddPoint(new VelocityConstraintPoint(
                        Vector2.zero, Vector2.zero, 0f, 0f, 0f, 0f, 0f));
                    pc.AddPoint(manifold.points[j].localPoint);
                }
            }
        }

        public ContactSolver(ContactSolverDefinition csDef) : this (
            csDef.step, csDef.contacts, csDef.count, csDef.positions, csDef.velocities)
        {}
        #endregion

        #region nature
        public TimeStep step { get; private set; }
        public Contact[] contacts { get; private set; }
        public uint count { get; private set; }
        public Position[] positions { get; private set; }
        public Velocity[] velocities { get; private set; }
        public ContactPositionConstraint[] positionConstraints { get; private set; }
        public ContactVelocityConstraint[] velocityConstraints { get; private set; }
        #endregion

        #region Initialization
        public void InitializeVelocityConstraints()
        {
            for (uint i = 0; i < count; i++)
            {
                Contact contact = contacts[i];
                ContactPositionConstraint pc = positionConstraints[i];
                ContactVelocityConstraint vc = velocityConstraints[i];

                float radiusA = pc.radiusA;
                float radiusB = pc.radiusB;
                Manifold manifold = contact.futur.manifold;

                uint indexA = vc.indexA;
                uint indexB = vc.indexB;

                float mA = vc.invMassA;
                float mB = vc.invMassB;
                float iA = vc.invIA;
                float iB = vc.invIB;
                Vector2 localCenterA = pc.centerA;
                Vector2 localCenterB = pc.centerB;

                Vector2 cA = positions[indexA].center;
                float aA = positions[indexA].angle;
                Vector2 vA = velocities[indexA].linearVelocity;
                float wA = velocities[indexA].angularVelocity;

                Vector2 cB = positions[indexB].center;
                float aB = positions[indexB].angle;
                Vector2 vB = velocities[indexB].linearVelocity;
                float wB = velocities[indexB].angularVelocity;

                Debug.Assert(manifold.pointCount > 0);

                Rotation rotA = new Rotation(aA);
                Transformation xfA = new Transformation(
                    cA - Calculus.Mul(rotA, localCenterA),
                    rotA);

                Rotation rotB = new Rotation(aB);
                Transformation xfB = new Transformation(
                    cB - Calculus.Mul(rotB, localCenterB),
                    rotB);

                WorldManifold worldManifold = new WorldManifold(
                    manifold, xfA, radiusA, xfB, radiusB);

                vc.normal = worldManifold.normal;

                uint pointCount = vc.pointCount;

                for (uint j = 0; j < pointCount; ++j)
                {
                    VelocityConstraintPoint vcp = vc.points[j];

                    Vector2 rA = worldManifold.points[j] - cA;
                    Vector2 rB = worldManifold.points[j] - cB;

                    float rnA = Calculus.Cross(rA, vc.normal);
                    float rnB = Calculus.Cross(rB, vc.normal);

                    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    float normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                    Vector2 tangent = Calculus.Cross(vc.normal, 1.0f);

                    float rtA = Calculus.Cross(rA, tangent);
                    float rtB = Calculus.Cross(rB, tangent);

                    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

                    float tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                    // Setup a velocity bias for restitution.
                    float velocityBias = 0.0f;
                    float vRel = Calculus.Dot(
                        vc.normal, 
                        vB + Calculus.Cross(wB, rB) - vA - Calculus.Cross(wA, rA));

                    if (vRel < -Constants.velocityThreshold)
                        velocityBias = -vc.restitution * vRel;

                    vc.points[j] = new VelocityConstraintPoint(
                        rA, rB,
                        vcp.normalImpulse, vcp.tangentImpulse,
                        normalMass, tangentMass,
                        velocityBias);
                }

                // If we have two points, then prepare the block solver.
                if (vc.pointCount == 2)
                {
                    VelocityConstraintPoint vcp1 = vc.points[0];
                    VelocityConstraintPoint vcp2 = vc.points[1];

                    float rn1A = Calculus.Cross(vcp1.rA, vc.normal);
                    float rn1B = Calculus.Cross(vcp1.rB, vc.normal);
                    float rn2A = Calculus.Cross(vcp2.rA, vc.normal);
                    float rn2B = Calculus.Cross(vcp2.rB, vc.normal);

                    float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
                    float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
                    float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

                    // Ensure a reasonable condition number.
                    const float k_maxConditionNumber = 1000.0f;
                    if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
                    {
                        // K is safe to invert.
                        vc.K = new Matrix2x2(k11, k12, k12, k22);
                        vc.normalMass = vc.K.GetInverse();
                    }
                    else
                        vc.PopPoint();// The constraints are redundant, just use one.
                }
            }
        }
        #endregion

        #region Solvers

        public void SolveVelocityConstraints()
        {
            for(uint i =0; i < count; i++)
            {
                ContactVelocityConstraint vc = velocityConstraints[i];
                uint indexA = vc.indexA;
                uint indexB = vc.indexB;
                float mA = vc.invMassA;
                float iA = vc.invIA;
                float mB = vc.invMassB;
                float iB = vc.invIB;
                uint pointCount = vc.pointCount;

                Vector2 vA = velocities[indexA].linearVelocity;
                float wA = velocities[indexA].angularVelocity;
                Vector2 vB = velocities[indexB].linearVelocity;
                float wB = velocities[indexB].angularVelocity;

                Vector2 normal = vc.normal;
                Vector2 tangent = Calculus.Cross(normal, 1.0f);
                float friction = vc.friction;

                Debug.Assert(pointCount == 1 || pointCount == 2);
                // Solve tangent constraints first because non-penetration is more important
                // than friction.
                for (uint j = 0; j < pointCount; j++)
                {
                    VelocityConstraintPoint vcp = vc.points[j];

                    // Relative velocity at contact
                    Vector2 dv = vB + Calculus.Cross(wB, vcp.rB) - vA - Calculus.Cross(wA, vcp.rA);

                    // Compute tangent force
                    float vt = Calculus.Dot(dv, tangent) - vc.tangentSpeed;
                    float lambda = vcp.tangentMass * (-vt);

                    // b2Clamp the accumulated force
                    float maxFriction = friction * vcp.normalImpulse;

                    float newImpulse = Mathf.Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
                    lambda = newImpulse - vcp.tangentImpulse;
                    vc.points[j] = new VelocityConstraintPoint(
                        vcp.rA, vcp.rB, 
                        vcp.normalImpulse, newImpulse, vcp.normalMass, vcp.tangentMass, 
                        vcp.velocityBias);

                    // Apply contact impulse
                    Vector2 P = lambda * tangent;

                    vA -= mA * P;
                    wA -= iA * Calculus.Cross(vcp.rA, P);

                    vB += mB * P;
                    wB += iB * Calculus.Cross(vcp.rB, P);
                }

                // Solve normal constraints
                if (pointCount == 1)
                {
                    for (uint j = 0; j < pointCount; j++)
                    {
                        VelocityConstraintPoint vcp = vc.points[j];

                        // Relative velocity at contact
                        Vector2 dv = vB + Calculus.Cross(wB, vcp.rB) - vA - Calculus.Cross(wA, vcp.rA);

                        // Compute normal impulse
                        float vn = Calculus.Dot(dv, normal);
                        float lambda = -vcp.normalMass * (vn - vcp.velocityBias);

                        // b2Clamp the accumulated impulse
                        float newImpulse = Mathf.Max(vcp.normalImpulse + lambda, 0.0f);
                        lambda = newImpulse - vcp.normalImpulse;
                        vc.points[j] = new VelocityConstraintPoint(
                        vcp.rA, vcp.rB,
                        newImpulse, vcp.tangentImpulse, vcp.normalMass, vcp.tangentMass,
                        vcp.velocityBias);

                        // Apply contact impulse
                        Vector2 P = lambda * normal;
                        vA -= mA * P;
                        wA -= iA * Calculus.Cross(vcp.rA, P);

                        vB += mB * P;
                        wB += iB * Calculus.Cross(vcp.rB, P);
                    }
                }
                else
                {
                    // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                    // Build the mini LCP for this contact patch
                    //
                    // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                    //
                    // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                    // b = vn0 - velocityBias
                    //
                    // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                    // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                    // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                    // solution that satisfies the problem is chosen.
                    // 
                    // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                    // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                    //
                    // Substitute:
                    // 
                    // x = a + d
                    // 
                    // a := old total impulse
                    // x := new total impulse
                    // d := incremental impulse 
                    //
                    // For the current iteration we extend the formula for the incremental impulse
                    // to compute the new total impulse:
                    //
                    // vn = A * d + b
                    //    = A * (x - a) + b
                    //    = A * x + b - A * a
                    //    = A * x + b'
                    // b' = b - A * a;

                    VelocityConstraintPoint cp1 = vc.points[0];
                    VelocityConstraintPoint cp2 = vc.points[1];

                    Vector2 a = new Vector2(cp1.normalImpulse, cp2.normalImpulse);
                    Debug.Assert(a.x >= 0.0f && a.y >= 0.0f);

                    // Relative velocity at contact
                    Vector2 dv1 = vB + Calculus.Cross(wB, cp1.rB) - vA - Calculus.Cross(wA, cp1.rA);
                    Vector2 dv2 = vB + Calculus.Cross(wB, cp2.rB) - vA - Calculus.Cross(wA, cp2.rA);

                    // Compute normal velocity
                    float vn1 = Calculus.Dot(dv1, normal);
                    float vn2 = Calculus.Dot(dv2, normal);

                    Vector2 b;
                    b.x = vn1 - cp1.velocityBias;
                    b.y = vn2 - cp2.velocityBias;

                    // Compute b'
                    b -= Calculus.Mul(vc.K, a);

                    for (;;)
                    {
                        //
                        // Case 1: vn = 0
                        //
                        // 0 = A * x + b'
                        //
                        // Solve for x:
                        //
                        // x = - inv(A) * b'
                        //
                        Vector2 x = -Calculus.Mul(vc.normalMass, b);

                        if (x.x >= 0.0f && x.y >= 0.0f)
                        {
                            // Get the incremental impulse
                            Vector2 d = x - a;

                            // Apply incremental impulse
                            Vector2 P1 = d.x * normal;
                            Vector2 P2 = d.y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (Calculus.Cross(cp1.rA, P1) + Calculus.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (Calculus.Cross(cp1.rB, P1) + Calculus.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1 = new VelocityConstraintPoint(
                                cp1.rA, cp1.rB,
                                x.x, cp1.tangentImpulse, cp1.normalMass, cp1.tangentMass,
                                cp1.velocityBias);
                            cp2 = new VelocityConstraintPoint(
                                cp2.rA, cp2.rB,
                                x.y, cp2.tangentImpulse, cp2.normalMass, cp2.tangentMass,
                                cp2.velocityBias);
                            break;
                        }

                        //
                        // Case 2: vn1 = 0 and x2 = 0
                        //
                        //   0 = a11 * x1 + a12 * 0 + b1' 
                        // vn2 = a21 * x1 + a22 * 0 + b2'
                        //
                        x.x = -cp1.normalMass * b.x;
                        x.y = 0.0f;
                        vn1 = 0.0f;
                        vn2 = vc.K.ex.y * x.x + b.y;
                        if (x.x >= 0.0f && vn2 >= 0.0f)
                        {
                            // Get the incremental impulse
                            Vector2 d = x - a;

                            // Apply incremental impulse
                            Vector2 P1 = d.x * normal;
                            Vector2 P2 = d.y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (Calculus.Cross(cp1.rA, P1) + Calculus.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (Calculus.Cross(cp1.rB, P1) + Calculus.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1 = new VelocityConstraintPoint(
                                cp1.rA, cp1.rB,
                                x.x, cp1.tangentImpulse, cp1.normalMass, cp1.tangentMass,
                                cp1.velocityBias);
                            cp2 = new VelocityConstraintPoint(
                                cp2.rA, cp2.rB,
                                x.y, cp2.tangentImpulse, cp2.normalMass, cp2.tangentMass,
                                cp2.velocityBias);
                            break;
                        }


                        //
                        // Case 3: vn2 = 0 and x1 = 0
                        //
                        // vn1 = a11 * 0 + a12 * x2 + b1' 
                        //   0 = a21 * 0 + a22 * x2 + b2'
                        //
                        x.x = 0.0f;
                        x.y = -cp2.normalMass * b.y;
                        vn1 = vc.K.ey.x * x.y + b.x;
                        vn2 = 0.0f;

                        if (x.y >= 0.0f && vn1 >= 0.0f)
                        {
                            // Resubstitute for the incremental impulse
                            Vector2 d = x - a;

                            // Apply incremental impulse
                            Vector2 P1 = d.x * normal;
                            Vector2 P2 = d.y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (Calculus.Cross(cp1.rA, P1) + Calculus.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (Calculus.Cross(cp1.rB, P1) + Calculus.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1 = new VelocityConstraintPoint(
                                cp1.rA, cp1.rB,
                                x.x, cp1.tangentImpulse, cp1.normalMass, cp1.tangentMass,
                                cp1.velocityBias);
                            cp2 = new VelocityConstraintPoint(
                                cp2.rA, cp2.rB,
                                x.y, cp2.tangentImpulse, cp2.normalMass, cp2.tangentMass,
                                cp2.velocityBias);
                            break;
                        }

                        //
                        // Case 4: x1 = 0 and x2 = 0
                        // 
                        // vn1 = b1
                        // vn2 = b2;
                        x.x = 0.0f;
                        x.y = 0.0f;
                        vn1 = b.x;
                        vn2 = b.y;

                        if (vn1 >= 0.0f && vn2 >= 0.0f)
                        {
                            // Resubstitute for the incremental impulse
                            Vector2 d = x - a;

                            // Apply incremental impulse
                            Vector2 P1 = d.x * normal;
                            Vector2 P2 = d.y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (Calculus.Cross(cp1.rA, P1) + Calculus.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (Calculus.Cross(cp1.rB, P1) + Calculus.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1 = new VelocityConstraintPoint(
                                cp1.rA, cp1.rB,
                                x.x, cp1.tangentImpulse, cp1.normalMass, cp1.tangentMass,
                                cp1.velocityBias);
                            cp2 = new VelocityConstraintPoint(
                                cp2.rA, cp2.rB,
                                x.y, cp2.tangentImpulse, cp2.normalMass, cp2.tangentMass,
                                cp2.velocityBias);
                            break;
                        }
                        // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                        break;
                    }
                }
                velocities[indexA] = new Velocity(vA, wA);
                velocities[indexB] = new Velocity(vB, wB);
            }
        }

        public void StoreImpulses()
        {
            for (uint i =0; i < count; i++)
            {
                Contact contact = contacts[i];
                ContactVelocityConstraint vc = velocityConstraints[i];
                Manifold manifold = contact.futur.manifold;

                for (uint j = 0; j < vc.pointCount; j++)
                    manifold.points[j] = new ManifoldPoint(
                        manifold.points[j].id, 
                        manifold.points[j].localPoint,
                        vc.points[j].normalImpulse,
                        vc.points[j].tangentImpulse);
            }
        }

        public bool SolvePositionConstraints()
        {
            float minSeparation = 0.0f;

            for (uint i = 0; i < count; i++)
            {
                ContactPositionConstraint pc = positionConstraints[i];

                uint indexA = pc.indexA;
                uint indexB = pc.indexB;
                Vector2 centerA = pc.centerA;
                float mA = pc.invMassA;
                float iA = pc.invIA;
                Vector2 centerB = pc.centerB;
                float mB = pc.invMassB;
                float iB = pc.invIB;
                uint pointCount = pc.pointCount;

                Vector2 cA = positions[indexA].center;
                float aA = positions[indexA].angle;

                Vector2 cB = positions[indexB].center;
                float aB = positions[indexB].angle;

                // Solve normal constraints
                for (uint j = 0; j < pointCount; ++j)
                {
                    Rotation rotA = new Rotation(aA);
                    Transformation xfA = new Transformation(
                        cA - Calculus.Mul(rotA, centerA),
                        rotA);
                    Rotation rotB = new Rotation(aB);
                    Transformation xfB = new Transformation(
                        cB - Calculus.Mul(rotB, centerB),
                        rotB);

                    PositionSolverManifold psm = new PositionSolverManifold(pc, xfA, xfB, j);

                    Vector2 normal = psm.normal;
                    Vector2 point = psm.point;
                    float separation = psm.separation;

                    Vector2 rA = point - cA;
                    Vector2 rB = point - cB;

                    // Track max constraint error.
                    minSeparation = Mathf.Min(minSeparation, separation);

                    // Prevent large corrections and allow slop.
                    float C = Mathf.Clamp(
                        Constants.baumgarte * (separation + Constants.linearSlop), 
                        -Constants.maxLinearCorrection,
                        0.0f);

                    // Compute the effective mass.
                    float rnA = Calculus.Cross(rA, normal);
                    float rnB = Calculus.Cross(rB, normal);
                    float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    // Compute normal impulse
                    float impulse = K > 0.0f ? -C / K : 0.0f;

                    Vector2 P = impulse * normal;

                    cA -= mA * P;
                    aA -= iA * Calculus.Cross(rA, P);

                    cB += mB * P;
                    aB += iB * Calculus.Cross(rB, P);
                }

                positions[indexA] = new Position(cA, aA);
                positions[indexB] = new Position(cB, aB);
            }

            // We can't expect minSpeparation >= -b2_linearSlop because we don't
            // push the separation above -b2_linearSlop.
            return minSeparation >= -3.0f * Constants.linearSlop;
        }
        #endregion

    }
    #endregion
}