using UnityEngine;

namespace CrispyPhysics
{
    public class Entity : MonoBehaviour
    {
        public float mass = 0f;
        [Range(0f, 1f)] public float linearDrag = 0f;
        [Range(0f, 1f)] public float angularDrag = 0f;
        [Range(0f, 1f)] public float gravityScale = 1f;
        public bool freezeRotation = false;
        public RigidbodySleepMode2D sleepMode;

        public Engine engine;

        private Engine parentEngine;

        [HideInInspector]
        public IBody body { get; private set; }

        void Awake()
        {
            mass = (mass > 0) ? mass : 1f;
        }

        void Start()
        {
            if(engine)
            {
                body = CreateBody();
                engine.AddEntity(this);
                parentEngine = engine;
            }
        }

        void Update()
        {
            if(engine != parentEngine)
            {
                Body newBody = (engine) ? CreateBody(body) : null;

                if (parentEngine != null)
                    parentEngine.RemoveEntity(this);

                body = newBody;
                if (engine) engine.AddEntity(this);

                parentEngine = engine;
            }
            else if(body != null)
                UpdateBody();
        }

        void FixedUpdate()
        {
        }

        public void SynchronizeBody()
        {
            Debug.Assert(body != null);
            transform.position = new Vector3(
                    body.position.x,
                    body.position.y,
                    transform.position.z
                );

            transform.rotation = Quaternion.Euler(0f, 0f, body.angle * Mathf.Rad2Deg);
        }

        private void UpdateBody()
        {
            Debug.Assert(body != null);
            if (body.mass != mass) body.SetMass(mass);
            if (body.linearDamping != linearDrag) body.linearDamping = linearDrag;
            if (body.angularDamping != angularDrag) body.angularDamping = angularDrag;
            if (body.gravityScale != gravityScale) body.gravityScale = gravityScale;
            if (body.IsFixedRotation() != freezeRotation) body.SetFixedRotation(freezeRotation);

            if  (
                    body.IsSleepingAllowed()
                && (sleepMode == RigidbodySleepMode2D.NeverSleep)
                )
                body.SetSleepingAllowed(sleepMode != RigidbodySleepMode2D.NeverSleep);
        }

        private Body CreateBody(IBody oldBody = null)
        {
            Debug.Assert(engine != null);
            if (oldBody != null)
                return new Body(
                    BodyType.DynamicBody,
                    oldBody.position,
                    oldBody.angle,
                    linearDrag,
                    angularDrag,
                    mass,
                    gravityScale,
                    freezeRotation,
                    sleepMode != RigidbodySleepMode2D.NeverSleep,
                    oldBody.IsAwake(),
                    oldBody.IsActive()
                );
            else
                return new Body(
                    BodyType.DynamicBody,
                    new Vector2(transform.position.x, transform.position.y),
                    transform.eulerAngles.z,
                    linearDrag,
                    angularDrag,
                    mass,
                    gravityScale,
                    freezeRotation,
                    sleepMode != RigidbodySleepMode2D.NeverSleep,
                    true,
                    true
                );
        }
    }
}

