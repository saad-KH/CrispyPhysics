using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace CrispyPhysics
{
    public class Engine : MonoBehaviour
    {
        public float crispSize = 1.0f;
        public float actionTick = 0.25f;

        public bool crisped { get; private set; }

        private List<Entity> entities;
        public IWorld world { get; private set; }

        void Awake()
        {
            crispSize = (crispSize > 0) ? crispSize : 1;
            actionTick = (actionTick > 0) ? actionTick : 10;

            crisped = false;

            entities = new List<Entity>();
            world = new World(crispSize, actionTick);
        }

        void OnEnable()
        {
            StartCoroutine("LateFixedUpdate");
        }

        void OnDisable()
        {
            StopCoroutine("LateFixedUpdate");
        }

        void Update()
        {
            UpdateEngine();
        }

        void LateUpdate()
        {
            crisped = world.IsCrisped();
        }

        private IEnumerator LateFixedUpdate()
        {
            yield return new WaitForFixedUpdate();
            while (true)
            {
                world.Step(
                    Time.fixedDeltaTime,
                    Physics2D.velocityIterations,
                    Physics2D.positionIterations
                );
                crisped = crisped || world.IsCrisped();
                foreach (Entity entity in entities)
                    entity.SynchronizeBody();
                yield return new WaitForFixedUpdate();
            }
        }

        private void UpdateEngine()
        {
            if (world.crispSize != crispSize) world.crispSize = crispSize;
            if (world.actionTick != actionTick) world.actionTick = actionTick;
        }

        public void AddEntity(Entity entity)
        {
            Debug.Assert(entity.body != null);
            entities.Add(entity);
            world.AddBody(entity.body);
        }

        public void RemoveEntity(Entity entity)
        {
            if (entities.IndexOf(entity) == -1) return;
            world.RemoveBody(entity.body);
            entities.Remove(entity);
        }
    }
}