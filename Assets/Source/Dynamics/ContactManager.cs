using UnityEngine;
using System;
using System.Collections.Generic;

namespace CrispyPhysics.Internal
{
    #region Delegate Definition
    #endregion

    public class ContactManager
    {
        private BodyIteratorDelegate bodyIterator;
        private NewPairDelegate newPair;
        private ContactIteratorDelegate contactIterator;

        private ContactHandlerDelegate contactStartForeseen;
        private ContactHandlerDelegate contactEndForeseen;
        private ContactHandlerDelegate contactForPreSolving;

        public ContactManager(
            BodyIteratorDelegate bodyIterator,
            NewPairDelegate newPair,
            ContactIteratorDelegate contactIterator,
            ContactHandlerDelegate contactStartForeseen = null,
            ContactHandlerDelegate contactEndForeseen = null,
            ContactHandlerDelegate contactForPreSolving = null)
        {
    		if (bodyIterator == null)
                throw new ArgumentOutOfRangeException("BodyIterator should not be null");
            if (newPair == null)
                throw new ArgumentOutOfRangeException("newPair should not be null");
            if (contactIterator == null)
                throw new ArgumentOutOfRangeException("contactIterator should not be null");

            this.bodyIterator =  bodyIterator;
            this.newPair = newPair;
            this.contactIterator = contactIterator;
            this.contactStartForeseen = contactStartForeseen;
            this.contactEndForeseen = contactEndForeseen;
            this.contactForPreSolving = contactForPreSolving;
        }


    	public void FindNewContacts(){
    		uint i =0;

    		foreach(Body body in bodyIterator())
    		{
                if (body.shape == null)
                    continue;

                AABB aabb = body.shape.computeAABB(body.futur.transform);
    			foreach(Body pairBody in bodyIterator(i + 1)){
                    if (pairBody.shape == null)
                        continue;

                    Debug.Assert(body.futur.tick == pairBody.futur.tick);
                    AABB pairAabb = pairBody.shape.computeAABB(pairBody.futur.transform);
                    if(Collision.TestOverlap(aabb, pairAabb))
                        newPair(body, pairBody);
                }
    			i++;
    		}
    	}

    	public void Collide()
        {
            foreach(Contact contact in contactIterator())
            {
                ContactMomentum futur = contact.futur;

                Manifold oldManifold = futur.manifold;
                bool wasTouching = futur.isTouching;

                Manifold nextManifold = null;
                bool touching = false;
                bool sensor = contact.bodyA.sensor || contact.bodyB.sensor;

                Body bodyA = contact.bodyA;
                Body bodyB = contact.bodyB;

                if (sensor)
                {
                    IShape shapeA = bodyA.shape;
                    IShape shapeB = bodyB.shape;
                    Debug.Assert(shapeA != null && shapeB != null);

                    touching = Collision.TestOverlap(
                        shapeA.computeAABB(bodyA.futur.transform),
                        shapeB.computeAABB(bodyB.futur.transform));
                }
                else
                {
                    nextManifold = contact.Evaluate(
                        bodyA.futur.transform, bodyB.futur.transform);

                    if (nextManifold != null && nextManifold.pointCount > 0
                        && oldManifold != null && oldManifold.pointCount > 0)
                    {
                        touching = true;
                        for (int i = 0; i < nextManifold.pointCount; i++)
                        {
                            ManifoldPoint point = nextManifold.points[i];
                            float normalImpulse = 0f;
                            float tangentImpulse = 0f;

                            for (int j = 0; j < oldManifold.pointCount; j++)
                            {
                                ManifoldPoint oldPoint = oldManifold.points[j];
                                if (oldPoint.id.key == point.id.key)
                                {
                                    normalImpulse = oldPoint.normalImpulse;
                                    tangentImpulse = oldPoint.tangentImpulse;
                                    break;
                                }
                            }

                            nextManifold.points[i] = new ManifoldPoint(
                                point.id,
                                point.localPoint, normalImpulse, tangentImpulse);
                        }
                    }
                }

                futur.Change(nextManifold, 0f, touching);

                if (wasTouching == false && touching == true && contactStartForeseen != null)
                    contactStartForeseen(contact, EventArgs.Empty);

                if (wasTouching == true && touching == false && contactEndForeseen != null)
                    contactEndForeseen(contact, EventArgs.Empty);

                if (sensor == false && contactForPreSolving != null)
                    contactForPreSolving(contact, EventArgs.Empty);
            }
        }



    }
}