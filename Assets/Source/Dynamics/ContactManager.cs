using System;
using System.Collections;
using System.Collections.Generic;

namespace CrispyPhysics.Internal
{


    public class ContactManager
    {
        private ArrayList pairs;
        private List<IContact> contacts;
    	public int contactCount { get { return contacts.Count; }}

    	private BodyIteratorDelegate bodyIterator;


    	public ContactManager(BodyIteratorDelegate bodyIterator){
    		if  (bodyIterator == null)
                throw new ArgumentOutOfRangeException("BodyIterator should not be null");    		

            contacts = new List<IContact>();
            pairs = new ArrayList(16);
            this.bodyIterator =  bodyIterator;
    	}


    	public void FindNewContacts(){
    		uint i =0;

            pairs.Clear();
    		foreach(Body body in bodyIterator())
    		{
                if (body.shape == null)
                    continue;

                AABB aabb = body.shape.computeAABB(body.futur.transform);
    			foreach(Body pairBody in bodyIterator(i + 1)){
                    if (pairBody.shape == null)
                        continue;

                    AABB pairAabb = pairBody.shape.computeAABB(pairBody.futur.transform);
                    if(Collision.TestOverlap(aabb, pairAabb))
                        AddPair(body, pairBody);
                }
    			i++;
    		}
    	}

        public void AddPair(Body bodyA, Body bodyB)
        {

        }

    	public void Destroy(IContact contact){

    	}

    	public void Collide(){

    	}

    }
}