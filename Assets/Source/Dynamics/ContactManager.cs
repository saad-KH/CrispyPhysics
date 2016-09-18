using System;
using System.Collections.Generic;

namespace CrispyPhysics.Internal
{
    public class ContactManager
    {
    	private List<IContact> contacts;
    	public int contactCount { get { return contacts.Count; }}

    	private BodyIteratorDelegate bodyIterator;


    	public ContactManager(BodyIteratorDelegate bodyIterator){
    		if  (bodyIterator == null)
                throw new ArgumentOutOfRangeException("BodyIterator should not be null");    		

            contacts = new List<IContact>();
            this.bodyIterator =  bodyIterator;
    	}


    	public void FindNewContacts(){
    		uint i =0;
    		foreach(IInternalBody body in bodyIterator())
    		{
    			foreach(IInternalBody pairBody in bodyIterator(i + 1)){
    				
    			}
    			i++;
    		}
    	}

    	public void AddPair(){

    	}

    	public void Destroy(IContact contact){

    	}

    	public void Collide(){

    	}

    }
}