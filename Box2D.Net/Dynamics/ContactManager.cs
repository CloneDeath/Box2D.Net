using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	// Delegate of World.
	public class ContactManager
	{
		static ContactFilter _defaultFilter;
		static ContactListener _defaultListener;

		public BroadPhase m_broadPhase;
		public List<Contact> m_contactList;
		public ContactFilter m_contactFilter;
		public ContactListener m_contactListener;

		public ContactManager() {
			m_contactList = new List<Contact>();
			m_contactFilter = _defaultFilter;
			m_contactListener = _defaultListener;
			m_broadPhase = new BroadPhase();
		}

		// Broad-phase callback.
		public void AddPair(object proxyUserDataA, object proxyUserDataB) {
			FixtureProxy proxyA = (FixtureProxy)proxyUserDataA;
			FixtureProxy proxyB = (FixtureProxy)proxyUserDataB;

			Fixture fixtureA = proxyA.fixture;
			Fixture fixtureB = proxyB.fixture;

			int indexA = proxyA.childIndex;
			int indexB = proxyB.childIndex;

			Body bodyA = fixtureA.GetBody();
			Body bodyB = fixtureB.GetBody();

			// Are the fixtures on the same body?
			if (bodyA == bodyB)
			{
				return;
			}

			// TODO_ERIN use a hash table to remove a potential bottleneck when both
			// bodies have a lot of contacts.
			// Does a contact already exist?
			List<ContactEdge> edges = bodyB.GetContactList();
			foreach (ContactEdge edge in edges) {
				if (edge.other == bodyA)
				{
					Fixture fA = edge.contact.FixtureA;
					Fixture fB = edge.contact.FixtureB;
					int iA = edge.contact.GetChildIndexA();
					int iB = edge.contact.GetChildIndexB();

					if (fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB)
					{
						// A contact already exists.
						return;
					}

					if (fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA)
					{
						// A contact already exists.
						return;
					}
				}
			}

			// Does a joint override collision? Is at least one body dynamic?
			if (bodyB.ShouldCollide(bodyA) == false)
			{
				return;
			}

			// Check user filtering.
			if ((m_contactFilter != null) && m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
			{
				return;
			}

			// Call the factory.
			Contact c = Contact.Create(fixtureA, indexA, fixtureB, indexB);
			if (c == null)
			{
				return;
			}

			// Contact creation may swap fixtures.
			fixtureA = c.FixtureA;
			fixtureB = c.FixtureB;
			indexA = c.GetChildIndexA();
			indexB = c.GetChildIndexB();
			bodyA = fixtureA.GetBody();
			bodyB = fixtureB.GetBody();

			// Insert into the world.
			m_contactList.Add(c);

			// Connect to island graph.

			// Connect to body A
			c.m_nodeA.contact = c;
			c.m_nodeA.other = bodyB;
			bodyA.m_contactList.Add(c.m_nodeA);

			// Connect to body B
			c.m_nodeB.contact = c;
			c.m_nodeB.other = bodyA;

			bodyB.m_contactList.Add(c.m_nodeB);

			// Wake up the bodies
			if (fixtureA.IsSensor == false && fixtureB.IsSensor == false)
			{
				bodyA.SetAwake(true);
				bodyB.SetAwake(true);
			}
		}

		public void FindNewContacts() {
			m_broadPhase.UpdatePairs(this);
		}

		public void Destroy(Contact c) {
			Fixture fixtureA = c.FixtureA;
			Fixture fixtureB = c.FixtureB;
			Body bodyA = fixtureA.GetBody();
			Body bodyB = fixtureB.GetBody();

			if (m_contactListener != null && c.IsTouching())
			{
			    m_contactListener.EndContact(c);
			}

			// Remove from the world.
			m_contactList.Remove(c);

			// Remove from body 1
			c.m_nodeA.other.m_contactList.Remove(c.m_nodeA);

			// Remove from body 2
			c.m_nodeB.other.m_contactList.Remove(c.m_nodeA);

			// Call the factory.
		}

		public void Collide() {
			// Update awake contacts.
			for (int i = 0; i < m_contactList.Count(); i++){
				Contact c = m_contactList[i];
			    Fixture fixtureA = c.FixtureA;
			    Fixture fixtureB = c.FixtureB;
			    int indexA = c.GetChildIndexA();
			    int indexB = c.GetChildIndexB();
			    Body bodyA = fixtureA.GetBody();
			    Body bodyB = fixtureB.GetBody();
		 
			    // Is this contact flagged for filtering?
			    if (c.m_flags.HasFlag(ContactFlags.e_filterFlag))
			    {
					throw new NotImplementedException();
					//// Should these bodies collide?
					//if (bodyB.ShouldCollide(bodyA) == false)
					//{
					//    Contact cNuke = c;
					//    c = cNuke.GetNext();
					//    Destroy(cNuke);
					//    continue;
					//}

					//// Check user filtering.
					//if (m_contactFilter && m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
					//{
					//    Contact* cNuke = c;
					//    c = cNuke.GetNext();
					//    Destroy(cNuke);
					//    continue;
					//}

					//// Clear the filtering flag.
					//c.m_flags &= ~ContactFlags.e_filterFlag;
			    }

				bool activeA = bodyA.IsAwake() && bodyA.m_type != BodyType._staticBody;
				bool activeB = bodyB.IsAwake() && bodyB.m_type != BodyType._staticBody;

				// At least one body must be awake and it must be dynamic or kinematic.
				if (activeA == false && activeB == false) {
					continue;
				}

				int proxyIdA = fixtureA.m_proxies[indexA].proxyId;
				int proxyIdB = fixtureB.m_proxies[indexB].proxyId;
				bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

				// Here we destroy contacts that cease to overlap in the broad-phase.
				if (overlap == false) {
					Destroy(c);
					continue;
				}

				// The contact persists.
				c.Update(m_contactListener);
			}
		}

		
	}
}
