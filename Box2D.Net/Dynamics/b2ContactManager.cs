using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	// Delegate of b2World.
	public class b2ContactManager
	{
		static b2ContactFilter b2_defaultFilter;
		static b2ContactListener b2_defaultListener;

		public b2BroadPhase m_broadPhase;
		public List<b2Contact> m_contactList;
		public b2ContactFilter m_contactFilter;
		public b2ContactListener m_contactListener;

		public b2ContactManager() {
			m_contactList = new List<b2Contact>();
			m_contactFilter = b2_defaultFilter;
			m_contactListener = b2_defaultListener;
			m_broadPhase = new b2BroadPhase();
		}

		// Broad-phase callback.
		public void AddPair(object proxyUserDataA, object proxyUserDataB) {
			b2FixtureProxy proxyA = (b2FixtureProxy)proxyUserDataA;
			b2FixtureProxy proxyB = (b2FixtureProxy)proxyUserDataB;

			b2Fixture fixtureA = proxyA.fixture;
			b2Fixture fixtureB = proxyB.fixture;

			int indexA = proxyA.childIndex;
			int indexB = proxyB.childIndex;

			b2Body bodyA = fixtureA.GetBody();
			b2Body bodyB = fixtureB.GetBody();

			// Are the fixtures on the same body?
			if (bodyA == bodyB)
			{
				return;
			}

			// TODO_ERIN use a hash table to remove a potential bottleneck when both
			// bodies have a lot of contacts.
			// Does a contact already exist?
			List<b2ContactEdge> edges = bodyB.GetContactList();
			foreach (b2ContactEdge edge in edges) {
				if (edge.other == bodyA)
				{
					b2Fixture fA = edge.contact.GetFixtureA();
					b2Fixture fB = edge.contact.GetFixtureB();
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
			b2Contact c = b2Contact.Create(fixtureA, indexA, fixtureB, indexB);
			if (c == null)
			{
				return;
			}

			// Contact creation may swap fixtures.
			fixtureA = c.GetFixtureA();
			fixtureB = c.GetFixtureB();
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
			if (fixtureA.IsSensor() == false && fixtureB.IsSensor() == false)
			{
				bodyA.SetAwake(true);
				bodyB.SetAwake(true);
			}
		}

		public void FindNewContacts() {
			m_broadPhase.UpdatePairs(this);
		}

		public void Destroy(b2Contact c) {
			throw new NotImplementedException();
			//b2Fixture* fixtureA = c.GetFixtureA();
			//b2Fixture* fixtureB = c.GetFixtureB();
			//b2Body* bodyA = fixtureA.GetBody();
			//b2Body* bodyB = fixtureB.GetBody();

			//if (m_contactListener && c.IsTouching())
			//{
			//    m_contactListener.EndContact(c);
			//}

			//// Remove from the world.
			//if (c.m_prev)
			//{
			//    c.m_prev.m_next = c.m_next;
			//}

			//if (c.m_next)
			//{
			//    c.m_next.m_prev = c.m_prev;
			//}

			//if (c == m_contactList)
			//{
			//    m_contactList = c.m_next;
			//}

			//// Remove from body 1
			//if (c.m_nodeA.prev)
			//{
			//    c.m_nodeA.prev.next = c.m_nodeA.next;
			//}

			//if (c.m_nodeA.next)
			//{
			//    c.m_nodeA.next.prev = c.m_nodeA.prev;
			//}

			//if (&c.m_nodeA == bodyA.m_contactList)
			//{
			//    bodyA.m_contactList = c.m_nodeA.next;
			//}

			//// Remove from body 2
			//if (c.m_nodeB.prev)
			//{
			//    c.m_nodeB.prev.next = c.m_nodeB.next;
			//}

			//if (c.m_nodeB.next)
			//{
			//    c.m_nodeB.next.prev = c.m_nodeB.prev;
			//}

			//if (&c.m_nodeB == bodyB.m_contactList)
			//{
			//    bodyB.m_contactList = c.m_nodeB.next;
			//}

			//// Call the factory.
			//b2Contact::Destroy(c, m_allocator);
			//--m_contactCount;
		}

		public void Collide() {
			// Update awake contacts.
			foreach (b2Contact c in m_contactList){
			    b2Fixture fixtureA = c.GetFixtureA();
			    b2Fixture fixtureB = c.GetFixtureB();
			    int indexA = c.GetChildIndexA();
			    int indexB = c.GetChildIndexB();
			    b2Body bodyA = fixtureA.GetBody();
			    b2Body bodyB = fixtureB.GetBody();
		 
			    // Is this contact flagged for filtering?
			    if (c.m_flags.HasFlag(ContactFlags.e_filterFlag))
			    {
					throw new NotImplementedException();
					//// Should these bodies collide?
					//if (bodyB.ShouldCollide(bodyA) == false)
					//{
					//    b2Contact cNuke = c;
					//    c = cNuke.GetNext();
					//    Destroy(cNuke);
					//    continue;
					//}

					//// Check user filtering.
					//if (m_contactFilter && m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false)
					//{
					//    b2Contact* cNuke = c;
					//    c = cNuke.GetNext();
					//    Destroy(cNuke);
					//    continue;
					//}

					//// Clear the filtering flag.
					//c.m_flags &= ~ContactFlags.e_filterFlag;
			    }

				throw new NotImplementedException();
				//bool activeA = bodyA.IsAwake() && bodyA.m_type != b2_staticBody;
				//bool activeB = bodyB.IsAwake() && bodyB.m_type != b2_staticBody;

				//// At least one body must be awake and it must be dynamic or kinematic.
				//if (activeA == false && activeB == false)
				//{
				//    continue;
				//}

				//int proxyIdA = fixtureA.m_proxies[indexA].proxyId;
				//int proxyIdB = fixtureB.m_proxies[indexB].proxyId;
				//bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

				//// Here we destroy contacts that cease to overlap in the broad-phase.
				//if (overlap == false)
				//{
				//    b2Contact* cNuke = c;
				//    c = cNuke.GetNext();
				//    Destroy(cNuke);
				//    continue;
				//}

				//// The contact persists.
				//c.Update(m_contactListener);
				//c = c.GetNext();
			}
		}

		
	};
}
