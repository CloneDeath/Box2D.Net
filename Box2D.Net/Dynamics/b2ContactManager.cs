using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	// Delegate of b2World.
	public class b2ContactManager
	{
		public b2ContactManager() {
			throw new NotImplementedException();
		}

		// Broad-phase callback.
		public void AddPair(object proxyUserDataA, object proxyUserDataB) {
			throw new NotImplementedException();
		}

		public void FindNewContacts() {
			throw new NotImplementedException();
		}

		public void Destroy(b2Contact c) {
			throw new NotImplementedException();
		}

		public void Collide() {
			throw new NotImplementedException();
		}

		public b2BroadPhase m_broadPhase;
		public List<b2Contact> m_contactList;
		public int m_contactCount;
		public b2ContactFilter m_contactFilter;
		public b2ContactListener m_contactListener;
	};
}
