using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A contact edge is used to connect bodies and contacts together
	/// in a contact graph where each body is a node and each contact
	/// is an edge. A contact edge belongs to a doubly linked list
	/// maintained in each attached body. Each contact has two contact
	/// nodes, one for each attached body.
	struct b2ContactEdge {
		b2Body other; //pointer			///< provides quick access to the other body attached.
		b2Contact contact;//pointer		///< the contact
		b2ContactEdge prev; //pointer	///< the previous contact edge in the body's contact list
		b2ContactEdge next;//pointer	///< the next contact edge in the body's contact list
	};
}
