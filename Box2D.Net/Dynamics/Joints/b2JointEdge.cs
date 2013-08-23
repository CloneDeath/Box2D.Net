using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// A joint edge is used to connect bodies and joints together
	/// in a joint graph where each body is a node and each joint
	/// is an edge. A joint edge belongs to a doubly linked list
	/// maintained in each attached body. Each joint has two joint
	/// nodes, one for each attached body.
	public struct b2JointEdge {
		public b2Body other; //pointer			///< provides quick access to the other body attached.
		public b2Joint joint; //pointer			///< the joint
		//public b2JointEdge prev; //pointer		///< the previous joint edge in the body's joint list
		//public b2JointEdge next; //pointer		///< the next joint edge in the body's joint list
	}
}
