using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Joint definitions are used to construct joints.
	public struct b2JointDef {
		public b2JointDef() {
			type = b2JointType.e_unknownJoint;
			userData = null;
			bodyA = null;
			bodyB = null;
			collideConnected = false;
		}

		/// The joint type is set automatically for concrete joint types.
		public b2JointType type;

		/// Use this to attach application specific data to your joints.
		public object userData;

		/// The first attached body.
		public b2Body bodyA;

		/// The second attached body.
		public b2Body bodyB;

		/// Set this flag to true if the attached bodies should collide.
		public bool collideConnected;
	}
}
