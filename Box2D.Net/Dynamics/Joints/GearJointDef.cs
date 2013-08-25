using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Gear joint definition. This definition requires two existing
	/// revolute or prismatic joints (any combination will work).
	public class GearJointDef : JointDef
	{
		public GearJointDef()
		{
			type = JointType.e_gearJoint;
			joint1 = null;
			joint2 = null;
			ratio = 1.0f;
		}

		/// The first revolute/prismatic joint attached to the gear joint.
		public Joint joint1;

		/// The second revolute/prismatic joint attached to the gear joint.
		public Joint joint2;

		/// The gear ratio.
		/// @see GearJoint for explanation.
		public float ratio;
	};
}
