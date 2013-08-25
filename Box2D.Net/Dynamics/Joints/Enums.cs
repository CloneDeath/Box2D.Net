using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	public enum JointType {
		e_unknownJoint,
		e_revoluteJoint,
		e_prismaticJoint,
		e_distanceJoint,
		e_pulleyJoint,
		e_mouseJoint,
		e_gearJoint,
		e_wheelJoint,
		e_weldJoint,
		e_frictionJoint,
		e_ropeJoint,
		e_motorJoint
	};

	public enum LimitState {
		e_inactiveLimit,
		e_atLowerLimit,
		e_atUpperLimit,
		e_equalLimits
	};
}
