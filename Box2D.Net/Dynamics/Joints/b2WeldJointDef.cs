using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Weld joint definition. You need to specify local anchor points
	/// where they are attached and the relative body angle. The position
	/// of the anchor points is important for computing the reaction torque.
	public class b2WeldJointDef : b2JointDef
	{
		public b2WeldJointDef()
		{
			type = b2JointType.e_weldJoint;
			localAnchorA.Set(0.0f, 0.0f);
			localAnchorB.Set(0.0f, 0.0f);
			referenceAngle = 0.0f;
			frequencyHz = 0.0f;
			dampingRatio = 0.0f;
		}

		/// Initialize the bodies, anchors, and reference angle using a world
		/// anchor point.
		// Point-to-point constraint
		// C = p2 - p1
		// Cdot = v2 - v1
		//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
		// J = [-I -r1_skew I r2_skew ]
		// Identity used:
		// w k % (rx i + ry j) = w * (-ry i + rx j)

		// Angle constraint
		// C = angle2 - angle1 - referenceAngle
		// Cdot = w2 - w1
		// J = [0 0 -1 0 0 1]
		// K = invI1 + invI2
		public void Initialize(b2Body bA, b2Body bB, b2Vec2 anchor) {
			bodyA = bA;
			bodyB = bB;
			localAnchorA = bodyA.GetLocalPoint(anchor);
			localAnchorB = bodyB.GetLocalPoint(anchor);
			referenceAngle = bodyB.GetAngle() - bodyA.GetAngle();
		}

		/// The local anchor point relative to bodyA's origin.
		public b2Vec2 localAnchorA;

		/// The local anchor point relative to bodyB's origin.
		public b2Vec2 localAnchorB;

		/// The bodyB angle minus bodyA angle in the reference state (radians).
		public float referenceAngle;
	
		/// The mass-spring-damper frequency in Hertz. Rotation only.
		/// Disable softness with a value of 0.
		public float frequencyHz;

		/// The damping ratio. 0 = no damping, 1 = critical damping.
		public float dampingRatio;
	};
}
