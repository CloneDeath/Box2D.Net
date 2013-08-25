using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Motor joint definition.
	public class MotorJointDef : JointDef
	{
		public MotorJointDef()
		{
			type = JointType.e_motorJoint;
			linearOffset.SetZero();
			angularOffset = 0.0f;
			maxForce = 1.0f;
			maxTorque = 1.0f;
			correctionFactor = 0.3f;
		}

		/// Initialize the bodies and offsets using the current transforms.
		// Point-to-point constraint
		// Cdot = v2 - v1
		//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
		// J = [-I -r1_skew I r2_skew ]
		// Identity used:
		// w k % (rx i + ry j) = w * (-ry i + rx j)

		// Angle constraint
		// Cdot = w2 - w1
		// J = [0 0 -1 0 0 1]
		// K = invI1 + invI2
		public void Initialize(Body bA, Body bB) {
			bodyA = bA;
			bodyB = bB;
			Vec2 xB = bodyB.GetPosition();
			linearOffset = bodyA.GetLocalPoint(xB);

			float angleA = bodyA.GetAngle();
			float angleB = bodyB.GetAngle();
			angularOffset = angleB - angleA;
		}


		/// Position of bodyB minus the position of bodyA, in bodyA's frame, in meters.
		public Vec2 linearOffset;

		/// The bodyB angle minus bodyA angle in radians.
		public float angularOffset;
	
		/// The maximum motor force in N.
		public float maxForce;

		/// The maximum motor torque in N-m.
		public float maxTorque;

		/// Position correction factor in the range [0,1].
		public float correctionFactor;
	};
}
