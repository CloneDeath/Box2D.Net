using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Friction joint definition.
	public class FrictionJointDef : JointDef
	{
		public FrictionJointDef()
		{
			type = JointType.e_frictionJoint;
			localAnchorA.SetZero();
			localAnchorB.SetZero();
			maxForce = 0.0f;
			maxTorque = 0.0f;
		}

		/// Initialize the bodies, anchors, axis, and reference angle using the world
		/// anchor and world axis.
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
		public void Initialize(Body bA, Body bB, Vec2 anchor) {
			bodyA = bA;
			bodyB = bB;
			localAnchorA = bodyA.GetLocalPoint(anchor);
			localAnchorB = bodyB.GetLocalPoint(anchor);
		}

		/// The local anchor point relative to bodyA's origin.
		public Vec2 localAnchorA;

		/// The local anchor point relative to bodyB's origin.
		public Vec2 localAnchorB;

		/// The maximum friction force in N.
		public float maxForce;

		/// The maximum friction torque in N-m.
		public float maxTorque;
	}
}
