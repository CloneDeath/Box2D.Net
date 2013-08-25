using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Revolute joint definition. This requires defining an
	/// anchor point where the bodies are joined. The definition
	/// uses local anchor points so that the initial configuration
	/// can violate the constraint slightly. You also need to
	/// specify the initial relative angle for joint limits. This
	/// helps when saving and loading a game.
	/// The local anchor points are measured from the body's origin
	/// rather than the center of mass because:
	/// 1. you might not know where the center of mass will be.
	/// 2. if you add/remove shapes from a body and recompute the mass,
	///    the joints will be broken.
	class b2RevoluteJointDef : b2JointDef
	{
		public b2RevoluteJointDef()
		{
			type = b2JointType.e_revoluteJoint;
			localAnchorA.Set(0.0f, 0.0f);
			localAnchorB.Set(0.0f, 0.0f);
			referenceAngle = 0.0f;
			lowerAngle = 0.0f;
			upperAngle = 0.0f;
			maxMotorTorque = 0.0f;
			motorSpeed = 0.0f;
			enableLimit = false;
			enableMotor = false;
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

		// Motor constraint
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

		/// A flag to enable joint limits.
		public bool enableLimit;

		/// The lower angle for the joint limit (radians).
		public float lowerAngle;

		/// The upper angle for the joint limit (radians).
		public float upperAngle;

		/// A flag to enable the joint motor.
		public bool enableMotor;

		/// The desired motor speed. Usually in radians per second.
		public float motorSpeed;

		/// The maximum motor torque used to achieve the desired motor speed.
		/// Usually in N-m.
		public float maxMotorTorque;
	};
}
