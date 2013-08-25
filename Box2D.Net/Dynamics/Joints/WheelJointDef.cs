using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Wheel joint definition. This requires defining a line of
	/// motion using an axis and an anchor point. The definition uses local
	/// anchor points and a local axis so that the initial configuration
	/// can violate the constraint slightly. The joint translation is zero
	/// when the local anchor points coincide in world space. Using local
	/// anchors and a local axis helps when saving and loading a game.
	public class WheelJointDef : JointDef
	{
		public WheelJointDef()
		{
			type = JointType.e_wheelJoint;
			localAnchorA.SetZero();
			localAnchorB.SetZero();
			localAxisA.Set(1.0f, 0.0f);
			enableMotor = false;
			maxMotorTorque = 0.0f;
			motorSpeed = 0.0f;
			frequencyHz = 2.0f;
			dampingRatio = 0.7f;
		}

		/// Initialize the bodies, anchors, axis, and reference angle using the world
		/// anchor and world axis.
		// Linear constraint (point-to-line)
		// d = pB - pA = xB + rB - xA - rA
		// C = dot(ay, d)
		// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
		//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
		// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

		// Spring linear constraint
		// C = dot(ax, d)
		// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
		// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

		// Motor rotational constraint
		// Cdot = wB - wA
		// J = [0 0 -1 0 0 1]
		public void Initialize(Body bA, Body bB, Vec2 anchor, Vec2 axis) {
			bodyA = bA;
			bodyB = bB;
			localAnchorA = bodyA.GetLocalPoint(anchor);
			localAnchorB = bodyB.GetLocalPoint(anchor);
			localAxisA = bodyA.GetLocalVector(axis);
		}

		/// The local anchor point relative to bodyA's origin.
		public Vec2 localAnchorA;

		/// The local anchor point relative to bodyB's origin.
		public Vec2 localAnchorB;

		/// The local translation axis in bodyA.
		public Vec2 localAxisA;

		/// Enable/disable the joint motor.
		public bool enableMotor;

		/// The maximum motor torque, usually in N-m.
		public float maxMotorTorque;

		/// The desired motor speed in radians per second.
		public float motorSpeed;

		/// Suspension frequency, zero indicates no suspension
		public float frequencyHz;

		/// Suspension damping ratio, one indicates critical damping
		public float dampingRatio;
	};
}
