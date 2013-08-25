using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Prismatic joint definition. This requires defining a line of
	/// motion using an axis and an anchor point. The definition uses local
	/// anchor points and a local axis so that the initial configuration
	/// can violate the constraint slightly. The joint translation is zero
	/// when the local anchor points coincide in world space. Using local
	/// anchors and a local axis helps when saving and loading a game.
	public class PrismaticJointDef : JointDef
	{
		public PrismaticJointDef()
		{
			type = JointType.e_prismaticJoint;
			localAnchorA.SetZero();
			localAnchorB.SetZero();
			localAxisA.Set(1.0f, 0.0f);
			referenceAngle = 0.0f;
			enableLimit = false;
			lowerTranslation = 0.0f;
			upperTranslation = 0.0f;
			enableMotor = false;
			maxMotorForce = 0.0f;
			motorSpeed = 0.0f;
		}

		/// Initialize the bodies, anchors, axis, and reference angle using the world
		/// anchor and unit world axis.
		// Linear constraint (point-to-line)
		// d = p2 - p1 = x2 + r2 - x1 - r1
		// C = dot(perp, d)
		// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
		//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
		// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
		//
		// Angular constraint
		// C = a2 - a1 + a_initial
		// Cdot = w2 - w1
		// J = [0 0 -1 0 0 1]
		//
		// K = J * invM * JT
		//
		// J = [-a -s1 a s2]
		//     [0  -1  0  1]
		// a = perp
		// s1 = cross(d + r1, a) = cross(p2 - x1, a)
		// s2 = cross(r2, a) = cross(p2 - x2, a)


		// Motor/Limit linear constraint
		// C = dot(ax1, d)
		// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
		// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

		// Block Solver
		// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
		// when the mass has poor distribution (leading to large torques about the joint anchor points).
		//
		// The Jacobian has 3 rows:
		// J = [-uT -s1 uT s2] // linear
		//     [0   -1   0  1] // angular
		//     [-vT -a1 vT a2] // limit
		//
		// u = perp
		// v = axis
		// s1 = cross(d + r1, u), s2 = cross(r2, u)
		// a1 = cross(d + r1, v), a2 = cross(r2, v)

		// M * (v2 - v1) = JT * df
		// J * v2 = bias
		//
		// v2 = v1 + invM * JT * df
		// J * (v1 + invM * JT * df) = bias
		// K * df = bias - J * v1 = -Cdot
		// K = J * invM * JT
		// Cdot = J * v1 - bias
		//
		// Now solve for f2.
		// df = f2 - f1
		// K * (f2 - f1) = -Cdot
		// f2 = invK * (-Cdot) + f1
		//
		// Clamp accumulated limit impulse.
		// lower: f2(3) = max(f2(3), 0)
		// upper: f2(3) = min(f2(3), 0)
		//
		// Solve for correct f2(1:2)
		// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
		//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
		// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
		// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
		//
		// Now compute impulse to be applied:
		// df = f2 - f1
		public void Initialize(Body bA, Body bB, Vec2 anchor, Vec2 axis){
			bodyA = bA;
			bodyB = bB;
			localAnchorA = bodyA.GetLocalPoint(anchor);
			localAnchorB = bodyB.GetLocalPoint(anchor);
			localAxisA = bodyA.GetLocalVector(axis);
			referenceAngle = bodyB.GetAngle() - bodyA.GetAngle();
		}

		/// The local anchor point relative to bodyA's origin.
		public Vec2 localAnchorA;

		/// The local anchor point relative to bodyB's origin.
		public Vec2 localAnchorB;

		/// The local translation unit axis in bodyA.
		public Vec2 localAxisA;

		/// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
		public float referenceAngle;

		/// Enable/disable the joint limit.
		public bool enableLimit;

		/// The lower translation limit, usually in meters.
		public float lowerTranslation;

		/// The upper translation limit, usually in meters.
		public float upperTranslation;

		/// Enable/disable the joint motor.
		public bool enableMotor;

		/// The maximum motor torque, usually in N-m.
		public float maxMotorForce;

		/// The desired motor speed in radians per second.
		public float motorSpeed;
	};
}
