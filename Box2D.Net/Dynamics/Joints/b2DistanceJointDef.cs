using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Distance joint definition. This requires defining an
	/// anchor point on both bodies and the non-zero length of the
	/// distance joint. The definition uses local anchor points
	/// so that the initial configuration can violate the constraint
	/// slightly. This helps when saving and loading a game.
	/// @warning Do not use a zero or short length.
	public class DistanceJointDef : JointDef
	{
		public DistanceJointDef()
		{
			type = JointType.e_distanceJoint;
			localAnchorA.Set(0.0f, 0.0f);
			localAnchorB.Set(0.0f, 0.0f);
			length = 1.0f;
			frequencyHz = 0.0f;
			dampingRatio = 0.0f;
		}

		/// Initialize the bodies, anchors, and length using the world
		/// anchors.
		// 1-D constrained system
		// m (v2 - v1) = lambda
		// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
		// x2 = x1 + h * v2

		// 1-D mass-damper-spring system
		// m (v2 - v1) + h * d * v2 + h * k * 

		// C = norm(p2 - p1) - L
		// u = (p2 - p1) / norm(p2 - p1)
		// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
		// J = [-u -cross(r1, u) u cross(r2, u)]
		// K = J * invM * JT
		//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2
		public void Initialize(Body b1, Body b2,
						Vec2 anchor1, Vec2 anchor2) {
			bodyA = b1;
			bodyB = b2;
			localAnchorA = bodyA.GetLocalPoint(anchor1);
			localAnchorB = bodyB.GetLocalPoint(anchor2);
			Vec2 d = anchor2 - anchor1;
			length = d.Length();
		}

		/// The local anchor point relative to bodyA's origin.
		public Vec2 localAnchorA;

		/// The local anchor point relative to bodyB's origin.
		public Vec2 localAnchorB;

		/// The natural length between the anchor points.
		public float length;

		/// The mass-spring-damper frequency in Hertz. A value of 0
		/// disables softness.
		public float frequencyHz;

		/// The damping ratio. 0 = no damping, 1 = critical damping.
		public float dampingRatio;
	};
}
