using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Pulley joint definition. This requires two ground anchors,
	/// two dynamic body anchor points, and a pulley ratio.
	class b2PulleyJointDef : b2JointDef
	{
		public b2PulleyJointDef()
		{
			type = b2JointType.e_pulleyJoint;
			groundAnchorA.Set(-1.0f, 1.0f);
			groundAnchorB.Set(1.0f, 1.0f);
			localAnchorA.Set(-1.0f, 0.0f);
			localAnchorB.Set(1.0f, 0.0f);
			lengthA = 0.0f;
			lengthB = 0.0f;
			ratio = 1.0f;
			collideConnected = true;
		}

		/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
		// Pulley:
		// length1 = norm(p1 - s1)
		// length2 = norm(p2 - s2)
		// C0 = (length1 + ratio * length2)_initial
		// C = C0 - (length1 + ratio * length2)
		// u1 = (p1 - s1) / norm(p1 - s1)
		// u2 = (p2 - s2) / norm(p2 - s2)
		// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
		// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
		// K = J * invM * JT
		//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)
		public void Initialize(b2Body bA, b2Body bB,
						b2Vec2 groundA, b2Vec2 groundB,
						b2Vec2 anchorA, b2Vec2 anchorB,
						float ratio){
			bodyA = bA;
			bodyB = bB;
			groundAnchorA = groundA;
			groundAnchorB = groundB;
			localAnchorA = bodyA.GetLocalPoint(anchorA);
			localAnchorB = bodyB.GetLocalPoint(anchorB);
			b2Vec2 dA = anchorA - groundA;
			lengthA = dA.Length();
			b2Vec2 dB = anchorB - groundB;
			lengthB = dB.Length();
			ratio = r;
			Utilities.Assert(ratio > Single.Epsilon);
		}

		/// The first ground anchor in world coordinates. This point never moves.
		public b2Vec2 groundAnchorA;

		/// The second ground anchor in world coordinates. This point never moves.
		public b2Vec2 groundAnchorB;

		/// The local anchor point relative to bodyA's origin.
		public b2Vec2 localAnchorA;

		/// The local anchor point relative to bodyB's origin.
		public b2Vec2 localAnchorB;

		/// The a reference length for the segment attached to bodyA.
		public float lengthA;

		/// The a reference length for the segment attached to bodyB.
		public float lengthB;

		/// The pulley ratio, used to simulate a block-and-tackle.
		public float ratio;
	};
}
