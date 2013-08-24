using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct b2ContactVelocityConstraint
	{
		public List<b2VelocityConstraintPoint> points;
		public b2Vec2 normal;
		public b2Mat22 normalMass;
		public b2Mat22 K;
		public int indexA;
		public int indexB;
		public float invMassA, invMassB;
		public float invIA, invIB;
		public float friction;
		public float restitution;
		public float tangentSpeed;
		public int contactIndex;
	};
}
