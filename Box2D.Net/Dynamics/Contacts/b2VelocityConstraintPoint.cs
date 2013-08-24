using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct b2VelocityConstraintPoint {
		public b2Vec2 rA;
		public b2Vec2 rB;
		public float normalImpulse;
		public float tangentImpulse;
		public float normalMass;
		public float tangentMass;
		public float velocityBias;
	}
}
