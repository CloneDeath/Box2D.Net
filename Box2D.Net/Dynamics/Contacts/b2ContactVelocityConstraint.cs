using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class b2ContactVelocityConstraint // was struct
	{
		public List<b2VelocityConstraintPoint> points = new List<b2VelocityConstraintPoint>();
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
