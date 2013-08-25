using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class ContactVelocityConstraint // was struct
	{
		public List<VelocityConstraintPoint> points = new List<VelocityConstraintPoint>();
		public Vec2 normal;
		public Mat22 normalMass;
		public Mat22 K;
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
