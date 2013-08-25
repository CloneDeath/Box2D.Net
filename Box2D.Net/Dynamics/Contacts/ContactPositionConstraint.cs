using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class ContactPositionConstraint
	{
		public Vec2[] localPoints = new Vec2[Settings._maxManifoldPoints];
		public Vec2 localNormal;
		public Vec2 localPoint;
		public int indexA;
		public int indexB;
		public float invMassA, invMassB;
		public Vec2 localCenterA, localCenterB;
		public float invIA, invIB;
		public Manifold.ManifoldType type;
		public float radiusA, radiusB;
		public int pointCount;
	};
}
