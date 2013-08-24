using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	class b2ContactPositionConstraint
	{
		public b2Vec2[] localPoints = new b2Vec2[b2Settings.b2_maxManifoldPoints];
		public b2Vec2 localNormal;
		public b2Vec2 localPoint;
		public int indexA;
		public int indexB;
		public float invMassA, invMassB;
		public b2Vec2 localCenterA, localCenterB;
		public float invIA, invIB;
		public b2Manifold.ManifoldType type;
		public float radiusA, radiusB;
		public int pointCount;
	};
}
