using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Testbed.Framework {
	struct ContactPoint {
		public b2Fixture fixtureA;
		public b2Fixture fixtureB;
		public b2Vec2 normal;
		public b2Vec2 position;
		public b2PointState state;
		public float normalImpulse;
		public float tangentImpulse;
	}
}
