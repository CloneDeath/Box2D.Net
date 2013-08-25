using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Box2D;

namespace Testbed.Framework {
	struct ContactPoint {
		public Fixture fixtureA;
		public Fixture fixtureB;
		public Vec2 normal;
		public Vec2 position;
		public PointState state;
		public float normalImpulse;
		public float tangentImpulse;
	}
}
