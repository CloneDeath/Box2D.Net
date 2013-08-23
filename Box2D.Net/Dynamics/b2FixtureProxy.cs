using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// This proxy is used internally to connect fixtures to the broad-phase.
	struct b2FixtureProxy {
		public b2AABB aabb;
		public b2Fixture fixture; //was pointer
		public int childIndex;
		public int proxyId;
	}
}
