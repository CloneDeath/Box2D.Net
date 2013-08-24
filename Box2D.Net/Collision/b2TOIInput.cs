using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Input parameters for b2TimeOfImpact
	struct b2TOIInput {
		public b2DistanceProxy proxyA;
		public b2DistanceProxy proxyB;
		public b2Sweep sweepA;
		public b2Sweep sweepB;
		public float tMax;		// defines sweep interval [0, tMax]
	};
}
