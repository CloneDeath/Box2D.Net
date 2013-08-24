using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Input parameters for b2TimeOfImpact
	struct b2TOIInput {
		b2DistanceProxy proxyA;
		b2DistanceProxy proxyB;
		b2Sweep sweepA;
		b2Sweep sweepB;
		float tMax;		// defines sweep interval [0, tMax]
	};
}
