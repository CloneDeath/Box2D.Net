using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Input parameters for b2TimeOfImpact
	class b2TOIInput { //was struct
		public b2DistanceProxy proxyA = new b2DistanceProxy();
		public b2DistanceProxy proxyB = new b2DistanceProxy();
		public b2Sweep sweepA = new b2Sweep();
		public b2Sweep sweepB = new b2Sweep();
		public float tMax;		// defines sweep interval [0, tMax]
	};
}
