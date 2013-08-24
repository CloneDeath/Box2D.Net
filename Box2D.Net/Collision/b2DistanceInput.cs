using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Input for b2Distance.
	/// You have to option to use the shape radii
	/// in the computation. Even 
	struct b2DistanceInput {
		public b2DistanceProxy proxyA;
		public b2DistanceProxy proxyB;
		public b2Transform transformA;
		public b2Transform transformB;
		public bool useRadii;
	}
}
