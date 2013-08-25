using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Input for Distance.
	/// You have to option to use the shape radii
	/// in the computation. Even 
	public struct DistanceInput {
		public DistanceProxy proxyA;
		public DistanceProxy proxyB;
		public Transform transformA;
		public Transform transformB;
		public bool useRadii;
	}
}
