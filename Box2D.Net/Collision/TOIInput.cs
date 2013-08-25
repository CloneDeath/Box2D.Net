using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Input parameters for TimeOfImpact
	public class TOIInput { //was struct
		public DistanceProxy proxyA = new DistanceProxy();
		public DistanceProxy proxyB = new DistanceProxy();
		public Sweep sweepA = new Sweep();
		public Sweep sweepB = new Sweep();
		public float tMax;		// defines sweep interval [0, tMax]
	};
}
