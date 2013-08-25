using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Output for b2Distance.
	public struct b2DistanceOutput {
		public b2Vec2 pointA;		///< closest point on shapeA
		public b2Vec2 pointB;		///< closest point on shapeB
		public float distance;
		public int iterations;	///< number of GJK iterations used
	}
}
