﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Output for Distance.
	public struct DistanceOutput {
		public Vec2 pointA;		///< closest point on shapeA
		public Vec2 pointB;		///< closest point on shapeB
		public float distance;
		public int iterations;	///< number of GJK iterations used
	}
}
