using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct SimplexVertex {
		public Vec2 wA;		// support point in proxyA
		public Vec2 wB;		// support point in proxyB
		public Vec2 w;		// wB - wA
		public float a;		// barycentric coordinate for closest point
		public int indexA;	// wA index
		public int indexB;	// wB index
	}
}
