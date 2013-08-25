using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct Jacobian {
		public Vec2 linear;
		public float angularA;
		public float angularB;
	}
}
