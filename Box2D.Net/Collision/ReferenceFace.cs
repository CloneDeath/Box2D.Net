using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	// Reference face used for clipping
	struct ReferenceFace {
		public int i1, i2;

		public Vec2 v1, v2;

		public Vec2 normal;

		public Vec2 sideNormal1;
		public float sideOffset1;

		public Vec2 sideNormal2;
		public float sideOffset2;
	};
}
