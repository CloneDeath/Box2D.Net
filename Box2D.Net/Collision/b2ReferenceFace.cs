using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	// Reference face used for clipping
	struct b2ReferenceFace {
		public int i1, i2;

		public b2Vec2 v1, v2;

		public b2Vec2 normal;

		public b2Vec2 sideNormal1;
		public float sideOffset1;

		public b2Vec2 sideNormal2;
		public float sideOffset2;
	};
}
