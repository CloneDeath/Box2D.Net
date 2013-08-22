using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct b2RopeDef {
		b2RopeDef() {
			vertices = null;
			count = 0;
			masses = null;
			gravity.SetZero();
			damping = 0.1f;
			k2 = 0.9f;
			k3 = 0.1f;
		}

		///
		b2Vec2* vertices;

		///
		int count;

		///
		float* masses;

		///
		b2Vec2 gravity;

		///
		float damping;

		/// Stretching stiffness
		float k2;

		/// Bending stiffness. Values above 0.5 can make the simulation blow up.
		float k3;
	}
}
