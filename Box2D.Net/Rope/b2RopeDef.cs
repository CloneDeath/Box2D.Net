using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct b2RopeDef {
		b2RopeDef(object ignore) {
			vertices = new List<b2Vec2>();
			count = 0;
			masses = new List<float>();
			gravity = new b2Vec2(0, 0);
			damping = 0.1f;
			k2 = 0.9f;
			k3 = 0.1f;
		}

		///
		List<b2Vec2> vertices; //was pointer

		///
		int count;

		///
		List<float> masses;//was pointer

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
