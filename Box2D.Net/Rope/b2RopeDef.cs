using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	public class b2RopeDef { //was struct
		public b2RopeDef() {
			vertices = new List<b2Vec2>();
			count = 0;
			masses = new List<float>();
			gravity = new b2Vec2(0, 0);
			damping = 0.1f;
			k2 = 0.9f;
			k3 = 0.1f;
		}

		///
		public List<b2Vec2> vertices; //was pointer

		///
		public int count;

		///
		public List<float> masses;//was pointer

		///
		public b2Vec2 gravity;

		///
		public float damping;

		/// Stretching stiffness
		public float k2;

		/// Bending stiffness. Values above 0.5 can make the simulation blow up.
		public float k3;
	}
}
