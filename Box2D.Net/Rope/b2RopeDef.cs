using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	public class RopeDef { //was struct
		public RopeDef() {
			vertices = new List<Vec2>();
			count = 0;
			masses = new List<float>();
			gravity = new Vec2(0, 0);
			damping = 0.1f;
			k2 = 0.9f;
			k3 = 0.1f;
		}

		///
		public List<Vec2> vertices; //was pointer

		///
		public int count;

		///
		public List<float> masses;//was pointer

		///
		public Vec2 gravity;

		///
		public float damping;

		/// Stretching stiffness
		public float k2;

		/// Bending stiffness. Values above 0.5 can make the simulation blow up.
		public float k3;
	}
}
