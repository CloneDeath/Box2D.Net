using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// This holds the mass data computed for a shape.
	public struct MassData {
		/// The mass of the shape, usually in kilograms.
		public float mass;

		/// The position of the shape's centroid relative to the shape's origin.
		public Vec2 center;

		/// The rotational inertia of the shape about the local origin.
		public float I;
	}
}
