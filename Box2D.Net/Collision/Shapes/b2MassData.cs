using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// This holds the mass data computed for a shape.
	struct b2MassData {
		/// The mass of the shape, usually in kilograms.
		float mass;

		/// The position of the shape's centroid relative to the shape's origin.
		b2Vec2 center;

		/// The rotational inertia of the shape about the local origin.
		float I;
	}
}
