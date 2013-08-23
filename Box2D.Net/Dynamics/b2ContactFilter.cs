using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Implement this class to provide collision filtering. In other words, you can implement
	/// this class if you want finer control over contact creation.
	abstract class b2ContactFilter
	{
		public virtual ~b2ContactFilter() {}

		/// Return true if contact calculations should be performed between these two shapes.
		/// @warning for performance reasons this is only called when the AABBs begin to overlap.
		public virtual bool ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB);
	};
}
