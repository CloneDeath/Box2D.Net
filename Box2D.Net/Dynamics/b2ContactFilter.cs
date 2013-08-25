using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Implement this class to provide collision filtering. In other words, you can implement
	/// this class if you want finer control over contact creation.
	public abstract class ContactFilter
	{
		~ContactFilter() {}

		/// Return true if contact calculations should be performed between these two shapes.
		/// @warning for performance reasons this is only called when the AABBs begin to overlap.
		public abstract bool ShouldCollide(Fixture fixtureA, Fixture fixtureB);
	};
}
