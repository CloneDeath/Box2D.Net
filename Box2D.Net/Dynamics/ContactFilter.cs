using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Implement this class to provide collision filtering. In other words, you can implement
	/// this class if you want finer control over contact creation.
	public class ContactFilter
	{
		/// Return true if contact calculations should be performed between these two shapes.
		/// @warning for performance reasons this is only called when the AABBs begin to overlap.
		public virtual bool ShouldCollide(Fixture fixtureA, Fixture fixtureB){
			Filter filterA = fixtureA.Filter;
			Filter filterB = fixtureB.Filter;

			if (filterA.GroupIndex == filterB.GroupIndex && filterA.GroupIndex != 0)
			{
				return filterA.GroupIndex > 0;
			}

			bool collide = (filterA.MaskBits & filterB.CategoryBits) != 0 && (filterA.CategoryBits & filterB.MaskBits) != 0;
			return collide;
		}
	}
}
