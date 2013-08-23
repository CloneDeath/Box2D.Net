using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D.Dynamics {
	/// Callback class for AABB queries.
	/// See b2World::Query
	abstract class b2QueryCallback
	{
		virtual ~b2QueryCallback() {}

		/// Called for each fixture found in the query AABB.
		/// @return false to terminate the query.
		public abstract bool ReportFixture(b2Fixture* fixture);
	}
}
