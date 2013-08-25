using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Callback class for AABB queries.
	/// See World::Query
	public abstract class QueryCallback
	{
		~QueryCallback() {}

		/// Called for each fixture found in the query AABB.
		/// @return false to terminate the query.
		public abstract bool ReportFixture(Fixture fixture); //fixture was pointer
	}
}
