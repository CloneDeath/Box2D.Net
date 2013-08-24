using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	public enum EPAxisType {
		e_unknown,
		e_edgeA,
		e_edgeB
	};

	// This structure is used to keep track of the best separating axis.
	struct b2EPAxis {
		public EPAxisType type;
		public int index;
		public float separation;
	};
}
