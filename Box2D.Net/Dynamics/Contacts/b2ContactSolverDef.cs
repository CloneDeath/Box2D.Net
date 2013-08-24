using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct b2ContactSolverDef {
		public b2TimeStep step;
		public List<b2Contact> contacts; //pointer to a pointer
		//public int count;
		public List<b2Position> positions; //pointer
		public List<b2Velocity> velocities; //pointer
	}
}
