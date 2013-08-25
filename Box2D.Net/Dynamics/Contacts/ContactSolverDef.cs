using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct ContactSolverDef {
		public TimeStep step;
		public List<Contact> contacts; //pointer to a pointer
		//public int count;
		public List<Position> positions; //pointer
		public List<Velocity> velocities; //pointer
	}
}
