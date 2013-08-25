using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	// Output parameters for b2TimeOfImpact.
	public struct b2TOIOutput {
		public enum State {
			e_unknown,
			e_failed,
			e_overlapped,
			e_touching,
			e_separated
		};

		public State state;
		public float t;
	};
}
