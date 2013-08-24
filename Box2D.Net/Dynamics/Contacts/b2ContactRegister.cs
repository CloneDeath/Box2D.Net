using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	public struct b2ContactRegister {
		public b2Contact.b2ContactCreateFcn createFcn;
		public b2Contact.b2ContactDestroyFcn destroyFcn;
		public bool primary;
	};
}
