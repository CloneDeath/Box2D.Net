using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	struct b2ContactRegister {
		b2Contact.b2ContactCreateFcn createFcn;
		b2Contact.b2ContactDestroyFcn destroyFcn;
		bool primary;
	};
}
