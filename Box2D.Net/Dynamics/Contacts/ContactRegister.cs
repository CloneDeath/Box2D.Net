﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	public struct ContactRegister {
		public Contact.ContactCreateFcn createFcn;
		public Contact.ContactDestroyFcn destroyFcn;
		public bool primary;
	};
}
