﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Joints and fixtures are destroyed when their associated
	/// body is destroyed. Implement this listener so that you
	/// may nullify references to these joints and shapes.
	abstract class b2DestructionListener
	{
		public virtual ~b2DestructionListener() {}

		/// Called when any joint is about to be destroyed due
		/// to the destruction of one of its attached bodies.
		public abstract void SayGoodbye(b2Joint* joint);

		/// Called when any fixture is about to be destroyed due
		/// to the destruction of its parent body.
		public abstract void SayGoodbye(b2Fixture* fixture);
	}
}
