﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Contact impulses for reporting. Impulses are used instead of forces because
	/// sub-step forces may approach infinity for rigid body collisions. These
	/// match up one-to-one with the contact points in b2Manifold.
	public struct b2ContactImpulse
	{
		public b2ContactImpulse(int cnt) {
			this.count = cnt;
			normalImpulses = new float[b2Settings.b2_maxManifoldPoints];
			tangentImpulses = new float[b2Settings.b2_maxManifoldPoints];
		}
		public float[] normalImpulses;
		public float[] tangentImpulses;
		public int count;
	};
}
