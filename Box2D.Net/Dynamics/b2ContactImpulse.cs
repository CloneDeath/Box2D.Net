﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Box2D {
	/// Contact impulses for reporting. Impulses are used instead of forces because
	/// sub-step forces may approach infinity for rigid body collisions. These
	/// match up one-to-one with the contact points in b2Manifold.
	public class b2ContactImpulse //was struct
	{
		public b2ContactImpulse() { }
		public b2ContactImpulse(int cnt) {
			this.count = cnt;
		}
		public List<float> normalImpulses = new List<float>();
		public List<float> tangentImpulses = new List<float>();
		public int count;
	};
}
